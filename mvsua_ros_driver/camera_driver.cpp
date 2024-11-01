#include <ros/ros.h>
#include <ros/package.h>
#include <signal.h>
#include <sensor_msgs/CameraInfo.h>

#include <iostream>
#include <math.h>


#include <opencv2/opencv.hpp>
#include "cv_bridge/cv_bridge.h"
#include <image_transport/image_transport.h>
#include "opencv2/videoio/videoio_c.h"

#include "mvsua_camera.h"
#include "camera_driver.h"
#include "tic_toc.h"
#include <std_msgs/Bool.h>

using namespace std;
using namespace cv;

int width, height, camera_num, cap_fps[MVSUA_CAMERA_MAX] = {0};
int pixel_bit;
std::string image_format;
std::string format_saveshow;
std::string format_pub;
double freq;
image_transport::Publisher pub_raw[MVSUA_CAMERA_MAX];

ros::Publisher cam_info_pub;


ros::Time last_reset_time[MVSUA_CAMERA_MAX];     // reset开始电脑时间
double get_half_exposure_time[MVSUA_CAMERA_MAX]; // s  曝光值一半

void SignalHandler(int signal)
{
  ros::shutdown();
}

sensor_msgs::CameraInfo getCameraInfo(void){        // extract cameraInfo.
    sensor_msgs::CameraInfo cam;

    vector<double> D{-0.010118, 0.002379, -0.004120, 0.000608};
    boost::array<double, 9> K = {
        385.19293,   0.68782, 606.10314,
           0.     , 385.46787, 526.97707,
           0.     ,   0.     ,   1.    
    };
    
     boost::array<double, 12> P = {
        385.19293,   0.68782, 606.10314,   0.     ,
           0.     , 385.46787, 526.97707,   0.     ,
           0.     ,   0.     ,   1.     ,   0.    
    };
    boost::array<double, 9> r = {1, 0, 0, 0, 1, 0, 0, 0, 1};

    cam.height = 1024;
    cam.width = 1280;
    cam.distortion_model = "equidistant";
    cam.D = D;
    cam.K = K;
    cam.P = P;
    cam.R = r;
    cam.binning_x = 0;
    cam.binning_y = 0;
    cam.header.frame_id = "cam_topic";  //frame_id为camera，也就是相机名字
    cam.header.stamp = ros::Time::now();
    cam.header.stamp.nsec = 0;
    return cam;
}

void ImageCallback(int camera_id, cv::Mat &image)
{
  // 发布image图像话题
  sensor_msgs::ImagePtr msg;
  std_msgs::Header header;

  ros::Time time_finish_exposure; // NOTE: 相机时间： 当前帧完成曝光时刻

  // 计算曝光时间戳
  uint64_t uTimestamp;
  UINT TimeStampH, TimeStampL;
  if (CameraGetFrameTimeStamp(camera_id, &TimeStampL, &TimeStampH) == CAMERA_STATUS_SUCCESS) // 获取相机时间戳
  {
    uTimestamp = (((uint64_t)TimeStampH) << 32) + (uint64_t)TimeStampL; // us
    time_finish_exposure.fromSec(0.000001 * uTimestamp);                // s
  }

  // 考虑时延 NOTE:  曝光中间时刻为时间戳: Stamp = 上一次reset时刻+当前帧完成曝光的相机时间(距离上一次reset时间间隔)-曝光时长一半
  header.stamp = last_reset_time[camera_id - 1] + ros::Duration(time_finish_exposure.toSec() - get_half_exposure_time[camera_id - 1]);

  // 如果格式是mono8，就把cv图像转成ros消息；否则报错
  if (image_format == "mono8")
  {
    cv::Mat rgb_image;
    cv::cvtColor(image, rgb_image, cv::COLOR_GRAY2RGB); // 将灰度图转换为RGB彩色图
    msg = cv_bridge::CvImage(header, "rgb8", rgb_image).toImageMsg();
  }
  else
  {
    ROS_ERROR("format is not mono8!!! \n");
  }

  // 对应id的相机发布消息
  pub_raw[camera_id - 1].publish(msg);

  // 获取并发布相机信息
  sensor_msgs::CameraInfo cam_info = getCameraInfo();
  cam_info.header.stamp = header.stamp;  // 将时间戳与图像保持一致
  cam_info_pub.publish(cam_info);
}

void timeCallback(const std_msgs::Bool &time_msg)
{
  for (int hCamera = 1; hCamera <= camera_num; hCamera++)
  {
    if (CameraRstTimeStamp(hCamera) == CAMERA_STATUS_SUCCESS)
    {
      last_reset_time[hCamera - 1] = ros::Time::now();
    }
    if (CameraSoftTrigger(hCamera) != CAMERA_STATUS_SUCCESS)
    {
      // 如果软触发失败，ros报错
      // ROS_ERROR("Can't Capture Image");
    }
  }
}

int main(int argc, char **argv)
{
  signal(SIGINT, SignalHandler);
  signal(SIGTERM, SignalHandler);
  ros::init(argc, argv, "mvsua_ros_driver_node", ros::init_options::NoSigintHandler);
  ros::NodeHandle ros_nh("~");
  ros::Subscriber xxx = ros_nh.subscribe("time_synchronization_camera_msgs", 1, timeCallback);
  // 初始化相机参数
  ros_nh.param("camera_num", camera_num, 1);                              // 相机数量1
  ros_nh.param("pixel_bit", pixel_bit, 255);                              // 相机位深255
  ros_nh.param<std::string>("format", image_format, "bgr8");                    // format:bgr
  ros_nh.param<std::string>("format_saveshow", format_saveshow, "mono8"); // 图像存储格式：mono8
  ros_nh.param<std::string>("format_pub", format_pub, "mono8");           // 图像发布格式：mono8
  ros_nh.param<double>("freq", freq, 30.);                                // 相机频率30
  camera_driver::mvsua_camera camera(camera_num, pixel_bit, image_format);      // 初始化相机
  camera_num = camera.iCameraCounts_;                                     // 相机数量赋给camera_num，相机数量由API中的初始化函数得到

  image_transport::ImageTransport it(ros_nh); // cv与img_topic转化
  // 每个相机发布需要订阅对应的话题(cv转ros的消息)
  for (int cap = 1; cap <= camera.iCameraCounts_; cap++)
  {
    pub_raw[cap - 1] = it.advertise("mvsua_cam/image_raw" + std::to_string(cap), 1);
  }

  cam_info_pub = ros_nh.advertise<sensor_msgs::CameraInfo>("/cam_topic/camera_info", 1);


  int analog_gain, gamma, saturation, shaprpness, w_offset, h_offset, speed, autoexposure_target;
  double exposure_time = 2;

  for (int cap = 1; cap <= camera.iCameraCounts_; cap++)
  {
    ros_nh.param("MVSUA" + std::to_string(cap) + "/analog_gain", analog_gain, 50);                 // 模拟增益
    ros_nh.param("MVSUA" + std::to_string(cap) + "/gamma", gamma, 50);                             // 设定LUT动态生成模式下的Gamma值
    ros_nh.param("MVSUA" + std::to_string(cap) + "/saturation", saturation, 100);                  // 图像饱和度
    ros_nh.param("MVSUA" + std::to_string(cap) + "/shaprpness", shaprpness, 0);                    // 图像锐化参数
    ros_nh.param("MVSUA" + std::to_string(cap) + "/width", width, 752);                            // 图像宽度
    ros_nh.param("MVSUA" + std::to_string(cap) + "/height", height, 480);                          // 图像高度
    ros_nh.param("MVSUA" + std::to_string(cap) + "/w_offset", w_offset, 0);                        // 横向偏移
    ros_nh.param("MVSUA" + std::to_string(cap) + "/h_offset", h_offset, 0);                        // 纵向偏移
    ros_nh.param("MVSUA" + std::to_string(cap) + "/speed", speed, 0);                              // 相机频率
    ros_nh.param("MVSUA" + std::to_string(cap) + "/autoexposure_target", autoexposure_target, 50); // 自动曝光模式

    while (!ros_nh.getParam("exposure_time", exposure_time))
    {
      ROS_WARN("[cam_driver] waiting for exposure_time");
      ros::Duration(0.1).sleep();
    }

    camera.LoadParam(cap, analog_gain, gamma, saturation, shaprpness, width, height, w_offset, h_offset, speed, autoexposure_target, exposure_time); // 载入参数

    double pfExposureTime;
    CameraGetExposureTime(cap, &pfExposureTime);                                              // 获得每个相机的曝光时间(for循环可以遍历每个相机)
    get_half_exposure_time[cap - 1] = pfExposureTime * 0.000001 * 0.5;                        // s
    std::cout << "success set exposure time " << pfExposureTime * 0.001 << "ms" << std::endl; // 启动此节点后会输出一次；"success set exposure time"和曝光时间
  }

  ros::spin();
  return 0;
}