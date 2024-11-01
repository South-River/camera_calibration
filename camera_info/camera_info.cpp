
#include <ros/ros.h>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <sensor_msgs/CameraInfo.h>
#include <math.h>

using namespace std;
using namespace cv;

sensor_msgs::CameraInfo getCameraInfo(void){        // extract cameraInfo.
    sensor_msgs::CameraInfo cam;

    vector<double> D{0.010922, -0.000855, -0.021676, 0.015643, 0.000000};
    boost::array<double, 9> K = {
        390.77206,   0.     , 622.97068,
           0.     , 390.84936, 502.86414,
           0.     ,   0.     ,   1.        
    };
    
     boost::array<double, 12> P = {
        417.56418,   0.     , 671.28527,   0.     ,
           0.     , 411.46283, 446.41384,   0.     ,
           0.     ,   0.     ,   1.     ,   0.     
    };
    boost::array<double, 9> r = {1, 0, 0, 0, 1, 0, 0, 0, 1};

    cam.height = 1024;
    cam.width = 1280;
    cam.distortion_model = "plumb_bob";
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

int main(int argc, char** argv) {

  ros::init(argc, argv, "camera_info");  //初始化了一个节点，名字为camera_info
  ros::NodeHandle n;
  ros::Publisher pub = n.advertise<sensor_msgs::CameraInfo>("/cam_topic/rect/camera_info", 1000);  
  sensor_msgs::CameraInfo camera_info_dyn;
  ros::Rate rate(30);  //点云更新频率0.5Hz

  while (ros::ok())
  {
      camera_info_dyn = getCameraInfo();
      pub.publish(camera_info_dyn); //发布出去
      rate.sleep();
  }
    ros::spin();
    return 0;
}


