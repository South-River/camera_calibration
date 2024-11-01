#include "ros/ros.h"
#include "apriltag_ros/AprilTagDetectionArray.h"
#include "iostream"
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <nav_msgs/Odometry.h>
#include <Eigen/QR>  
using namespace std;

ros::Subscriber ar_sub_;
ros::Publisher ar_pub_;
 
class Localizer
{
public:
  Localizer(ros::NodeHandle& nh)
  {
      ar_pub_ = nh.advertise<nav_msgs::Odometry>("/tag_detections2",1);
      ar_sub_ = nh.subscribe<apriltag_ros::AprilTagDetectionArray>("/tag_detections", 1, & Localizer::number_callback, this);
  }

  void number_callback(const apriltag_ros::AprilTagDetectionArray::ConstPtr& msg)
  {
    if(msg->detections.size()>0)
        {
            //将tag_detections的消息提取出来
            Eigen::Vector3d P0;
            P0(0)  = msg->detections[0].pose.pose.pose.position.x;
            P0(1)  = msg->detections[0].pose.pose.pose.position.y;
            P0(2)  = msg->detections[0].pose.pose.pose.position.z;
            Eigen::Quaterniond Q0;
            Q0.w() = msg->detections[0].pose.pose.pose.orientation.w;
            Q0.x() = msg->detections[0].pose.pose.pose.orientation.x;
            Q0.y() = msg->detections[0].pose.pose.pose.orientation.y;
            Q0.z() = msg->detections[0].pose.pose.pose.orientation.z;
            Q0.normalize();
            Eigen::Matrix3d R0;
            //将四元数转为旋转矩阵
            R0=Q0.matrix();
            // //定义4x4变换e矩阵
            Eigen::Matrix<double,3,1> zero={0,0,0};
            // zero.Zero().transpose();
            Eigen::Matrix4d T0;
            T0.block(0,0,3,3)=R0;
            T0.block(0,3,3,1)=P0;
            T0.block(3,0,1,3)=zero;
            T0(3,3)=1;

            //将T矩阵进行求逆
            Eigen::Matrix4d T1=T0.completeOrthogonalDecomposition().pseudoInverse();
            // Eigen::Matrix4d T1=T0.inverse();
            // T1=-T1;
            //转换至动捕标定系下
            Eigen::Matrix4d R_x,R_z,R_z_inverse;
            R_x<<1,0,0,0,
                 0,0,-1,0,
                 0,1,0,0,
                 0,0,0,1;
            R_z<<0,-1,0,0,
                 1,0,0,0,
                 0,0,1,0,
                 0,0,0,1;
            R_z_inverse<<0,1,0,0,
                         -1,0,0,0,
                         0,0,1,0,
                         0,0,0,1;
            // Eigen::Matrix4d T2=R_z_inverse*T1*R_x*R_z;
            
            // ME
            Eigen::Matrix4d T2=R_z_inverse*T1*R_x*R_z;

            Eigen::Matrix3d R1=T2.block(0,0,3,3);
            Eigen::Vector3d P1=T2.block(0,3,3,1);
            // Eigen::Matrix3d R1=R0.inverse();
            //将旋转矩阵转为四元素
            Eigen::Quaterniond Q1 = Eigen::Quaterniond(R1);
            Q1.normalize();
            //提取转换后的位姿信息并且以/tag_detections2的话题名字发出来
            nav_msgs::Odometry odom;
            odom.header.stamp=ros::Time::now();
            odom.header.frame_id="world";
            odom.pose.pose.position.x=T2(0,3);
            odom.pose.pose.position.y=T2(1,3);
            odom.pose.pose.position.z=T2(2,3);
            // odom.pose.pose.position.x=-P0(0);
            // odom.pose.pose.position.y=P0(1);
            // odom.pose.pose.position.z=P0(2);
            odom.pose.pose.orientation.w=Q1.w();
            odom.pose.pose.orientation.x=Q1.x();
            odom.pose.pose.orientation.y=Q1.y();
            odom.pose.pose.orientation.z=Q1.z();

            ar_pub_.publish(odom);
            // cout<<"T0"<<endl<<T0<<endl;
            // cout<<"T1"<<endl<<T1<<endl;
            // cout<<"T2"<<endl<<T2<<endl;                
        }
    else 
  	    {
        // cout<<"未检测到tag"<<endl;
        }
  	    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv,"apriltag_detector_subscriber");
    ros::NodeHandle node_obj;
    Localizer localizer(node_obj);    
    ROS_INFO("start");
    ros::spin();
    return 0;
}

