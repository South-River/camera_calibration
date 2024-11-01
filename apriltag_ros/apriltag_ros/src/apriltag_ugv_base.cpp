#include "ros/ros.h"
#include "apriltag_ros/AprilTagDetectionArray.h"
#include "iostream"
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <nav_msgs/Odometry.h>
#include <Eigen/QR>  
#include <deque>
#include <ostream>
using namespace std;

ros::Subscriber ar_sub_;
ros::Subscriber ar_sub2_;
ros::Publisher ar_pub_;
ros::Publisher ar_pub2_;

int const odom_window_num = 10;
int const vec_window_num = 10;
float const delta_t = 0.005;
std::deque<nav_msgs::Odometry> apriltag_data_queue_;
std::deque<nav_msgs::Odometry> apriltag_data2_queue_;
double last_x_0,last_y_0,last_z_0;
double last_x_1,last_y_1,last_z_1;
int flag=0;//1
double last_time_0,last_time_1;//2h

class Localizer
{
public:
  Localizer(ros::NodeHandle& nh)
  {
      ar_pub_ = nh.advertise<nav_msgs::Odometry>("/tag_detections3",1);
      ar_pub2_ = nh.advertise<nav_msgs::Odometry>("/vrpn_client_node/yjj_apriltag/pose2",1);
      ar_sub_ = nh.subscribe<apriltag_ros::AprilTagDetectionArray>("/tag_detections", 1, & Localizer::number_callback, this);
      ar_sub2_ = nh.subscribe<nav_msgs::Odometry>("/vrpn_client_node/yjj_apriltag/pose", 1, & Localizer::number_callback2, this);
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
            Eigen::Matrix4d R_x,R_z,T_xz,R_z_inverse;
            R_x<<1,0,0,0,
                 0,-1,0,0,
                 0,0,-1,0,
                 0,0,0,1;
            R_z<<0,-1,0,0,
                 1,0,0,0,
                 0,0,1,0,
                 0,0,0,1;
            T_xz<<1,0,0,0.04,
                  0,1,0,0,
                  0,0,1,0.125,
                  0,0,0,1;
            R_z_inverse<<0,1,0,0,
                         -1,0,0,0,
                         0,0,1,0,
                         0,0,0,1;
            // Eigen::Matrix4d T2=R_z_inverse*T1*R_x*R_z;
            
            // 
            Eigen::Matrix4d T2=R_z_inverse*T1 *R_x*R_z*T_xz;

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

            if (flag==0) {

                 last_x_0=odom.pose.pose.position.x;
                 last_y_0=odom.pose.pose.position.y;
                 last_z_0=odom.pose.pose.position.z;
                 flag=1;
                 last_time_0=ros::Time::now().toSec();
            }
            else if (flag==1) {
                 last_x_1=odom.pose.pose.position.x;
                 last_y_1=odom.pose.pose.position.y;
                 last_z_1=odom.pose.pose.position.z;
                 flag=2;
                 last_time_1=ros::Time::now().toSec();
            }
            //第三次观测再进行中心差分导数
            else {
                //划窗优化
                nav_msgs::Odometry odom_pvq;
                odom_pvq.header.stamp=ros::Time::now();
                odom_pvq.header.frame_id="world";
                if (apriltag_data_queue_.size()>=odom_window_num) {

                    apriltag_data_queue_.pop_front();
                
                }
                apriltag_data_queue_.push_back(odom);
                //把最大最小值去掉?
                auto n=apriltag_data_queue_.size();

                for (const auto &item : apriltag_data_queue_) {

                    odom_pvq.pose.pose.position.x+=item.pose.pose.position.x/n;
                    odom_pvq.pose.pose.position.y+=item.pose.pose.position.y/n;
                    odom_pvq.pose.pose.position.z+=item.pose.pose.position.z/n;

                }

                // //进行微分操作
                // odom_pvq.twist.twist.linear.x=(odom_pvq.pose.pose.position.x-last_x)/delta_t;
                // odom_pvq.twist.twist.linear.y=(odom_pvq.pose.pose.position.y-last_y)/delta_t;
                // odom_pvq.twist.twist.linear.z=(odom_pvq.pose.pose.position.z-last_Z)/delta_t;

                //Central derivative: dy/dx ~= f(x+delta_x)-f(x-delta_x)/(2*delta_x)
                double dt=ros::Time::now().toSec()-last_time_0;
                odom_pvq.twist.twist.linear.x=(odom_pvq.pose.pose.position.x-last_x_0)/dt;
                odom_pvq.twist.twist.linear.y=(odom_pvq.pose.pose.position.y-last_y_0)/dt;
                odom_pvq.twist.twist.linear.z=(odom_pvq.pose.pose.position.z-last_z_0)/dt;

                //速度的划窗优化
                nav_msgs::Odometry odom_pvq2;
                odom_pvq2.header.stamp=odom_pvq.header.stamp;
                odom_pvq2.header.frame_id="world";
                if (apriltag_data2_queue_.size()>=vec_window_num) {

                    apriltag_data2_queue_.pop_front();
                
                }
                apriltag_data2_queue_.push_back(odom_pvq);
                //把最大最小值去掉?
                auto n2=apriltag_data2_queue_.size();

                for (const auto &item : apriltag_data2_queue_) {

                    odom_pvq2.twist.twist.linear.x+=item.twist.twist.linear.x/n2;
                    odom_pvq2.twist.twist.linear.y+=item.twist.twist.linear.y/n2;
                    odom_pvq2.twist.twist.linear.z+=item.twist.twist.linear.z/n2;

                }

                last_time_0=last_time_1;
                last_x_0=last_x_1;
                last_y_0=last_y_1;
                last_z_0=last_z_1;

                last_time_1=ros::Time::now().toSec();
                odom_pvq2.pose.pose.position.x=odom_pvq.pose.pose.position.x;
                odom_pvq2.pose.pose.position.y=odom_pvq.pose.pose.position.y;
                odom_pvq2.pose.pose.position.z=odom_pvq.pose.pose.position.z;
                last_x_1=odom_pvq2.pose.pose.position.x;
                last_y_1=odom_pvq2.pose.pose.position.y;
                last_z_1=odom_pvq2.pose.pose.position.z;
                
                ar_pub_.publish(odom_pvq2);    

            }
                      
        }
    else 
  	    {
        // cout<<"未检测到tag"<<endl;
        }
  	    }
    void number_callback2(const nav_msgs::Odometry::ConstPtr& msg)
    {
            nav_msgs::Odometry odom2;
            odom2.header.stamp=ros::Time::now();
            odom2.header.frame_id="world";
            odom2.pose.pose.position.x=msg->pose.pose.position.x;
            odom2.pose.pose.position.y=msg->pose.pose.position.y;
            odom2.pose.pose.position.z=msg->pose.pose.position.z;
            odom2.pose.pose.orientation.w=msg->pose.pose.orientation.w;
            odom2.pose.pose.orientation.x=msg->pose.pose.orientation.x;
            odom2.pose.pose.orientation.y=msg->pose.pose.orientation.y;
            odom2.pose.pose.orientation.z=msg->pose.pose.orientation.z;
            ar_pub2_.publish(odom2);
  	 }
};


int main(int argc, char **argv)
{
    ros::init(argc, argv,"apriltag_ugv_base");
    ros::NodeHandle node_obj;
    Localizer localizer(node_obj);    
    ROS_INFO("start");
    ros::spin();
    return 0;
}

