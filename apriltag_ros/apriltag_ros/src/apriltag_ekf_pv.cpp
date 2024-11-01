#include <Eigen/Dense>
#include <iostream>
#include "Eigen/src/Core/Matrix.h"
#include "Eigen/src/Geometry/Quaternion.h"
#include "ros/init.h"
#include "ros/rate.h"
#include "ros/ros.h"
#include "apriltag_ros/AprilTagDetectionArray.h"
#include <nav_msgs/Odometry.h>
#include <deque>
using namespace std;

ros::Subscriber ar_sub_;
ros::Subscriber imu_sub_uav;
ros::Subscriber imu_sub_ugv;
ros::Publisher  ekf_pvq_;
ros::Publisher  _pq_;

Eigen::Quaterniond q_uav;
Eigen::Matrix3d R_uav2w;
ros::Time T;

int const odom_window_num = 5;
int const vec_window_num = 5;
float const delta_t = 0.005;
std::deque<nav_msgs::Odometry> apriltag_data_queue_;
std::deque<nav_msgs::Odometry> apriltag_data2_queue_;
double last_x_0,last_y_0,last_z_0;

double last_x_1,last_y_1,last_z_1;
int flag=0;//1
double last_time_0,last_time_1;//2h

// State vector: [position (3), velocity (3), quaternion (4)]
typedef Eigen::Matrix<double, 6, 1> StateVector;
// // Control vector: [acceleration (3), angular velocity (3)]
// typedef Eigen::Matrix<double, 6, 1> ControlVector;
// Measurement vector: [relative position (3), relative quaternion (4)]
typedef Eigen::Matrix<double, 3, 1> MeasurementVector;

const int STATE_DIM = 6;
const int MEASUREMENT_DIM = 3;

// Process and measurement noise covariance matrices
Eigen::Matrix<double, MEASUREMENT_DIM, MEASUREMENT_DIM> Q;
Eigen::Matrix<double, STATE_DIM, MEASUREMENT_DIM> W;
Eigen::Matrix<double, MEASUREMENT_DIM, MEASUREMENT_DIM> R;

// Initial state
StateVector x = StateVector::Zero();

// Initial covariance matrix
Eigen::Matrix<double, STATE_DIM, STATE_DIM> P = Eigen::Matrix<double, STATE_DIM, STATE_DIM>::Identity();

// Example control input (acceleration and angular velocity)
// 这里接受来自两个imu的相对控制量a，w
// ControlVector u,u_uav,u_ugv;

// Example measurement (relative position and quaternion from Apriltag)
// 这里接受来自Apriltag的p，q //v
MeasurementVector z,z_last,z_error,z_temp;

double dt = 0.1; // time step

// StateVector f(const StateVector& x, const ControlVector& u, double dt) {
//     StateVector x_new = x;

//     // Position update
//     x_new.segment<3>(0) += x.segment<3>(3) * dt + 0.5 * u.segment<3>(0) * dt * dt;
//     // Velocity update
//     x_new.segment<3>(3) += u.segment<3>(0) * dt;
//     // Quaternion update (simplified, assuming small angles)
//     Eigen::Quaterniond q(x(6), x(7), x(8), x(9));
//     Eigen::Quaterniond dq(1, u(3) * dt / 2, u(4) * dt / 2, u(5) * dt / 2);
//     Eigen::Quaterniond q_new = (q * dq).normalized();
//     x_new.segment<4>(6) = Eigen::Vector4d(q_new.w(), q_new.x(), q_new.y(), q_new.z());

//     return x_new;
// }

Eigen::Matrix<double, MEASUREMENT_DIM, STATE_DIM> H_jacobian(const StateVector& x) {
    Eigen::Matrix<double, MEASUREMENT_DIM, STATE_DIM> H = Eigen::Matrix<double, MEASUREMENT_DIM, STATE_DIM>::Zero();
    // H.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity();
    // H.block<4, 4>(3, 6) = Eigen::Matrix4d::Identity();
    H.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity();
    return H;
}

MeasurementVector h(const StateVector& x) {
    MeasurementVector z1;
    z1.segment<3>(0) = x.segment<3>(0);
    // z1.segment<4>(3) = x.segment<4>(6);
              //z1.segment<3>(0) = x.segment<3>(3);
    return z1;
}

void ekf_update(StateVector& x, Eigen::Matrix<double, STATE_DIM, STATE_DIM>& P, const MeasurementVector& z, const double dt) {
    // Predict
    // StateVector x_pred = f(x,  dt);
    Eigen::Matrix<double, STATE_DIM, STATE_DIM> F = Eigen::Matrix<double, STATE_DIM, STATE_DIM>::Identity();
    F.block<3, 3>(0, 3) = Eigen::Matrix3d::Identity() * dt;
    StateVector x_pred = F*x;
    Eigen::Matrix<double, STATE_DIM, STATE_DIM> P_pred = F * P * F.transpose() + W * Q * W.transpose();

    // Update
    Eigen::Matrix<double, MEASUREMENT_DIM, STATE_DIM> H = H_jacobian(x_pred);
    MeasurementVector z_pred = h(x_pred);
    Eigen::Matrix<double, MEASUREMENT_DIM, MEASUREMENT_DIM> S = H * P_pred * H.transpose() + R;
    Eigen::Matrix<double, STATE_DIM, MEASUREMENT_DIM> K = P_pred * H.transpose() * S.inverse();

    x = x_pred + K * (z - z_pred);
    //x(2)=z(2);
    P = (Eigen::Matrix<double, STATE_DIM, STATE_DIM>::Identity() - K * H) * P_pred;
    //发布相应话题
    nav_msgs::Odometry x_efk_;
    x_efk_.header.stamp=ros::Time::now();
    x_efk_.header.frame_id="world";
    x_efk_.pose.pose.position.x=x(0);
    x_efk_.pose.pose.position.y=x(1);
    x_efk_.pose.pose.position.z=x(2);
    x_efk_.twist.twist.linear.x=x(3);
    x_efk_.twist.twist.linear.y=x(4);
    x_efk_.twist.twist.linear.z=x(5);
    // x_efk_.pose.pose.orientation.w=x(6);
    // x_efk_.pose.pose.orientation.x=x(7);
    // x_efk_.pose.pose.orientation.y=x(8);
    // x_efk_.pose.pose.orientation.z=x(9);

    ekf_pvq_.publish(x_efk_);

}

void aprilatg_callback(const apriltag_ros::AprilTagDetectionArray::ConstPtr& msg)
  {
      if(msg->detections.size()>0)
      {
            z(0) = msg->detections[0].pose.pose.pose.position.x;
            z(1) = msg->detections[0].pose.pose.pose.position.y;            
            z(2) = msg->detections[0].pose.pose.pose.position.z;
            z(3) = msg->detections[0].pose.pose.pose.orientation.w;
            z(4) = msg->detections[0].pose.pose.pose.orientation.x;
            z(5) = msg->detections[0].pose.pose.pose.orientation.y;
            z(6) = msg->detections[0].pose.pose.pose.orientation.z;
	    	 //cout<<"Point position:"<<endl;
	    	 //cout<<"musement_data: "<<z<<endl;    
             }
    	 else 
  	{//cout<<"未检测到tag"<<endl;
    }
   }
  void number_callback(const apriltag_ros::AprilTagDetectionArray::ConstPtr& msg)
  {
    if(msg->detections.size()>0)
        {
            T=ros::Time::now();
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
                //_pq_.publish(odom_pvq);
                
                
            //提取转换后的位姿信息
            z(0) = odom_pvq.pose.pose.position.x;
            z(1) = odom_pvq.pose.pose.position.y;            
            z(2) = odom_pvq.pose.pose.position.z;
            // z_error=z-z_temp;
            // double error_x=z_error(0)/z(0);
            // if (error_x<5) {
            //     z=z_temp;
            // }

            // z(0)=P1(0);
            // z(1)=P1(1);
            // z(2)=P1(2);

            // z(3) = Q1.w();
            // z(4) = Q1.x();
            // z(5) = Q1.y();
            // z(6) = Q1.z();
             
        //}
        }
    else 
  	    {
        // cout<<"未检测到tag"<<endl;
        }
  	    }
  

int main(int argc, char **argv) {
    ros::init(argc, argv,"apriltag_imu_ekf");
    ros::NodeHandle nh;
    x(6) = 1.0; // Initial quaternion is identity
    // Process noise covariance matrix
    Q = Eigen::Matrix<double, MEASUREMENT_DIM, MEASUREMENT_DIM>::Identity() ;
    Q(0,0)=4;
    Q(1,1)=4;
    Q(2,2)=1;

    // W matrix
    double t2 = dt * dt / 2;
    W(0, 0) = t2;
    W(1, 1) = t2;
    W(2, 2) = t2;
    W(3, 0) = dt;
    W(4, 1) = dt;
    W(5, 2) = dt;
    // Measurement noise covariance matrixz
    R = Eigen::Matrix<double, MEASUREMENT_DIM, MEASUREMENT_DIM>::Identity()*0.1 ;
    // R(0,0)=4;
    // R(1,1)=4;
    // R(2,2)=1;
    // 初始化控制变量和观测变量
    // u  << 0.1, 0.2, 0.3, 0.01, 0.02, 0.03;
    // u_uav << 0.1, 0.2, 0.3, 0.01, 0.02, 0.03;
    // u_ugv << 0.1, 0.2, 0.3, 0.01, 0.02, 0.03;
    //z << 1.0,1.0,1.0; // assuming a simple measurement for demonstration
    // 接受Apriltag数据
    ar_sub_ = nh.subscribe<apriltag_ros::AprilTagDetectionArray>("/tag_detections", 1, number_callback,ros::TransportHints().tcpNoDelay());
    ekf_pvq_ = nh.advertise<nav_msgs::Odometry>("ekf_v_", 1);
    //_pq_ = nh.advertise<nav_msgs::Odometry>("oringin_pq", 1);
    // 加IMU测量得到的相对a和w值
    // imu_sub_uav = nh.subscribe<sensor_msgs::Imu>("/mavros/imu/data", 100, uav_imu);
    // imu_sub_ugv = nh.subscribe<sensor_msgs::Imu>("/ugv0/imu", 100, ugv_imu);
    ros::Rate loop_rate(100);
    //接收话题
    while (ros::ok()) {

    // EKF update step
    ekf_update(x, P, z, dt);
                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                   
    // Output the updated state
    //std::cout << "Updated state: " << x.transpose() << std::endl;
    ros::spinOnce();
    loop_rate.sleep();
    }
 

    return 0;
}