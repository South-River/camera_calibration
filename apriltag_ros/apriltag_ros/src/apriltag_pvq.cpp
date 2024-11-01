#include "ros/ros.h"
#include "apriltag_ros/AprilTagDetectionArray.h"
#include "iostream"

using namespace std;

ros::Subscriber ar_sub_;
 
class Localizer
{
public:
  Localizer(ros::NodeHandle& nh)
  {
      ar_sub_ = nh.subscribe<apriltag_ros::AprilTagDetectionArray>("/tag_detections", 1, &Localizer::number_callback, this);
  }

  void number_callback(const apriltag_ros::AprilTagDetectionArray::ConstPtr& msg)
  {
      if(msg->detections.size()>0)
      {
             float getX = msg->detections[0].pose.pose.pose.position.x;
             
	    	 cout<<"Point position:"<<endl;
	    	 cout<<"cam_getX: "<<getX<<endl;    }
    	 else 
  	{cout<<"未检测到tag"<<endl;}
  	 }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv,"apriltag_detector_subscriber");
    ros::NodeHandle node_obj;
    Localizer localizer(node_obj);    
    ROS_INFO("节点开始");
    ros::spin();
    return 0;
}

