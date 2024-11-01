#include <ros/ros.h>

#include <std_msgs/Bool.h>

int main(int ac, char** av)
{
    ros::init(ac, av, "fps_ctrl");
    ros::NodeHandle nh("~");

    double fps;
    while (!nh.getParam("fps", fps))
    {
        ROS_ERROR("fps not set");
        ros::Duration(0.1).sleep();
    }

    ros::Publisher cam_trigger_pub = nh.advertise<std_msgs::Bool>("/time_synchronization_camera_msgs", 1);

    std_msgs::Bool trig_msg;
    trig_msg.data = true;

    ros::Timer cam_trigger_timer = nh.createTimer(ros::Duration(1.0/fps), 
            [&cam_trigger_pub, &trig_msg]
            (const ros::TimerEvent&){
                cam_trigger_pub.publish(trig_msg);
    });

    ros::spin();
    return 0;
}