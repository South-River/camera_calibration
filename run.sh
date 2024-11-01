#! /bin/bash

gnome-terminal -- bash -c "cd /home/yjj/usb_cam; source devel/setup.sh; roslaunch usb_cam usb_cam-test.launch; exec bash"

echo “usb_cam-test  successfully started”

sleep 1s  

gnome-terminal -- bash -c "ROS_NAMESPACE=usb_cam rosrun image_proc image_proc; exec bash"

echo “image_proc  successfully started”

sleep 0.5s

gnome-terminal -- bash -c "cd /home/yjj/usb_cam; source devel/setup.sh; roslaunch apriltag_ros continuous_detection.launch; exec bash"

echo “apriltag_ros  successfully started”

sleep 0.5s

gnome-terminal -- bash -c "cd /home/yjj/usb_cam; source devel/setup.sh;  rosrun apriltag_ros apriltag_sub; exec bash"

echo “apriltag_sub  successfully started”

