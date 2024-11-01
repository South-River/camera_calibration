
# 相机的标定/校正之逅
rosrun camera_calibration cameracalibrator.py --size 11x8 --square 0.030 image:=/cam_topic/rect

# 相机的标定/校正之前
rosrun camera_calibration cameracalibrator.py --size 11x8 --square 0.030 image:=/cam_topic

###运行相机驱动
roslaunch mvsua_ros_driver camera_driver.launch 

### 运行image_proc去畸变，得到去畸变后的图像/usb_cam/image_rect_color
ROS_NAMESPACE=cam_topic rosrun image_proc image_proc

### 使用Apriltag_ros包
roslaunch apriltag_ros continuous_detection.launch

### 4查看效果
#查看相机位姿信息
rostopic echo /tag_detections
#打开rqt_image_view/在rviz里添加topic
