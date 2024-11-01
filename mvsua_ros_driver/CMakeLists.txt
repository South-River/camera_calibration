cmake_minimum_required(VERSION 2.8.3)
project(mvsua_ros_driver)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_BUILD_TYPE "Release")

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED)
find_package(OpenCV)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslib
  cv_bridge
  image_transport
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES robot_camera
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

include_directories(
  include 
  mvsua_camera
  mvsua_camera/mvSDK_v21
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRECTORIES}
)
add_executable(${PROJECT_NAME}
  camera_driver.cpp
  mvsua_camera/mvsua_camera.cpp
)
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${CMAKE_CURRENT_SOURCE_DIR}/mvsua_camera/mvSDK_v21/libMVSDK.so
)

add_executable(camera_trigger
  fps_ctrl.cpp
)
target_link_libraries(camera_trigger
  ${catkin_LIBRARIES}
)
