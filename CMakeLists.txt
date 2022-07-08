cmake_minimum_required(VERSION 3.0.2)
project(trans_msg_time)
find_package(catkin REQUIRED COMPONENTS 
 roscpp
 rosbag
 sensor_msgs 
 livox_ros_driver
 )
 find_package(OpenCV 4 REQUIRED COMPONENTS )
 catkin_package()
 include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OPENCV_INCLUDE_DIRS}
 )
 add_executable(trans_msg_time_node trans_msg_time.cpp )
target_link_libraries(trans_msg_time_node
                      ${catkin_LIBRARIES} 
                      ${OpenCV_LIBS}
)
