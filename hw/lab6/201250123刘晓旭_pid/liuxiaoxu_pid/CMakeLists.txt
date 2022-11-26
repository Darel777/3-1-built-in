cmake_minimum_required(VERSION 3.0.2)
project(liuxiaoxu_pid)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  nav_msgs
  ackermann_msgs
  message_generation
  roslaunch
)
roslaunch_add_file_check(launch)
add_message_files(
  FILES
  scan_range.msg
)
generate_messages(
  DEPENDENCIES
  std_msgs
)
catkin_package()
include_directories(include ${catkin_INCLUDE_DIRS})
add_executable(wall_follow src/wall_follow.cpp)
target_link_libraries(wall_follow ${catkin_LIBRARIES})
catkin_package(

)
include_directories(
  ${catkin_INCLUDE_DIRS}
)