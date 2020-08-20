cmake_minimum_required(VERSION 3.1.3)
project(UAV-ARM-control)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  std_srvs
  sensor_msgs
  actionlib
  actionlib_msgs
  control_msgs 
  move_base_msgs
  trajectory_msgs
  cmake_modules
  moveit_core
  moveit_msgs
  moveit_ros_planning_interface
)
 
catkin_package(
  INCLUDE_DIRS 
  CATKIN_DEPENDS std_srvs roscpp std_msgs sensor_msgs geometry_msgs nav_msgs actionlib  move_base_msgs  control_msgs trajectory_msgs interactive_markers moveit_core moveit_ros_perception moveit_ros_planning_interface pluginlib   actionlib_msgs 
)
 
include_directories(
   ${catkin_INCLUDE_DIRS} 	
)
 
link_directories(
  ${catkin_LIB_DIRS} 
)
 
 

add_executable(my_arm_control src/my_arm_control.cpp)
target_link_libraries(my_arm_control
  ${catkin_LIBRARIES}  
)
add_executable(space_to_move src/space_to_move.cpp)
target_link_libraries(space_to_move
  ${catkin_LIBRARIES}  
)