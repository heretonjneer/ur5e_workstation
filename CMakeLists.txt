cmake_minimum_required(VERSION 2.8.3)
project(ur5e_workstation)

add_compile_options(-std=c++11)

find_package(Eigen3 REQUIRED)

if(NOT EIGEN3_INCLUDE_DIRS)
  set(EIGEN3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})
endif()

find_package(catkin REQUIRED COMPONENTS
  interactive_markers
  geometry_msgs
  moveit_core
  moveit_visual_tools
  moveit_ros_planning
  moveit_ros_planning_interface
  pluginlib
  geometric_shapes
  pcl_conversions
  pcl_ros
  roscpp
  rospy
  sensor_msgs
  std_msgs
  tf
  rosbag
  tf2_ros
  tf2_eigen
  tf2_geometry_msgs
  robotiq_2f_gripper_control
)

catkin_package(
)

include_directories(include
  ${catkin_INCLUDE_DIRS}
)

add_executable(manipulation_node
	src/nodes/manipulation_node.cpp 
	src/functions/manipulation/move_to_.cpp
        src/functions/manipulation/generate_workspace.cpp
	src/functions/perception/perception_class.cpp
        src/functions/perception/concatenate_cloud.cpp
	src/functions/perception/take_snapshot_.cpp)
target_link_libraries(manipulation_node ${catkin_LIBRARIES})


