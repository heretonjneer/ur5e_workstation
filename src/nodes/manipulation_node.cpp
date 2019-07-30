// ********************************************************************************************
// Author: Brian Flynn;
// Test Engineer - NERVE Center @ UMASS Lowell
// manipulation_node.cpp
//
// manipulation_class function
// ********************************************************************************************

#include "manipulation_class.hpp"
#include "perception_class.hpp"

int main(int argc, char** argv)
{
  // ros initialization
  ros::init(argc,argv,"manipulation_node");
  ros::NodeHandle nh;
  ros::AsyncSpinner spinner(2);
  spinner.start();

  // Create manipulation and perception objects
  Manipulation manipulation(nh);
  Perception perception(nh);

  // Wait for spinner to start
  ros::WallDuration(1.0).sleep();

  // Transform listener
  perception.transform_listener_ptr = TransformListenerPtr(
      new tf::TransformListener());
  perception.init_subscriber(nh);

  // Planning scene interface
  manipulation.planning_scene_ptr = PlanningScenePtr(
      new moveit::planning_interface::PlanningSceneInterface());

  // Moveit interface
  manipulation.move_group_ptr = MoveGroupPtr(
      new moveit::planning_interface::MoveGroupInterface(manipulation.PLANNING_GROUP));
  
  // Set useful variables before robot manipulation begins
  manipulation.move_group_ptr->setPlanningTime(45.0);
  manipulation.move_group_ptr->setMaxVelocityScalingFactor(0.1);
  manipulation.move_group_ptr->setPlannerId("RRTConnectkConfigDefault");
  
  // Add object(s) to planning scene
  manipulation.generate_workspace();

  // Wait for objects to initialize
  ros::WallDuration(1.0).sleep();

  // Move robot into starting position and wait a moment
  manipulation.move_to_wait_position();
  ros::Duration(1).sleep();
  
  // Move into each position and halt for a moment to capture pointcloud snapshot
  //***********************************************
  manipulation.move_to_top();
  ros::Duration(1).sleep();
  perception.take_snapshot_top();
  ros::Duration(1).sleep();

  manipulation.move_to_left();
  ros::Duration(1).sleep();
  perception.take_snapshot_left();
  ros::Duration(1).sleep();

  manipulation.move_to_right();
  ros::Duration(1).sleep();
  perception.take_snapshot_right();
  ros::Duration(1).sleep();

  manipulation.move_to_front();
  ros::Duration(1).sleep();
  perception.take_snapshot_front();
  ros::Duration(1).sleep();
  //***********************************************

  // Return to waiting position at completion of cloud capture
  manipulation.move_to_wait_position();

  // Concatenate the pointclouds and run filters on them
  perception.concatenate_clouds();

  // Publish concatenated cloud
  perception.publish_combined_cloud();

  ros::waitForShutdown();
  return 0;

}
