/* Author: Brian Flynn*/

// ROS
#include <ros/ros.h>

// MoveIt!
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/move_group_interface/move_group_interface.h>

// TF2
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

void addCollisionObjects(moveit::planning_interface::PlanningSceneInterface& planning_scene_interface) {

  // Create a vector to hold collision objects.
  std::vector<moveit_msgs::CollisionObject> collision_objects;
  collision_objects.resize(2);

  // Add workstation surface
  // ************************************************************************************************ 
  // 0.192 (robot center from edge of table)
  collision_objects[0].id = "workstation";
  collision_objects[0].header.frame_id = "world";

  // Define primitives, dimensions and position
  collision_objects[0].primitives.resize(1);
  collision_objects[0].primitives[0].type = collision_objects[0].primitives[0].BOX;
  collision_objects[0].primitives[0].dimensions.resize(3);
  collision_objects[0].primitives[0].dimensions[0] = 0.9;
  collision_objects[0].primitives[0].dimensions[1] = 1.2;
  collision_objects[0].primitives[0].dimensions[2] = 0.25;
  collision_objects[0].primitive_poses.resize(1);
  collision_objects[0].primitive_poses[0].position.x = -0.642;
  collision_objects[0].primitive_poses[0].position.y = 0;
  collision_objects[0].primitive_poses[0].position.z = -0.1345;

  // Add and apply collision object(s) to scene
  collision_objects[0].operation = collision_objects[0].ADD;
  planning_scene_interface.applyCollisionObjects(collision_objects);
  // ************************************************************************************************


  // Add pedestal area
  // ************************************************************************************************
  // mount plate is slightly higher than table surface
  collision_objects[1].id = "pedestal";
  collision_objects[1].header.frame_id = "world";

  // Define primitives, dimensions and position
  collision_objects[1].primitives.resize(1);
  collision_objects[1].primitives[0].type = collision_objects[1].primitives[0].BOX;
  collision_objects[1].primitives[0].dimensions.resize(3);
  collision_objects[1].primitives[0].dimensions[0] = 0.384;
  collision_objects[1].primitives[0].dimensions[1] = 0.384;
  collision_objects[1].primitives[0].dimensions[2] = 0.25;
  collision_objects[1].primitive_poses.resize(1);
  collision_objects[1].primitive_poses[0].position.x = 0.0;
  collision_objects[1].primitive_poses[0].position.y = 0.0;
  collision_objects[1].primitive_poses[0].position.z = -0.125;

  // Add and apply collision object(s) to scene
  collision_objects[1].operation = collision_objects[1].ADD;
  planning_scene_interface.applyCollisionObjects(collision_objects);
  // ************************************************************************************************

}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "insert_workstation_objects");
  ros::NodeHandle nh;
  ros::AsyncSpinner spinner(1);
  spinner.start();

  // Set up planning scene/group (robot_name)
  ros::WallDuration(1.0).sleep();
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  moveit::planning_interface::MoveGroupInterface group("manipulator");
  group.setPlanningTime(45.0);

  // Add object(s) to planning scene
  addCollisionObjects(planning_scene_interface);

  // Wait for objects to initialize
  ros::WallDuration(1.0).sleep();

  ros::waitForShutdown();
  return 0;
}
