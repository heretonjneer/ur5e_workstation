// ********************************************************************************************
// Author: Brian Flynn;
// Test Engineer - NERVE Center @ UMASS Lowell
// generate_workspace.cpp
//
// manipulation_class function
// ********************************************************************************************

#include "manipulation_class.hpp"

void Manipulation::generate_workspace() {

  // Create a vector to hold collision objects.
  std::vector<moveit_msgs::CollisionObject> collision_objects;
  collision_objects.resize(3);

  // Add workstation surface
  // ************************************************************************************************ 
  // 0.192 (distance from robot center to edge of workstation)
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
  // ************************************************************************************************

  // Add dropoff box
  // ************************************************************************************************
  // min line with table surface
  collision_objects[2].id = "dropoffbox";
  collision_objects[2].header.frame_id = "world";

  // Define primitives, dimensions and position
  collision_objects[2].primitives.resize(1);
  collision_objects[2].primitives[0].type = collision_objects[2].primitives[0].BOX;
  collision_objects[2].primitives[0].dimensions.resize(3);
  collision_objects[2].primitives[0].dimensions[0] = 0.33;
  collision_objects[2].primitives[0].dimensions[1] = 0.13;
  collision_objects[2].primitives[0].dimensions[2] = 0.14;
  collision_objects[2].primitive_poses.resize(1);
  collision_objects[2].primitive_poses[0].position.x = -0.255;
  collision_objects[2].primitive_poses[0].position.y = -0.765;
  collision_objects[2].primitive_poses[0].position.z = -0.0795;
  // ************************************************************************************************

  // Add and apply collision object(s) to scene
  collision_objects[0].operation = collision_objects[0].ADD;
  collision_objects[1].operation = collision_objects[1].ADD;
  collision_objects[2].operation = collision_objects[2].ADD;
  this->planning_scene_ptr->applyCollisionObjects(collision_objects);

}
