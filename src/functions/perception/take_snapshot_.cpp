// ********************************************************************************************
// Author: Brian Flynn;
// Test Engineer - NERVE Center @ UMASS Lowell
// take_snapshot_.cpp
//
// perception_class function
// ********************************************************************************************

#include "perception_class.hpp"

// take_snapshot_ function list
// --------------------------------
// take snapshots (save the current_cloud into different member variables) of the pointcloud in 
// different robot positions in order to get a more complete pointcloud after concatenation

// TAKE SNAPSHOT LEFT FUNCTION
//
// Move robot into position (calling manipulation_class move_to_left function) to take "left" pointcloud snapshot
// save current_cloud into left_cloud for concatenation later
void Perception::take_snapshot_left()
{
  this->left_cloud = this->current_cloud;
}

// TAKE SNAPSHOT RIGHT FUNCTION
//
// Move robot into position (calling manipulation_class move_to_left function) to take "left" pointcloud snapshot
// save current_cloud into left_cloud for concatenation later
void Perception::take_snapshot_right()
{
  this->right_cloud = this->current_cloud;
}

// TAKE SNAPSHOT TOP FUNCTION
//
// Move robot into position (calling manipulation_class move_to_left function) to take "left" pointcloud snapshot
// save current_cloud into left_cloud for concatenation later
void Perception::take_snapshot_top()
{
  this->top_cloud = this->current_cloud;
}

// TAKE SNAPSHOT FRONT FUNCTION
//
// Move robot into position (calling manipulation_class move_to_left function) to take "left" pointcloud snapshot
// save current_cloud into left_cloud for concatenation later
void Perception::take_snapshot_front()
{
  this->front_cloud = this->current_cloud;
}


