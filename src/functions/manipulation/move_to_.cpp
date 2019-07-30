// ********************************************************************************************
// Author: Brian Flynn;
// Test Engineer - NERVE Center @ UMASS Lowell
// move_to_.cpp
//
// manipulation_class function
// ********************************************************************************************

#include "manipulation_class.hpp"

// move_to_ function list
// --------------------------------
// Move robot to known position via specific joint angles
// "left," "right," etc. are with reference to the person looking at the robot for simplicity
// technically the robot will be moving to it's left when I tell it to go right but this can be
// modified later for clarity if you so choose

// MOVE TO LEFT FUNCTION
//
// Move robot into position to take "left" pointcloud snapshot
void Manipulation::move_to_left()
{
  const robot_state::JointModelGroup* joint_model_group =
    this->move_group_ptr->getCurrentState()->getJointModelGroup(this->PLANNING_GROUP);
  this->current_state = this->move_group_ptr->getCurrentState();
  this->current_state->copyJointGroupPositions(joint_model_group, this->joint_group_positions);
  this->joint_group_positions[0] = -0.349;
  this->joint_group_positions[1] = -2.181;
  this->joint_group_positions[2] = -0.785;
  this->joint_group_positions[3] = -2.007;
  this->joint_group_positions[4] = -4.101;
  this->joint_group_positions[5] = 1.134;
  
  this->move_group_ptr->setJointValueTarget(this->joint_group_positions);
  this->move_group_ptr->move();
}

// MOVE TO RIGHT FUNCTION
//
// Move robot into position to take "right" pointcloud snapshot
void Manipulation::move_to_right()
{
  const robot_state::JointModelGroup* joint_model_group =
    this->move_group_ptr->getCurrentState()->getJointModelGroup(this->PLANNING_GROUP);
  this->current_state = this->move_group_ptr->getCurrentState();
  this->current_state->copyJointGroupPositions(joint_model_group, this->joint_group_positions);
  this->joint_group_positions[0] = 0.698;
  this->joint_group_positions[1] = -2.356;
  this->joint_group_positions[2] = -0.436;
  this->joint_group_positions[3] = -2.408;
  this->joint_group_positions[4] = -5.236;
  this->joint_group_positions[5] = -0.785;

  this->move_group_ptr->setJointValueTarget(this->joint_group_positions);
  this->move_group_ptr->move();
}

// MOVE TO TOP FUNCTION
//
// Move robot into position to take "top" pointcloud snapshot
// TODO*
void Manipulation::move_to_top()
{
  const robot_state::JointModelGroup* joint_model_group =
    this->move_group_ptr->getCurrentState()->getJointModelGroup(this->PLANNING_GROUP);
  this->current_state = this->move_group_ptr->getCurrentState();
  this->current_state->copyJointGroupPositions(joint_model_group, this->joint_group_positions);
  this->joint_group_positions[0] = 0.176;
  this->joint_group_positions[1] = -1.713;
  this->joint_group_positions[2] = -1.418;
  this->joint_group_positions[3] = -1.581;
  this->joint_group_positions[4] = 1.571;
  this->joint_group_positions[5] = 0.175;

  this->move_group_ptr->setJointValueTarget(this->joint_group_positions);
  this->move_group_ptr->move();
}

// MOVE TO FRONT FUNCTION
//
// Move robot into position to take "front" pointcloud snapshot
// TODO*
void Manipulation::move_to_front()
{
  const robot_state::JointModelGroup* joint_model_group =
    this->move_group_ptr->getCurrentState()->getJointModelGroup(this->PLANNING_GROUP);
  this->current_state = this->move_group_ptr->getCurrentState();
  this->current_state->copyJointGroupPositions(joint_model_group, this->joint_group_positions);
  this->joint_group_positions[0] = 0.784;
  this->joint_group_positions[1] = -1.238;
  this->joint_group_positions[2] = -2.246;
  this->joint_group_positions[3] = -3.387;
  this->joint_group_positions[4] = -0.899;
  this->joint_group_positions[5] = 0.398;

  this->move_group_ptr->setJointValueTarget(this->joint_group_positions);
  this->move_group_ptr->move();
}

// MOVE TO WAIT POSITION FUNCTION
//
// Move robot into neutral waiting position before taking snapshots
// or performing manipulation task
// TODO*
void Manipulation::move_to_wait_position()
{
  const robot_state::JointModelGroup* joint_model_group =
    this->move_group_ptr->getCurrentState()->getJointModelGroup(this->PLANNING_GROUP);
  this->current_state = this->move_group_ptr->getCurrentState();
  this->current_state->copyJointGroupPositions(joint_model_group, this->joint_group_positions);
  this->joint_group_positions[0] = 0;
  this->joint_group_positions[1] = -1.57;
  this->joint_group_positions[2] = -1.57;
  this->joint_group_positions[3] = -1.57;
  this->joint_group_positions[4] = 1.57;
  this->joint_group_positions[5] = 0;

  this->move_group_ptr->setJointValueTarget(this->joint_group_positions);
  this->move_group_ptr->move();
}


