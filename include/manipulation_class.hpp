// ********************************************************************************************
// Author: Brian Flynn;
// Test Engineer - NERVE Center @ UMASS Lowell
// grasp_cluster_class.hpp
// ********************************************************************************************

#ifndef MANIPULATION_CLASS_HPP
#define MANIPULATION_CLASS_HPP

#include <boost/filesystem.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <array>
#include <octomap/octomap.h>
#include <octomap/OcTree.h>
#include <moveit/move_group/capability_names.h>
#include <std_srvs/Empty.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <robotiq_2f_gripper_control/Robotiq2FGripper_robot_output.h>

typedef boost::shared_ptr<moveit::planning_interface::MoveGroupInterface> MoveGroupPtr;
typedef boost::shared_ptr<moveit::planning_interface::PlanningSceneInterface> PlanningScenePtr;

class Manipulation
{
  public:

    //Constructor
    Manipulation()
    {
      PLANNING_GROUP = "manipulator";
    }

    //Members
    MoveGroupPtr move_group_ptr;
    PlanningScenePtr planning_scene_ptr;
    std::string PLANNING_GROUP;
    std::vector<double> joint_group_positions;
    moveit::core::RobotStatePtr current_state;

    //Functions
    Manipulation(ros::NodeHandle nodeHandle)
    {
      PLANNING_GROUP = "manipulator";
    }
    void move_to_left();
    void move_to_right();
    void move_to_top();
    void move_to_front();
    void move_to_wait_position();
    //void pickup_object();
    //void set_attached_object();
    //void dropoff_object();
    void generate_workspace();

};  

#endif // MANIPULATION_CLASS
