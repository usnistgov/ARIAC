#ifndef GANTRY_ROBOT_H
#define GANTRY_ROBOT_H

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <string>
#include <vector>
#include <ros/console.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <array>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>


#include <boost/foreach.hpp>
#define foreach BOOST_FOREACH
#include <Eigen/Dense>
#include <tf2/LinearMath/Quaternion.h>
#include <nist_gear/VacuumGripperState.h>
#include <nist_gear/VacuumGripperControl.h>
#include <sensor_msgs/JointState.h>
#include <control_msgs/JointTrajectoryControllerState.h>
#include <trajectory_msgs/JointTrajectory.h>

class GantryRobot
{
public:
    explicit GantryRobot(ros::NodeHandle &node);
    void init();

private:
    std::vector<double> joint_group_positions_;
    ros::NodeHandle node_;
    std::string planning_group_;
    moveit::planning_interface::MoveGroupInterface::Options gantry_torso_options_;
    moveit::planning_interface::MoveGroupInterface::Options gantry_arm_options_;
    moveit::planning_interface::MoveGroupInterface::Options gantry_full_options_;
    moveit::planning_interface::MoveGroupInterface::Options gantry_arm_ee_link_options_;
    moveit::planning_interface::MoveGroupInterface gantry_torso_group_;
    moveit::planning_interface::MoveGroupInterface gantry_arm_group_;
    moveit::planning_interface::MoveGroupInterface gantry_full_group_;
    moveit::planning_interface::MoveGroupInterface gantry_arm_ee_link_group_;
};

#endif  // GANTRY_ROBOT_H
