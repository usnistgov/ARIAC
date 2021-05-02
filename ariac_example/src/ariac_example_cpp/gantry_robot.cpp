#include "gantry_robot.h"

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>

/**
 * @brief Construct a new Gantry Control:: Gantry Control object
 * 
 * @param node 
 */
GantryRobot::GantryRobot(ros::NodeHandle &node) : node_("/ariac/gantry"),
                                                      planning_group_("/ariac/gantry/robot_description"),
                                                      gantry_full_options_("gantry_full", planning_group_, node_),
                                                      gantry_arm_options_("gantry_arm", planning_group_, node_),
                                                      gantry_torso_options_("gantry_torso", planning_group_, node_),
                                                      gantry_arm_ee_link_options_("gantry_ee", planning_group_, node_),
                                                      gantry_full_group_(gantry_full_options_),
                                                      gantry_arm_group_(gantry_arm_options_),
                                                      gantry_torso_group_(gantry_torso_options_),
                                                      gantry_arm_endeffector_group_(gantry_arm_ee_link_options_)
{
    ROS_INFO_STREAM("[GantryRobot::GantryRobot] constructor called... ");
}

void GantryRobot::init()
{
    ROS_INFO_STREAM("[GantryRobot::init] called... ");
    double time_called = ros::Time::now().toSec();

    ROS_INFO_NAMED("init", "Planning frame: %s", gantry_full_group_.getPlanningFrame().c_str());
    ROS_INFO_NAMED("init", "Planning frame: %s", gantry_arm_group_.getPlanningFrame().c_str());
    ROS_INFO_NAMED("init", "Planning frame: %s", gantry_torso_group_.getPlanningFrame().c_str());
    ROS_INFO_NAMED("init", "Planning frame: %s", gantry_arm_endeffector_group_.getPlanningFrame().c_str());

    ROS_INFO_NAMED("init", "End effector link: %s", gantry_arm_group_.getEndEffectorLink().c_str());

    gantry_arm_group_.setPoseReferenceFrame("world");

    // preset locations

    // joint positions to go to start location
    home_.gantry = {0, 0, 0};
    home_.left_arm = {0.0, -PI / 4, PI / 2, -PI / 4, PI / 2, 0};

    // joint positions to go to bin3
    bin3_.gantry = {4.0, -1.1, 0.};
    bin3_.left_arm = {0.0, -PI / 4, PI / 2, -PI / 4, PI / 2, 0};

    // joint positions to go to agv2
    agv2_.gantry = {0.6, 6.9, PI};
    agv2_.left_arm = {0.0, -PI / 4, PI / 2, -PI / 4, PI / 2, 0};

    //--Raw pointers are frequently used to refer to the planning group for improved performance.
    //--To start, we will create a pointer that references the current robot’s state.
    const moveit::core::JointModelGroup *joint_model_group =
        full_robot_group_.getCurrentState()->getJointModelGroup("Full_Robot");

    //--Let’s set a joint space goal and move towards it.
    moveit::core::RobotStatePtr current_state = full_robot_group_.getCurrentState();

    //--Next get the current set of joint values for the group.
    current_state->copyJointGroupPositions(joint_model_group, joint_group_positions_);

    gantry_joint_trajectory_publisher_ =
        node_.advertise<trajectory_msgs::JointTrajectory>("/ariac/gantry/gantry_controller/command", 10);

    left_arm_joint_trajectory_publisher_ =
        node_.advertise<trajectory_msgs::JointTrajectory>("/ariac/gantry/left_arm_controller/command", 10);

    right_arm_joint_trajectory_publisher_ =
        node_.advertise<trajectory_msgs::JointTrajectory>("/ariac/gantry/right_arm_controller/command", 10);

    joint_states_subscriber_ = node_.subscribe(
        "/ariac/gantry/joint_states", 10, &GantryControl::joint_states_callback, this);

    left_gripper_state_subscriber_ = node_.subscribe(
        "/ariac/gantry/left_arm/gripper/state", 10, &GantryControl::left_gripper_state_callback, this);

    right_gripper_state_subscriber_ = node_.subscribe(
        "/ariac/gantry/right_arm/gripper/state", 10, &GantryControl::right_gripper_state_callback, this);

    gantry_controller_state_subscriber_ = node_.subscribe(
        "/ariac/gantry/gantry_controller/state", 10, &GantryControl::gantry_controller_state_callback, this);

    left_arm_controller_state_subscriber_ = node_.subscribe(
        "/ariac/gantry/left_arm_controller/state", 10, &GantryControl::left_arm_controller_state_callback, this);

    right_arm_controller_state_subscriber_ = node_.subscribe(
        "/ariac/gantry/right_arm_controller/state", 10, &GantryControl::right_arm_controller_state_callback, this);

    while ((current_gantry_controller_state_.joint_names.size() == 0) || 
    (current_left_arm_controller_state_.joint_names.size() == 0) || 
    (current_right_arm_controller_state_.joint_names.size() == 0))
    {
        ROS_WARN("[GantryControl::init] Waiting for first controller_state callbacks...");
        ros::Duration(0.1).sleep();
    }

    left_gripper_control_client =
        node_.serviceClient<nist_gear::VacuumGripperControl>("/ariac/gantry/left_arm/gripper/control");
    left_gripper_control_client.waitForExistence();

    right_gripper_control_client =
        node_.serviceClient<nist_gear::VacuumGripperControl>("/ariac/gantry/right_arm/gripper/control");
    right_gripper_control_client.waitForExistence();

    // Move robot to init position
    ROS_INFO("[GantryControl::init] Init position ready)...");
}