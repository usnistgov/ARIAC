#ifndef ROBOTIQ_GRIPPER_CONTROLLER__ROBOTIQ_GRIPPER_CONTROLLER_HPP_
#define ROBOTIQ_GRIPPER_CONTROLLER__ROBOTIQ_GRIPPER_CONTROLLER_HPP_

#include <controller_interface/controller_interface.hpp>
#include <robotiq_gripper_controller/robotiq_gripper_controller_parameters.hpp>

#include "rclcpp_action/create_server.hpp"

#include <ariac_interfaces/action/gripper_command.hpp>

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

using GripperCommandAction = ariac_interfaces::action::GripperCommand;
using ActionServer = rclcpp_action::Server<GripperCommandAction>;
using ActionServerPtr = ActionServer::SharedPtr;
using GoalHandle = rclcpp_action::ServerGoalHandle<GripperCommandAction>;
using GoalHandlePtr = std::shared_ptr<GoalHandle>;

namespace robotiq_gripper_controller {

class RobotiqGripperController : public controller_interface::ControllerInterface {
  public:
    CallbackReturn on_init() override;
    CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;
    CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;
    CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;
    controller_interface::InterfaceConfiguration command_interface_configuration() const override;
    controller_interface::InterfaceConfiguration state_interface_configuration() const override;
    controller_interface::return_type update(const rclcpp::Time& time, const rclcpp::Duration& period) override;

  private:
    std::shared_ptr<ParamListener> param_listener_;
    Params params_;

    ActionServerPtr action_server_;

    rclcpp_action::GoalResponse goal_callback(
      const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const GripperCommandAction::Goal> goal);
  
    rclcpp_action::CancelResponse cancel_callback(const std::shared_ptr<GoalHandle> goal_handle);
  
    void accepted_callback(std::shared_ptr<GoalHandle> goal_handle);

    double width_to_joint(double width);
    double joint_to_width(double joint_angle);

    bool goal_accepted_ = false;
    bool executing_goal_ = false;
    bool cancel_requested_ = false;

    GoalHandlePtr current_goal_handle_;
    rclcpp::Time execution_start_time_;
    rclcpp::Time last_feedback_time_;
    double timeout_sec_ = 10;
    double desired_joint_position_;
    double initial_joint_angle_;
    double current_width_;
    double current_joint_angle_;
    bool closing_;

    std::vector<double> joint_velocities_;

    double feedback_rate_ = 10;
    double velocity_sample_size_ = 100;

    double joint_tolerance_ = 0.001;
    double stalled_velocity_tolerance_ = 1E-3;

    double max_width_ = 0.085;
    double max_joint_angle_ = 0.8;
};

}  // namespace robotiq_gripper_controller

#endif  // ROBOTIQ_GRIPPER_CONTROLLER__ROBOTIQ_GRIPPER_CONTROLLER_HPP_
