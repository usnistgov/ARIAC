#include "robotiq_gripper_controller/robotiq_gripper_controller.hpp"

namespace robotiq_gripper_controller
{

CallbackReturn RobotiqGripperController::on_init()
{
  try
  {
    param_listener_ = std::make_shared<ParamListener>(get_node());
  }
  catch (const std::exception & e)
  {
    fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
    return controller_interface::CallbackReturn::ERROR;
  }
  
  return CallbackReturn::SUCCESS;
}

CallbackReturn RobotiqGripperController::on_configure(const rclcpp_lifecycle::State&)
{
  params_ = param_listener_->get_params();

  // Controlled joint
  if (params_.joint.empty())
  {
    RCLCPP_ERROR(get_node()->get_logger(), "Joint name cannot be empty");
    return controller_interface::CallbackReturn::ERROR;
  }

  return CallbackReturn::SUCCESS;
}

CallbackReturn RobotiqGripperController::on_activate(const rclcpp_lifecycle::State&)
{
  // Create action server
  action_server_ = rclcpp_action::create_server<GripperCommandAction>(
    get_node(), 
    "~/gripper_command",
    std::bind(&RobotiqGripperController::goal_callback, this, std::placeholders::_1, std::placeholders::_2),
    std::bind(&RobotiqGripperController::cancel_callback, this, std::placeholders::_1),
    std::bind(&RobotiqGripperController::accepted_callback, this, std::placeholders::_1)
  );

  auto current_joint_angle = state_interfaces_[0].get_optional();

  if (current_joint_angle.has_value()) { 
    desired_joint_position_ = current_joint_angle.value();
    current_width_ = joint_to_width(current_joint_angle.value());
    return CallbackReturn::SUCCESS;
  } else {
    return CallbackReturn::FAILURE;
  }
  
}

CallbackReturn RobotiqGripperController::on_deactivate(const rclcpp_lifecycle::State&)
{
  release_interfaces();

  return CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration RobotiqGripperController::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;
      
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  
  config.names.push_back(params_.joint + "/position");

  return config;
}

controller_interface::InterfaceConfiguration RobotiqGripperController::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;
      
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  config.names.push_back(params_.joint + "/position");
  config.names.push_back(params_.joint + "/velocity");
  config.names.push_back(params_.joint + "/effort");

  return config;
}

controller_interface::return_type RobotiqGripperController::update(const rclcpp::Time& time, const rclcpp::Duration&) 
{
  // Get current joint position
  auto joint_angle_optional = state_interfaces_[0].get_optional();

  if (joint_angle_optional.has_value()) {   
    current_joint_angle_ = joint_angle_optional.value(); 
    current_width_ = joint_to_width(joint_angle_optional.value());
  } else {
    RCLCPP_ERROR(get_node()->get_logger(), "Unable to read position state interface");
    return controller_interface::return_type::ERROR;
  }

  // Get current joint velocity
  auto current_joint_velocity = state_interfaces_[1].get_optional();

  if (current_joint_velocity.has_value()) {    
    // Capture a rolling vector of the last ten velocity values
    joint_velocities_.push_back(current_joint_velocity.value());
    if ((int)joint_velocities_.size() > velocity_sample_size_) {
      joint_velocities_.erase(joint_velocities_.begin());
    }
  } else {
    RCLCPP_ERROR(get_node()->get_logger(), "Unable to read velocity state interface");
    return controller_interface::return_type::ERROR;
  }
  
  if (goal_accepted_) { // Handle new action goal

    initial_joint_angle_ = current_joint_angle_;
    
    desired_joint_position_ = width_to_joint(current_goal_handle_->get_goal()->width);

    executing_goal_ = true;
    goal_accepted_ = false;

    execution_start_time_ = time;
    last_feedback_time_ = time;
    joint_velocities_.clear();
  
  } else if (executing_goal_) { // Handle currently executing goal

    // Handle feedback
    if ((time - last_feedback_time_).seconds() > 1/feedback_rate_) {
      auto feedback = std::make_shared<GripperCommandAction::Feedback>();
    
      feedback->width = current_width_;
      current_goal_handle_->publish_feedback(feedback);

      last_feedback_time_ = time;
    }

    auto result = std::make_shared<GripperCommandAction::Result>();

    if (cancel_requested_) {  // Handle cancel
      result->reached_goal_width = false;
      result->stalled = false;
      result->width = current_width_;
      
      current_goal_handle_->canceled(result);
      
      executing_goal_ = false;
      cancel_requested_ = false;
    }

    if (abs(width_to_joint(current_goal_handle_->get_goal()->width) - width_to_joint(current_width_)) < joint_tolerance_) { // Check if current width is within goal tolerance
      
      result->reached_goal_width = true;
      result->stalled = false;
      result->width = current_width_;

      current_goal_handle_->succeed(result);
      executing_goal_ = false;

    } else if ((int)joint_velocities_.size() == velocity_sample_size_) { // Check if stalled

      // if (abs(current_joint_angle_- initial_joint_angle_) < joint_tolerance_) {
      //   result->reached_goal_width = false;
      //   result->stalled = false;
      //   result->width = current_width_;
        
      //   RCLCPP_ERROR(get_node()->get_logger(), "Never moved");
      //   current_goal_handle_->abort(result);
      //   executing_goal_= false;
      // } else {
    
      double average_velocity = std::accumulate(joint_velocities_.begin(), joint_velocities_.end(), 0.0) / joint_velocities_.size();

      if (abs(average_velocity) < stalled_velocity_tolerance_) {
        result->reached_goal_width = false;
        result->stalled = true;
        result->width = current_width_;

        if (closing_) {
          current_goal_handle_->succeed(result);
        } else {
          current_goal_handle_->abort(result);
          RCLCPP_ERROR(get_node()->get_logger(), "Trying to open but stalled");
        }
        
        executing_goal_ = false;
      // }
      }

    } else if ((time - execution_start_time_).seconds() > timeout_sec_) { // Check if timeout was reached
      
      auto result = std::make_shared<GripperCommandAction::Result>();
      result->reached_goal_width = false;
      result->stalled = false;
      result->width = current_width_;

      RCLCPP_ERROR(get_node()->get_logger(), "Timeout");

      current_goal_handle_->abort(result);
      executing_goal_ = false;

    } 
  }

  // Set the position to the desired position
  if (!command_interfaces_[0].set_value(desired_joint_position_)) {
    return controller_interface::return_type::ERROR;
  }

  return controller_interface::return_type::OK;
}

rclcpp_action::GoalResponse RobotiqGripperController::goal_callback(
  const rclcpp_action::GoalUUID &, std::shared_ptr<const GripperCommandAction::Goal> goal)
{

  if (goal->width < 0 || goal->width > 0.085) {
    RCLCPP_ERROR(get_node()->get_logger(), "Desired width must be between 0 and 85mm");
  }

  if (executing_goal_) {
    return rclcpp_action::GoalResponse::REJECT;
  }

  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse RobotiqGripperController::cancel_callback(const std::shared_ptr<GoalHandle>)
{
  if (!executing_goal_) {
    return rclcpp_action::CancelResponse::REJECT;
  }

  cancel_requested_ = true;

  return rclcpp_action::CancelResponse::ACCEPT;
}

void RobotiqGripperController::accepted_callback(std::shared_ptr<GoalHandle> goal_handle)
{
  current_goal_handle_ = goal_handle;

  // Define if the action goal is closing or opening the gripper
  closing_ =  current_goal_handle_->get_goal()->width < current_width_;

  goal_accepted_ = true;
}

double RobotiqGripperController::width_to_joint(double width)
{
  double joint_angle = max_joint_angle_ * (1.0 - (width / max_width_));

  if (joint_angle > 0.79) {
    return 0.79;
  } else if (joint_angle < 0.01) {
    return 0.01;
  }

  return joint_angle;
}

double RobotiqGripperController::joint_to_width(double joint_angle)
{
  return max_width_ * (1.0 - (joint_angle / max_joint_angle_));
}


}  // namespace robotiq_gripper_controller

#include "pluginlib/class_list_macros.hpp"
// NOLINTNEXTLINE
PLUGINLIB_EXPORT_CLASS(
  robotiq_gripper_controller::RobotiqGripperController,
  controller_interface::ControllerInterface)