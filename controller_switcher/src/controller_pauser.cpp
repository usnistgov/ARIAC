#include <ros/ros.h>
#include <string.h>
#include <controller_manager_msgs/ListControllers.h>
#include <controller_manager_msgs/SwitchController.h>
#include <nist_gear/RobotHealth.h>

/**
 * @brief Class to switch the controllers of the kitting and gantry robots
 *
 */
class RobotControllerSwitcher
{
public:
  /**
   * @brief Construct a new Robot Controller Switcher object
   *
   * @param nh Node handle
   */
  RobotControllerSwitcher(ros::NodeHandle* nh)
  {
    ROS_INFO_STREAM("Controller switcher loaded");
    this->kitting_robot_health = "active";
    this->gantry_robot_health = "active";
    this->robot_health_sub =
        nh->subscribe("/ariac/robot_health", 1, &RobotControllerSwitcher::OnRobotHealthContent, this);

    this->stopped_kitting_controllers = false;
    this->stopped_gantry_controllers = false;
  }

  ~RobotControllerSwitcher()
  {
  }

  /**
   * @brief
   *
   * @param msg
   */
  void OnRobotHealthContent(const nist_gear::RobotHealth& msg)
  {
    // ROS_INFO_STREAM("Inside callback");
    //   std::lock_guard<std::mutex> lock(this->data_ptr->mutex);
    this->kitting_robot_health = msg.kitting_robot_health;
    if (this->kitting_robot_health == "inactive") {
      this->kitting_lock_countdown_begin = ros::Time::now();
    }

    this->gantry_robot_health = msg.assembly_robot_health;
    if (this->gantry_robot_health == "inactive") {
      this->gantry_lock_countdown_begin = ros::Time::now();
    }
    // ROS_INFO_STREAM("kitting robot health: " << this->kitting_robot_health);
    // ROS_INFO_STREAM("gantry robot health: " << this->gantry_robot_health);
  }

  /**
   * @brief Get the Static Controllers object
   *
   * @param nh
   * @param robot_name
   * @return std::vector<std::string>
   */
  std::vector<std::string> GetStaticControllers(ros::NodeHandle& nh, std::string robot_name)
  {
    std::vector<std::string> static_controllers;

    std::string srv_name = "/ariac/" + robot_name + "/controller_manager/list_controllers";
    ros::ServiceClient client = nh.serviceClient<controller_manager_msgs::ListControllers>(srv_name);
    controller_manager_msgs::ListControllers srv;

    if (client.call(srv))
    {
      for (auto state : srv.response.controller)
      {
        if (state.name.rfind("static", 0) == 0)
        {
          static_controllers.push_back(state.name);
        }
      }
    }
    else
    {
      ROS_ERROR_STREAM("Unable to call " << srv_name);
    }

    return static_controllers;
  }

  /**
   * @brief
   *
   * @param nh
   */
  void StopKittingRobot(ros::NodeHandle& nh)
  {
    // if the controllers for kitting robot have already been stopped, ignore

    if (this->kitting_robot_health == "active")
      return;

    if (this->stopped_kitting_controllers)
      return;

    auto current_time = ros::Time::now();
    if (current_time.toSec() - this->kitting_lock_countdown_begin.toSec() < 10.0) {
      // ROS_INFO_STREAM("Diff: " << current_time.toSec() - this->kitting_lock_countdown_begin.toSec());
      return;
    }
      

    // ROS_INFO_STREAM("Stopping kitting robot");

    controller_manager_msgs::SwitchController srv;
    srv.request.start_controllers = GetStaticControllers(nh, "kitting");
    srv.request.stop_controllers.push_back("kitting_arm_controller");

    std::string srv_name = "/ariac/kitting/controller_manager/switch_controller";
    ros::ServiceClient client = nh.serviceClient<controller_manager_msgs::SwitchController>(srv_name);

    srv.request.strictness = controller_manager_msgs::SwitchControllerRequest::BEST_EFFORT;

    if (client.call(srv))
    {
      if (srv.response.ok)
      {
        ROS_INFO_STREAM("Stopped kitting controllers");
        this->stopped_kitting_controllers = true;
      }
      else
      {
        ROS_ERROR_STREAM("Unable to stop controller");
      }
    }
  }

  /**
   * @brief
   *
   * @param nh
   */
  void StopGantryRobot(ros::NodeHandle& nh)
  {
    // if the controllers for the gantry have already been stopped, ignore
    if (this->gantry_robot_health == "active")
      return;

    if (this->stopped_gantry_controllers)
      return;

    auto current_time = ros::Time::now();
    if (current_time.toSec() - this->gantry_lock_countdown_begin.toSec() < 10.0) {
      // ROS_INFO_STREAM("Diff gantry: " << current_time.toSec() - this->gantry_lock_countdown_begin.toSec());
      return;
    }

    

    controller_manager_msgs::SwitchController srv;
    srv.request.start_controllers = GetStaticControllers(nh, "gantry");
    srv.request.stop_controllers.push_back("gantry_arm_controller");

    std::string srv_name = "/ariac/gantry/controller_manager/switch_controller";
    ros::ServiceClient client = nh.serviceClient<controller_manager_msgs::SwitchController>(srv_name);

    srv.request.strictness = controller_manager_msgs::SwitchControllerRequest::BEST_EFFORT;

    if (client.call(srv))
    {
      if (srv.response.ok)
      {
        ROS_INFO_STREAM("Stopped gantry controllers");
        this->stopped_gantry_controllers = true;
      }
      else
      {
        ROS_ERROR_STREAM("Unable to stop controller");
      }
    }
  }

  /**
   * @brief
   *
   * @param nh
   * @param static_controllers
   * @param robot_name
   */
  void StartKittingRobot(ros::NodeHandle& nh)
  {
    if (this->kitting_robot_health == "inactive")
      return;

    if (!this->stopped_kitting_controllers)
      return;

    controller_manager_msgs::SwitchController srv;
    srv.request.stop_controllers = GetStaticControllers(nh, "kitting");
    ;
    srv.request.start_controllers.push_back("kitting_arm_controller");

    std::string srv_name = "/ariac/kitting/controller_manager/switch_controller";
    ros::ServiceClient client = nh.serviceClient<controller_manager_msgs::SwitchController>(srv_name);

    srv.request.strictness = 1;

    if (client.call(srv))
    {
      if (srv.response.ok)
      {
        ROS_INFO_STREAM("Started kitting controllers");
        this->stopped_kitting_controllers = false;
      }
      else
      {
        ROS_ERROR_STREAM("Unable to start controller");
      }
    }
  }

  void StartGantryRobot(ros::NodeHandle& nh)
  {
    if (this->gantry_robot_health == "inactive")
      return;

    if (!this->stopped_gantry_controllers)
      return;

    controller_manager_msgs::SwitchController srv;
    srv.request.stop_controllers = GetStaticControllers(nh, "gantry");
    ;
    srv.request.start_controllers.push_back("gantry_arm_controller");

    std::string srv_name = "/ariac/gantry/controller_manager/switch_controller";
    ros::ServiceClient client = nh.serviceClient<controller_manager_msgs::SwitchController>(srv_name);

    srv.request.strictness = 1;

    if (client.call(srv))
    {
      if (srv.response.ok)
      {
        ROS_INFO_STREAM("Started gantry controllers");
        this->stopped_gantry_controllers = false;
      }
      else
      {
        ROS_ERROR_STREAM("Unable to start controller");
      }
    }
  }

private:
  // subscriber to retrieve the status of both robots
  ros::Subscriber robot_health_sub;
  // robot health updated from /ariac/robot_health topic
  std::string kitting_robot_health;
  // robot health updated from /ariac/robot_health topic
  std::string gantry_robot_health;
  // True if the controllers for the kitting robot have already been stopped
  bool stopped_kitting_controllers;
  // True if the controllers for the gantry robot have already been stopped
  bool stopped_gantry_controllers;
  ros::Time kitting_lock_countdown_begin;
  ros::Time gantry_lock_countdown_begin;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "controller_switcher");
  ros::NodeHandle node;
  RobotControllerSwitcher rcs = RobotControllerSwitcher(&node);

  ros::Rate rate(1);

  while (ros::ok())
  {
    // ROS_INFO_STREAM("STATUS");
    rcs.StopKittingRobot(node);
    rcs.StopGantryRobot(node);
    rcs.StartKittingRobot(node);
    rcs.StartGantryRobot(node);

    rate.sleep();
    ros::spinOnce();
  }

  return 0;
}