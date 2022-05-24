#include "RobotControllerSwitcherPlugin.hh"
// standard library
#include <algorithm>
#include <chrono>
#include <cstdlib>
#include <limits>
#include <mutex>
#include <iostream>
#include <string>
#include <thread>
#include <vector>
// gazebo
#include <gazebo/common/Assert.hh>
#include <gazebo/common/Console.hh>
#include <gazebo/common/Events.hh>
#include <gazebo/msgs/gz_string.pb.h>
#include <gazebo/physics/ContactManager.hh>
#include <gazebo/physics/PhysicsEngine.hh>
#include <gazebo/physics/PhysicsTypes.hh>
#include <gazebo/physics/World.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/util/LogRecord.hh>
#include "gazebo_msgs/GetModelState.h"
#include "gazebo_msgs/GetLinkState.h"
#include "gazebo_msgs/ModelStates.h"
// ros
// #include "geometry_msgs/Pose.h"
// #include <ignition/math/Pose3.hh>
#include <ros/ros.h>
// #include <sdf/sdf.hh>
// #include <std_msgs/Float32.h>
// #include <std_msgs/String.h>
// #include <std_msgs/Bool.h>
// #include <std_srvs/Trigger.h>
// #include <tf/transform_listener.h>
// gear
// #include "nist_gear/ARIAC.hh"
// #include "nist_gear/TaskManagerPlugin.hh"
// #include "nist_gear/AriacScorer.h"
// #include "nist_gear/ConveyorBeltControl.h"
// #include "nist_gear/DetectKittingShipment.h"
// #include "nist_gear/DetectMovableTray.h"
// #include "nist_gear/DetectAssemblyShipment.h"
// #include "nist_gear/ExpectedKittingShipment.h"
// #include "nist_gear/DetectedKittingShipment.h"
// #include "nist_gear/AssemblyShipment.h"
// #include "nist_gear/Product.h"
// #include "nist_gear/MovableTray.h"
// #include "nist_gear/Order.h"
// #include "nist_gear/RobotHealth.h"
// #include "nist_gear/VacuumGripperState.h"
// #include "nist_gear/DropProducts.h"
// #include "nist_gear/SubmitKittingShipment.h"
// #include "nist_gear/MoveToStation.h"
// #include "nist_gear/MovableTrayInOrder.h"
// #include "nist_gear/TrayContents.h"
// #include "nist_gear/AgvInOrder.h"
// #include "nist_gear/KittingInOrder.h"
// #include "nist_gear/ProductInOrder.h"
// #include "nist_gear/Orders.h"
// #include "nist_gear/AssemblyInOrder.h"
// #include <nist_gear/DetectKitTrayContent.h>

namespace gazebo
{
/**
 * @internal
 * @struct RobotControllerSwitcherPluginPrivate
 * @brief Private data for the RobotControllerSwitcherPlugin class.
 */
struct RobotControllerSwitcherPluginPrivate
{
public:
  //!@brief World pointer
  physics::WorldPtr world;
  //!@brief SDF pointer
  sdf::ElementPtr sdf;
  transport::NodePtr gz_node;
  event::ConnectionPtr connection;
  ros::NodeHandle rosNode;
  ros::Subscriber robotHealthSubscriber;
  //!@brief Health status of the kitting robot
  bool kitting_robot_health;
  //!@brief Health status of the gantry robot
  bool assembly_robot_health;
  ros::ServiceClient gantry_controller_manager_client;
  /*!< Controller manager service to switch controllers for the kitting robot. */
  ros::ServiceClient kitting_controller_manager_client;
  std::mutex mutex;
  bool stopped_kitting_controllers;
  bool stopped_gantry_controllers;

  ros::ServiceClient kitting_switch_client;
  std::vector<std::string> gantry_static_controllers;
  std::vector<std::string> kitting_static_controllers;
  // std::vector<std::string> gantry_static_controllers{ "static_shoulder_pan_joint_controller",
  //                                                     "static_shoulder_lift_joint_controller",
  //                                                     "static_elbow_joint_controller",
  //                                                     "static_wrist_1_controller",
  //                                                     "static_wrist_2_controller",
  //                                                     "static_wrist_3_controller" };
  
  // std::vector<std::string> kitting_static_controllers{ "static_linear_arm_actuator_controller",
  //                                                      "static_shoulder_pan_joint_controller",
  //                                                      "static_shoulder_lift_joint_controller",
  //                                                      "static_elbow_joint_controller",
  //                                                      "static_wrist_1_controller",
  //                                                      "static_wrist_2_controller",
  //                                                      "static_wrist_3_controller" };
};
}  // namespace gazebo

using namespace gazebo;

// Register this plugin with the simulator
GZ_REGISTER_WORLD_PLUGIN(RobotControllerSwitcherPlugin)

RobotControllerSwitcherPlugin::RobotControllerSwitcherPlugin() : data_ptr(new RobotControllerSwitcherPluginPrivate)
{
}

//////////////////////////////////////////////////
RobotControllerSwitcherPlugin::~RobotControllerSwitcherPlugin()
{
  this->data_ptr->rosNode.shutdown();
}

//////////////////////////////////////////////////
void RobotControllerSwitcherPlugin::Load(physics::WorldPtr _world, sdf::ElementPtr _sdf)
{
  GZ_ASSERT(_world, "TaskManagerPlugin world pointer is NULL");
  GZ_ASSERT(_sdf, "TaskManagerPlugin sdf pointer is NULL");
  this->data_ptr->world = _world;
  this->data_ptr->sdf = _sdf;

  // Initialize Gazebo transport.
  this->data_ptr->gz_node = transport::NodePtr(new transport::Node());
  this->data_ptr->gz_node->Init();
  // initialize ROS
  // this->data_ptr->rosNode.reset(new ros::NodeHandle("switch_controller_plugin"));

  this->data_ptr->kitting_robot_health = 1;
  this->data_ptr->assembly_robot_health = 1;

  this->data_ptr->robotHealthSubscriber = this->data_ptr->rosNode.subscribe(
      "/ariac/robot_health", 1000, &RobotControllerSwitcherPlugin::OnRobotHealthContent, this);

  // controller manager
  // gantry
  std::string gantry_controller_service = "/ariac/gantry/controller_manager/switch_controller";
  this->data_ptr->gantry_controller_manager_client =
      this->data_ptr->rosNode.serviceClient<controller_manager_msgs::SwitchController>(gantry_controller_service);

  // kitting
  std::string kitting_controller_service = "/ariac/kitting/controller_manager/switch_controller";
  this->data_ptr->kitting_controller_manager_client =
      this->data_ptr->rosNode.serviceClient<controller_manager_msgs::SwitchController>(kitting_controller_service);

  this->data_ptr->stopped_kitting_controllers = false;
  this->data_ptr->stopped_gantry_controllers = false;

  
  std::string srv_name = "/ariac/kitting/controller_manager/switch_controller";
  this->data_ptr->kitting_switch_client =
    this->data_ptr->rosNode.serviceClient<controller_manager_msgs::SwitchController>(srv_name);

  


  this->data_ptr->connection =
      event::Events::ConnectWorldUpdateEnd(boost::bind(&RobotControllerSwitcherPlugin::OnUpdate, this));

  // GetStaticControllers();
}

void RobotControllerSwitcherPlugin::OnRobotHealthContent(nist_gear::RobotHealth msg)
{
  std::lock_guard<std::mutex> lock(this->data_ptr->mutex);
  this->data_ptr->kitting_robot_health = msg.kitting_robot_health;
  this->data_ptr->assembly_robot_health = msg.assembly_robot_health;
}


void RobotControllerSwitcherPlugin::GetStaticControllers(std::string robot_name) {
    std::vector<std::string> static_controllers;

    std::string srv_name = "/ariac/" + robot_name + "/controller_manager/list_controllers";
    ros::ServiceClient client = this->data_ptr->rosNode.serviceClient<controller_manager_msgs::ListControllers>(srv_name);
    controller_manager_msgs::ListControllers srv;

    if (client.call(srv)){
      for (auto state : srv.response.controller) {
            if (state.name.rfind("static", 0) == 0){
                static_controllers.push_back(state.name);
            }
        }
    }
    else {
        ROS_ERROR_STREAM("Unable to call " << srv_name);
    }

    if (robot_name == "kitting") {
      this->data_ptr->kitting_static_controllers = static_controllers;
    }
    else if (robot_name == "gantry") {
      this->data_ptr->gantry_static_controllers = static_controllers;
    }
}


//////////////////////////////////////////////////
void RobotControllerSwitcherPlugin::Init()
{
}

//////////////////////////////////////////////////
void RobotControllerSwitcherPlugin::OnUpdate()
{
  // GetStaticControllers();
    StopKittingRobot();
    StopGantryRobot();
  // gzdbg << "RobotControllerPlugin" << std::endl;
}

void RobotControllerSwitcherPlugin::StopKittingRobot()
{
  if (this->data_ptr->kitting_robot_health == 1) {
    return;
  }

  if (this->data_ptr->stopped_kitting_controllers) {
    return;
  }
  GetStaticControllers("kitting");
  gzerr << "kitting_robot_health: " << this->data_ptr->kitting_robot_health << std::endl;
  controller_manager_msgs::SwitchController srv;

  srv.request.start_controllers = this->data_ptr->kitting_static_controllers;
  // gzerr << "static controllers: " << this->data_ptr->kitting_static_controllers << std::endl;
  for (const auto& i : this->data_ptr->kitting_static_controllers) {
    gzerr << i << std::endl;
  }

  srv.request.stop_controllers.push_back("kitting_arm_controller");

  // std::string srv_name = "/ariac/kitting/controller_manager/switch_controller";
  // ros::ServiceClient kitting_switch_client =
  //     this->data_ptr->rosNode.serviceClient<controller_manager_msgs::SwitchController>(srv_name);

  srv.request.strictness = controller_manager_msgs::SwitchControllerRequest::BEST_EFFORT;

  this->data_ptr->kitting_switch_client.waitForExistence();
  // ROS_INFO_STREAM("Service available.");

  if (this->data_ptr->kitting_switch_client.call(srv))
  {
    if (srv.response.ok)
    {
      ROS_INFO_STREAM("Stopped kitting controller");
      this->data_ptr->stopped_kitting_controllers = true;
    }
    else
    {
      ROS_ERROR_STREAM("Unable to stop controller");
    }
  }
}

void RobotControllerSwitcherPlugin::StopGantryRobot()
{
  if (this->data_ptr->assembly_robot_health == 1) {
    return;
  }
  if (this->data_ptr->stopped_gantry_controllers) {
    return;
  }
  GetStaticControllers("gantry");
  controller_manager_msgs::SwitchController srv;

  srv.request.start_controllers = this->data_ptr->gantry_static_controllers;
  srv.request.stop_controllers.push_back("gantry_arm_controller");

  std::string srv_name = "/ariac/gantry/controller_manager/switch_controller";
  ros::ServiceClient client =
      this->data_ptr->rosNode.serviceClient<controller_manager_msgs::SwitchController>(srv_name);

  srv.request.strictness = controller_manager_msgs::SwitchControllerRequest::BEST_EFFORT;

  if (client.call(srv))
  {
    if (srv.response.ok)
    {
      ROS_INFO_STREAM("Stopped gantry controller");
      this->data_ptr->stopped_gantry_controllers = true;
    }
    else
    {
      ROS_ERROR_STREAM("Unable to stop controller");
    }
  }
}

/////////////////////////////////////////////////
void RobotControllerSwitcherPlugin::StartRobot(std::string robot_name)
{
  controller_manager_msgs::SwitchController srv;
  std::vector<std::string> static_controllers;
  if (robot_name.compare("kitting") == 0)
  {
    static_controllers = this->data_ptr->kitting_static_controllers;
  }
  else if (robot_name.compare("gantry") == 0)
  {
    static_controllers = this->data_ptr->gantry_static_controllers;
  }
  srv.request.start_controllers.push_back(robot_name + "_arm_controller");

  std::string srv_name = "/ariac/" + robot_name + "/controller_manager/switch_controller";
  ros::ServiceClient client =
      this->data_ptr->rosNode.serviceClient<controller_manager_msgs::SwitchController>(srv_name);

  srv.request.strictness = 1;

  if (client.call(srv))
  {
    if (srv.response.ok)
    {
      ROS_INFO_STREAM("Started " << robot_name << " controller");
    }
    else
    {
      ROS_ERROR_STREAM("Unable to start controller");
    }
  }
}