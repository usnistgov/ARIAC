// Copyright 2016 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.


#include <algorithm>
#include <vector>
#include <string>
#include <ros/ros.h>

#include <nist_gear/LogicalCameraImage.h>
#include <nist_gear/Order.h>
#include <nist_gear/Proximity.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/Range.h>
#include <std_msgs/Float32.h>
#include <std_msgs/String.h>
#include <std_srvs/Trigger.h>
#include <trajectory_msgs/JointTrajectory.h>



/**
 * @brief Structure of a product
 * 
 */
typedef struct Product
{
    std::string name;
    geometry_msgs::Pose frame_pose;
    geometry_msgs::Pose world_pose;
    std::string agv;
    std::string kit_tray;
}
product;

/**
 * @brief Structure of a shipment
 * 
 */
typedef struct Shipment
{
    std::string shipment_type;
    std::string agv;
    std::vector<Product> products;
}
shipment;

/**
 * @brief Structure of an order
 * 
 */
typedef struct Order
{
    std::string order_id;
    std::vector<Shipment> shipments;
}
order;

/**
 * @brief Start the competition by waiting for and then calling the start ROS Service.
 * 
 * @param node Nodehandle
 */
void start_competition(ros::NodeHandle & node)
{
  // create a Service client for the correct service, i.e. '/ariac/start_competition'.
  ros::ServiceClient start_client =
    node.serviceClient<std_srvs::Trigger>("/ariac/start_competition");
  // if it's not already ready, wait for it to be ready.
  // calling the Service using the client before the server is ready would fail.
  if (!start_client.exists())
  {
    ROS_INFO("Waiting for the competition to be ready...");
    start_client.waitForExistence();
    ROS_INFO("Competition is now ready.");
  }
  ROS_INFO("Requesting competition start...");
  std_srvs::Trigger srv;  // combination of the "request" and the "response".
  start_client.call(srv);  // call the start Service.
  // if not successful, print out why.
  if (!srv.response.success)
  {
    ROS_ERROR_STREAM("Failed to start the competition: " << srv.response.message);
  }
  else
  {
    ROS_INFO("Competition started!");
  }
}

/**
 * @brief Example class that can hold state and provide methods that handle incoming data.
 * 
 */
class MyCompetitionClass
{
public:
  explicit MyCompetitionClass(ros::NodeHandle & node)
  : current_score_(0)
  {
    gantry_arm_joint_trajectory_publisher_ = node.advertise<trajectory_msgs::JointTrajectory>(
      "/ariac/arm1/arm/command", 10);

    kitting_arm_joint_trajectory_publisher_ = node.advertise<trajectory_msgs::JointTrajectory>(
      "/ariac/arm2/arm/command", 10);
  }

  /// Called when a new message is received.
  void current_score_callback(const std_msgs::Float32::ConstPtr & msg)
  {
    if (msg->data != current_score_)
    {
      ROS_INFO_STREAM("Score: " << msg->data);
    }
    current_score_ = msg->data;
  }

  /// Called when a new message is received.
  void competition_state_callback(const std_msgs::String::ConstPtr & msg)
  {
    if (msg->data == "done" && competition_state_ != "done")
    {
      ROS_INFO("Competition ended.");
    }
    competition_state_ = msg->data;
  }

  /// Called when a new Order message is received.
  void order_callback(const nist_gear::Order::ConstPtr & order_msg)
  {
    ROS_INFO_STREAM("Received order:\n" << *order_msg);
    received_orders_.push_back(*order_msg);
  }

  /// Called when a new LogicalCameraImage message is received.
  void logical_camera_callback(
    const nist_gear::LogicalCameraImage::ConstPtr & image_msg)
  {
    ROS_INFO_STREAM_THROTTLE(10,
      "Logical camera: '" << image_msg->models.size() << "' objects.");
  }

  /// Called when a new Proximity message is received.
  void break_beam_callback(const nist_gear::Proximity::ConstPtr & msg) {
    if (msg->object_detected) {  // If there is an object in proximity.
      ROS_INFO("Break beam triggered.");
    }
  }

private:
  std::string competition_state_;
  double current_score_;
  ros::Publisher gantry_arm_joint_trajectory_publisher_;
  ros::Publisher kitting_arm_joint_trajectory_publisher_;
  std::vector<nist_gear::Order> received_orders_;
  sensor_msgs::JointState gantry_arm_current_joint_states_;
  sensor_msgs::JointState kitting_arm_current_joint_states_;
};

void proximity_sensor_callback(const sensor_msgs::Range::ConstPtr & msg)
{
  if ((msg->max_range - msg->range) > 0.01)
  {  // If there is an object in proximity.
    ROS_INFO_THROTTLE(1, "Proximity sensor sees something.");
  }
}

void laser_profiler_callback(const sensor_msgs::LaserScan::ConstPtr & msg)
{
  size_t number_of_valid_ranges = std::count_if(
    msg->ranges.begin(), msg->ranges.end(), [](const float f)
    {
      return std::isfinite(f);
      });
  if (number_of_valid_ranges > 0)
  {
    ROS_INFO_THROTTLE(1, "Laser profiler sees something.");
  }
}


int main(int argc, char ** argv)
{
  // Last argument is the default name of the node.
  ros::init(argc, argv, "ariac_example_node");

  ros::NodeHandle node;

  // Instance of custom class from above.
  MyCompetitionClass comp_class(node);

  // Subscribe to the '/ariac/current_score' topic.
  ros::Subscriber current_score_subscriber = node.subscribe(
    "/ariac/current_score", 10,
    &MyCompetitionClass::current_score_callback, &comp_class);

  // Subscribe to the '/ariac/competition_state' topic.
  ros::Subscriber competition_state_subscriber = node.subscribe(
    "/ariac/competition_state", 10,
    &MyCompetitionClass::competition_state_callback, &comp_class);

  // %Tag(SUB_CLASS)%
  // Subscribe to the '/ariac/orders' topic.
  ros::Subscriber orders_subscriber = node.subscribe(
    "/ariac/orders", 10,
    &MyCompetitionClass::order_callback, &comp_class);

  // Subscribe to the '/ariac/proximity_sensor_1' topic.
  ros::Subscriber proximity_sensor_subscriber = node.subscribe(
    "/ariac/proximity_sensor_1", 10, proximity_sensor_callback);
  // %EndTag(SUB_FUNC)%

  // Subscribe to the '/ariac/break_beam_1_change' topic.
  ros::Subscriber break_beam_subscriber = node.subscribe(
    "/ariac/break_beam_1_change", 10,
    &MyCompetitionClass::break_beam_callback, &comp_class);

  // Subscribe to the '/ariac/logical_camera_1' topic.
  ros::Subscriber logical_camera_subscriber = node.subscribe(
    "/ariac/logical_camera_1", 10,
    &MyCompetitionClass::logical_camera_callback, &comp_class);

  // Subscribe to the '/ariac/laser_profiler_1' topic.
  ros::Subscriber laser_profiler_subscriber = node.subscribe(
    "/ariac/laser_profiler_1", 10, laser_profiler_callback);

  ROS_INFO("Setup complete.");
  start_competition(node);
  ros::spin();  // This executes callbacks on new data until ctrl-c.

  return 0;
}
