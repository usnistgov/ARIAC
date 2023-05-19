/*
This software was developed by employees of the National Institute of Standards and Technology (NIST), an agency of the Federal Government. Pursuant to title 17 United States Code Section 105, works of NIST employees are not subject to copyright protection in the United States and are considered to be in the public domain. Permission to freely use, copy, modify, and distribute this software and its documentation without fee is hereby granted, provided that this notice and disclaimer of warranty appears in all copies.

The software is provided 'as is' without any warranty of any kind, either expressed, implied, or statutory, including, but not limited to, any warranty that the software will conform to specifications, any implied warranties of merchantability, fitness for a particular purpose, and freedom from infringement, and any warranty that the documentation will conform to the software, or any warranty that the software will be error free. In no event shall NIST be liable for any damages, including, but not limited to, direct, indirect, special or consequential damages, arising out of, resulting from, or in any way connected with this software, whether or not based upon warranty, contract, tort, or otherwise, whether or not injury was sustained by persons or property or otherwise, and whether or not loss was sustained from, or arose out of the results of, or use of, the software or services provided hereunder.

Distributions of NIST software should also include copyright and licensing statements of any third-party software that are legally bundled with the code in compliance with the conditions of those licenses.
*/

// Gazebo
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/World.hh>
#include <gazebo_ros/node.hpp>
#include <gazebo/transport/Subscriber.hh>
#include <gazebo/transport/Node.hh>
#include <gazebo/sensors/LogicalCameraSensor.hh>
#include <gazebo_ros/conversions/builtin_interfaces.hpp>
#include <gazebo_ros/conversions/geometry_msgs.hpp>

// Messages
#include <ariac_plugins/task_manager_plugin.hpp>
#include <ariac_msgs/msg/trial.hpp>
#include <ariac_msgs/msg/order.hpp>
#include <ariac_msgs/msg/order_condition.hpp>
#include <ariac_msgs/msg/challenge.hpp>
#include <ariac_msgs/msg/sensors.hpp>
#include <ariac_msgs/msg/condition.hpp>
#include <ariac_msgs/msg/kitting_task.hpp>
#include <ariac_msgs/msg/kitting_part.hpp>
#include <ariac_msgs/msg/assembly_task.hpp>
#include <ariac_msgs/msg/assembly_part.hpp>
#include <ariac_msgs/msg/combined_task.hpp>
#include <ariac_msgs/msg/part.hpp>
#include <ariac_msgs/msg/parts.hpp>
#include <ariac_msgs/msg/robots.hpp>
#include <ariac_msgs/msg/competition_state.hpp>
#include <ariac_msgs/msg/quality_issue.hpp>
#include <ariac_msgs/msg/agv_status.hpp>
#include <ariac_msgs/srv/submit_order.hpp>
#include <ariac_msgs/srv/perform_quality_check.hpp>
#include <ariac_msgs/srv/get_pre_assembly_poses.hpp>
#include <std_msgs/msg/bool.hpp>
#include <ariac_msgs/srv/get_pre_assembly_poses.hpp>
// ROS
#include <rclcpp/rclcpp.hpp>
#include <controller_manager_msgs/srv/switch_controller.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
// C++
#include <memory>
#include <iostream>
#include <cstdlib>
// ARIAC
#include <ariac_plugins/ariac_common.hpp>

namespace ariac_plugins
{
    /// Class to hold private data members (PIMPL pattern)
    class TaskManagerPluginPrivate
    {
    public:
        gazebo::transport::NodePtr gznode_;

        //============== C++ =================
        /*!< A mutex to protect the current state. */
        std::mutex lock_;
        /*!< Pointer to the current state. */
        unsigned int current_state_{ariac_msgs::msg::CompetitionState::IDLE};
        /*!< Score for the whole trial. */
        double trial_score_;
        /*!< Time limit for the current trial. */
        double time_limit_;
        /*!< Name of the trial file. */
        std::string trial_name_;
        /*< Health status for the break beam*/
        bool break_beam_sensor_health_;
        /*< Health status for the proximity sensor*/
        bool proximity_sensor_health_;
        /*< Health status for the laser profiler sensor*/
        bool laser_profiler_sensor_health_;
        /*< Health status for the laser profiler sensor*/
        bool lidar_sensor_health_;
        /*< Health status for the camera sensor*/
        bool camera_sensor_health_;
        /*< Health status for the logical camera sensor*/
        bool logical_camera_sensor_health_;
        /*< Health status for the ceiling robot*/
        bool ceiling_robot_health_;
        /*< Health status for the floor robot*/
        bool floor_robot_health_;

        //============== GAZEBO =================
        /*!< Connection to world update event. Callback is called while this is alive. */
        gazebo::event::ConnectionPtr update_connection_;
        /*!< ROS node. */
        gazebo_ros::Node::SharedPtr ros_node_{nullptr};
        /*!< Pointer to the world. */
        gazebo::physics::WorldPtr world_;
        /*!< Pointer to the sdf tag. */
        sdf::ElementPtr sdf_;
        /*!< Time since the start competition service is called. */
        gazebo::common::Time start_competition_time_;
        gazebo::common::Time last_sim_time_;
        gazebo::common::Time last_on_update_time_;

        //============== MISC ATTRIBUTES =================
        bool competition_time_set_{false};
        /*< Counter to keep track of the total number of orders in the current trial. */
        int total_orders_{-1};
        unsigned int agv1_location_;
        unsigned int agv2_location_;
        unsigned int agv3_location_;
        unsigned int agv4_location_;

        std::map<int, std::vector<ariac_common::Part>> agv_parts_;
        std::map<int, std::vector<ariac_common::Part>> insert_parts_;

        //============== ROS =================
        /*!< Time since when this plugin is loaded. */
        rclcpp::Time current_sim_time_;
        /*!< Time since the competition started. */
        double elapsed_time_;

        //============== SUBSCRIBERS =================
        /*!< Subscriber to topic: "/ariac/trial_config"*/
        rclcpp::Subscription<ariac_msgs::msg::Trial>::SharedPtr trial_config_sub_;
        /*!< Subscriber to topic: "/ariac/agv1_status"*/
        rclcpp::Subscription<ariac_msgs::msg::AGVStatus>::SharedPtr agv1_status_;
        /*!< Subscriber to topic: "/ariac/agv2_status"*/
        rclcpp::Subscription<ariac_msgs::msg::AGVStatus>::SharedPtr agv2_status_;
        /*!< Subscriber to topic: "/ariac/agv3_status"*/
        rclcpp::Subscription<ariac_msgs::msg::AGVStatus>::SharedPtr agv3_status_;
        /*!< Subscriber to topic: "/ariac/agv4_status"*/
        rclcpp::Subscription<ariac_msgs::msg::AGVStatus>::SharedPtr agv4_status_;

        gazebo::transport::SubscriberPtr agv1_tray_sub_;
        gazebo::transport::SubscriberPtr agv2_tray_sub_;
        gazebo::transport::SubscriberPtr agv3_tray_sub_;
        gazebo::transport::SubscriberPtr agv4_tray_sub_;

        gazebo::transport::SubscriberPtr station1_sub_;
        gazebo::transport::SubscriberPtr station2_sub_;
        gazebo::transport::SubscriberPtr station3_sub_;
        gazebo::transport::SubscriberPtr station4_sub_;

        //============== SERVICES =================
        /*!< Service to start the competition. */
        rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr start_competition_service_{nullptr};
        /*!< Service to end the competition. */
        rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr end_competition_service_{nullptr};
        /*!< Service to submit an order. */
        rclcpp::Service<ariac_msgs::srv::SubmitOrder>::SharedPtr submit_order_service_{nullptr};
        /*!< Service to perform a quality check. */
        rclcpp::Service<ariac_msgs::srv::PerformQualityCheck>::SharedPtr quality_check_service_;
        /*!< Service to get pre-assembly poses. */
        rclcpp::Service<ariac_msgs::srv::GetPreAssemblyPoses>::SharedPtr pre_assembly_poses_service_;
        /*!< Service to penalize the ceiling robot. */
        rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr human_safe_zone_penalty_service_;

        //============== PUBLISHERS =================
        /*!< Publisher to the topic /ariac/orders */
        rclcpp::Publisher<ariac_msgs::msg::Order>::SharedPtr order_pub_;
        /*!< Publisher to the topic /ariac/sensor_health */
        rclcpp::Publisher<ariac_msgs::msg::Sensors>::SharedPtr sensor_health_pub_;
        /*!< Publisher to the topic /ariac/competition_state */
        rclcpp::Publisher<ariac_msgs::msg::CompetitionState>::SharedPtr competition_state_pub_;
        /*!< Publisher to the topic /ariac/robot_health */
        rclcpp::Publisher<ariac_msgs::msg::Robots>::SharedPtr robot_health_pub_;
        /*!< Publisher to the topic /ariac/start_human */
        rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr start_human_pub_;

        // Reduce the rate of publishing to some topics
        rclcpp::Time last_publish_time_;
        int update_ns_;
        bool first_publish_;

        //============== Orders =================
        /*!< List of orders. that have already been submitted*/
        std::vector<std::string> submitted_orders_;
        /*!< List of orders. that have already been announced*/
        std::vector<std::string> announced_orders_;
        /*!< List of trial orders. */
        std::vector<std::string> trial_orders_;
        /*!< List of orders that are announced based on time. */
        std::vector<std::shared_ptr<ariac_common::OrderTemporal>>
            time_based_orders_;
        /*!< List of orders that are announced based on part placement. */
        std::vector<std::shared_ptr<ariac_common::OrderOnPartPlacement>> on_part_placement_orders_;
        /*!< List of orders that are announced based on submission. */
        std::vector<std::shared_ptr<ariac_common::OrderOnSubmission>> on_order_submission_orders_;
        /*!< List of all orders in the trial. */
        std::vector<std::shared_ptr<ariac_common::Order>> all_orders_;

        //============== Human Challenge =================
        /*!< human challenge announced based on time. */
        std::shared_ptr<ariac_common::HumanChallengeTemporal> time_based_human_challenge_;
        /*!< human challenge announced based on part placement. */
        std::shared_ptr<ariac_common::HumanChallengeOnPartPlacement> on_part_placement_human_challenge_;
        /*!< human challenge announced based on submission. */
        std::shared_ptr<ariac_common::HumanChallengeOnSubmission> on_order_submission_human_challenge_;
        /*!<Time at which the safe zone penalty started */
        double safe_zone_penalty_start_time_;
        /*!<Time at which the safe zone penalty ended */
        double safe_zone_penalty_end_time_;
        /*!<Flag for whether or not the safe zone penalty has started */
        bool safe_zone_penalty_started_;
        /*!<Duration for disabling the ceiling robot */
        double safe_zone_penalty_duration_;
        /*!<Ceiling robot grace period */
        double ceiling_robot_grace_period_;

        //============== Sensor Blackout Challenge =================
        /*!< List of sensor blackout challenges that are announced based on time. */
        std::vector<std::shared_ptr<ariac_common::SensorBlackoutTemporal>>
            time_based_sensor_blackouts_;
        /*!< List of sensor blackout challenges that are announced based on part placement. */
        std::vector<std::shared_ptr<ariac_common::SensorBlackoutOnPartPlacement>> on_part_placement_sensor_blackouts_;
        /*!< List of sensor blackout challenges that are announced based on submission. */
        std::vector<std::shared_ptr<ariac_common::SensorBlackoutOnSubmission>> on_submission_sensor_blackouts_;
        /*!< List of sensor blackout challenges that have been started. */
        std::vector<std::shared_ptr<ariac_common::SensorBlackout>> in_progress_sensor_blackouts_;

        //============== Robot Malfunction Challenge =================
        /*!< List of robot malfunction challenges that are announced based on time. */
        std::vector<std::shared_ptr<ariac_common::RobotMalfunctionTemporal>>
            time_based_robot_malfunctions_;
        /*!< List of robot malfunction challenges that are announced based on part placement. */
        std::vector<std::shared_ptr<ariac_common::RobotMalfunctionOnPartPlacement>> on_part_placement_robot_malfunctions_;
        /*!< List of robot malfunction challenges that are announced based on submission. */
        std::vector<std::shared_ptr<ariac_common::RobotMalfunctionOnSubmission>> on_submission_robot_malfunctions_;
        /*!< List of robot malfunction challenges that have been started. */
        std::vector<std::shared_ptr<ariac_common::RobotMalfunction>> in_progress_robot_malfunctions_;

        //============== Faulty Part Challenge =================
        /*!< List of faulty part challenges. */
        std::vector<std::shared_ptr<ariac_common::FaultyPartChallenge>> faulty_part_challenges_;
        /*!< Array for faulty part information for each quadrant. */
        std::array<bool, 4> HandleFaultyParts(
            std::shared_ptr<ariac_common::FaultyPartChallenge> _challenge,
            ariac_common::KittingTask _task,
            ariac_common::KittingShipment _shipment);

        //============== Tray and AGV Sensors =================
        /*!< Map of AGV trays. The key is the AGV ID and the value is information about the tray*/
        std::map<int, gazebo::msgs::LogicalCameraImage> agv_tray_images_;
        /*!< Map of Assembly Station images. The key is the AGV ID and the value is the camera image.*/
        std::map<int, gazebo::msgs::LogicalCameraImage> assembly_station_images_;

        //============== Callbacks for Tray and AGVs Cameras =================
        /*!< Callback to parse the tray located on AGV1. */
        void AGV1TraySensorCallback(ConstLogicalCameraImagePtr &_msg);
        /*!< Callback to parse the tray located on AGV2. */
        void AGV2TraySensorCallback(ConstLogicalCameraImagePtr &_msg);
        /*!< Callback to parse the tray located on AGV3. */
        void AGV3TraySensorCallback(ConstLogicalCameraImagePtr &_msg);
        /*!< Callback to parse the tray located on AGV4. */
        void AGV4TraySensorCallback(ConstLogicalCameraImagePtr &_msg);
        /*!< Callback to logical camera above assembly station 1. */
        void Station1SensorCallback(ConstLogicalCameraImagePtr &_msg);
        /*!< Callback to logical camera above assembly station 2. */
        void Station2SensorCallback(ConstLogicalCameraImagePtr &_msg);
        /*!< Callback to logical camera above assembly station 3. */
        void Station3SensorCallback(ConstLogicalCameraImagePtr &_msg);
        /*!< Callback to logical camera above assembly station 4. */
        void Station4SensorCallback(ConstLogicalCameraImagePtr &_msg);

        //============== Score Displays =================
        /*!< Print details of the score for an assembly task. */
        void PrintAssemblyScore(std::shared_ptr<ariac_common::AssemblyScore> assembly_score);
        /*!< Print details of the score for a combined task. */
        void PrintCombinedScore(std::shared_ptr<ariac_common::CombinedScore> combined_score);
        /*!< Print details of the score for a kitting task. */
        void PrintKittingScore(std::shared_ptr<ariac_common::KittingScore> kitting_score);
        /*!< Print details of the score for the trial. */
        void PrintTrialScore(
            std::shared_ptr<ariac_common::Order> kitting_order,
            std::shared_ptr<ariac_common::Order> assembly_order,
            std::shared_ptr<ariac_common::Order> combined_order);

        /**
         * @brief Retrieve the tray information from the message and store it in the agv_parts_ map.
         * @param _msg The message containing the tray information.
         */
        void StoreAGVParts(int agv_id, gazebo::msgs::LogicalCameraImage &_msg);

        /**
         * @brief Retrieve the insert information from the message and store it in the insert_parts_ map.
         * @param _msg The message containing the insert information.
         */
        void StoreStationParts(int station, gazebo::msgs::LogicalCameraImage &_msg);

        int ScoreQuadrant(bool is_correct_part_type, bool is_correct_part_color, bool is_flipped_part, bool is_faulty_part);

        void ScoreKittingTask(std::shared_ptr<ariac_common::Order> _order, std::string _submitted_order_id);
        void ScoreAssemblyTask(std::shared_ptr<ariac_common::Order> _order, std::string _submitted_order_id);
        void ScoreCombinedTask(std::shared_ptr<ariac_common::Order> _order, std::string _submitted_order_id);
        ariac_common::KittingShipment ParseAGVTraySensorImage(gazebo::msgs::LogicalCameraImage &_msg);
        ariac_common::AssemblyShipment ParseAssemblyStationImage(gazebo::msgs::LogicalCameraImage &_msg);
        ariac_msgs::msg::QualityIssue CheckQuadrantQuality(int quadrant, ariac_common::KittingTask task, ariac_common::KittingShipment shipment, std::string order_id);

        void PerformQualityCheck(ariac_msgs::srv::PerformQualityCheck::Request::SharedPtr request,
                                 ariac_msgs::srv::PerformQualityCheck::Response::SharedPtr response);

        void GetPreAssemblyPoses(ariac_msgs::srv::GetPreAssemblyPoses::Request::SharedPtr request,
                                 ariac_msgs::srv::GetPreAssemblyPoses::Response::SharedPtr response);

        std::map<std::string, int> part_types_ = {{"battery", ariac_msgs::msg::Part::BATTERY},
                                                  {"pump", ariac_msgs::msg::Part::PUMP},
                                                  {"sensor", ariac_msgs::msg::Part::SENSOR},
                                                  {"regulator", ariac_msgs::msg::Part::REGULATOR}};

        std::map<std::string, int> part_colors_ = {{"red", ariac_msgs::msg::Part::RED},
                                                   {"green", ariac_msgs::msg::Part::GREEN},
                                                   {"blue", ariac_msgs::msg::Part::BLUE},
                                                   {"orange", ariac_msgs::msg::Part::ORANGE},
                                                   {"purple", ariac_msgs::msg::Part::PURPLE}};

        void WriteToLog();
        std::string log_message_ = "";
    };
    //==============================================================================
    TaskManagerPlugin::TaskManagerPlugin()
        : impl_(std::make_unique<TaskManagerPluginPrivate>())
    {
    }
    //==============================================================================
    TaskManagerPlugin::~TaskManagerPlugin()
    {
        impl_->ros_node_.reset();
    }

    void TaskManagerPluginPrivate::WriteToLog()
    {
        // Get the environment variable INSIDE_DOCKER
        auto inside_docker = std::getenv("INSIDE_DOCKER");

        // compare inside_docker to "ok"
        if (inside_docker == NULL)
        {
            // RCLCPP_WARN_STREAM_ONCE(ros_node_->get_logger(), "INSIDE_DOCKER environment variable not set");
            return;
        }

        // Create a folder
        std::string folder_path = "/home/ubuntu/logs/";
        std::string command = "mkdir -p " + folder_path;
        system(command.c_str());

        // remove .yaml from trial name
        if (trial_name_.length() > 5)
        {
            trial_name_.erase(trial_name_.length() - 5);
        }

        // Create a file name with the trial name
        std::string file_name = trial_name_ + ".txt";
        auto log_file_path = folder_path + file_name;

        // Open the log file
        RCLCPP_WARN_STREAM_ONCE(ros_node_->get_logger(), "Generating log file: " << log_file_path);
        auto log_file = std::ofstream(log_file_path);
        // Write to log file
        log_file << log_message_;
        // Close the log file
        log_file.close();
    }

    //==============================================================================
    void
    TaskManagerPlugin::Load(gazebo::physics::WorldPtr _world, sdf::ElementPtr _sdf)
    {

        GZ_ASSERT(_world, "TaskManagerPlugin world pointer is NULL");
        GZ_ASSERT(_sdf, "TaskManagerPlugin sdf pointer is NULL");

        gzdbg << "ARIAC VERSION: v.05.30.2022\n";

        // // Create a GazeboRos node instead of a common ROS node.
        // // Pass it SDF parameters so common options like namespace and remapping
        // // can be handled.
        impl_->ros_node_ = gazebo_ros::Node::Get(_sdf);
        impl_->world_ = _world;
        impl_->sdf_ = _sdf;

        impl_->gznode_ = gazebo::transport::NodePtr(new gazebo::transport::Node());
        impl_->gznode_->Init(impl_->world_->Name());

        std::string tray1_topic = "/gazebo/world/agv_tray_sensor_1/sensor_base/agv_tray_sensor/models";
        std::string tray2_topic = "/gazebo/world/agv_tray_sensor_2/sensor_base/agv_tray_sensor/models";
        std::string tray3_topic = "/gazebo/world/agv_tray_sensor_3/sensor_base/agv_tray_sensor/models";
        std::string tray4_topic = "/gazebo/world/agv_tray_sensor_4/sensor_base/agv_tray_sensor/models";

        impl_->agv1_tray_sub_ = impl_->gznode_->Subscribe(tray1_topic, &TaskManagerPluginPrivate::AGV1TraySensorCallback, impl_.get());
        impl_->agv2_tray_sub_ = impl_->gznode_->Subscribe(tray2_topic, &TaskManagerPluginPrivate::AGV2TraySensorCallback, impl_.get());
        impl_->agv3_tray_sub_ = impl_->gznode_->Subscribe(tray3_topic, &TaskManagerPluginPrivate::AGV3TraySensorCallback, impl_.get());
        impl_->agv4_tray_sub_ = impl_->gznode_->Subscribe(tray4_topic, &TaskManagerPluginPrivate::AGV4TraySensorCallback, impl_.get());

        std::string station1_topic = "/gazebo/world/assembly_station_sensor_1/sensor_base/assembly_station_sensor/models";
        std::string station2_topic = "/gazebo/world/assembly_station_sensor_2/sensor_base/assembly_station_sensor/models";
        std::string station3_topic = "/gazebo/world/assembly_station_sensor_3/sensor_base/assembly_station_sensor/models";
        std::string station4_topic = "/gazebo/world/assembly_station_sensor_4/sensor_base/assembly_station_sensor/models";

        impl_->station1_sub_ = impl_->gznode_->Subscribe(station1_topic, &TaskManagerPluginPrivate::Station1SensorCallback, impl_.get());
        impl_->station2_sub_ = impl_->gznode_->Subscribe(station2_topic, &TaskManagerPluginPrivate::Station2SensorCallback, impl_.get());
        impl_->station3_sub_ = impl_->gznode_->Subscribe(station3_topic, &TaskManagerPluginPrivate::Station3SensorCallback, impl_.get());
        impl_->station4_sub_ = impl_->gznode_->Subscribe(station4_topic, &TaskManagerPluginPrivate::Station4SensorCallback, impl_.get());

        RCLCPP_INFO(impl_->ros_node_->get_logger(), "Starting ARIAC 2023");

        // Get QoS profiles
        const gazebo_ros::QoS &qos = impl_->ros_node_->get_qos();

        // Create a connection so the OnUpdate function is called at every simulation
        // iteration. Remove this call, the connection and the callback if not needed.
        impl_->update_connection_ = gazebo::event::Events::ConnectWorldUpdateBegin(
            std::bind(&TaskManagerPlugin::OnUpdate, this));

        // Init subscribers
        impl_->trial_config_sub_ = impl_->ros_node_->create_subscription<ariac_msgs::msg::Trial>(
            "/ariac/trial_config", qos.get_subscription_qos("/ariac/trial_config", rclcpp::QoS(1)),
            std::bind(&TaskManagerPlugin::OnTrialCallback, this, std::placeholders::_1));

        impl_->agv1_status_ = impl_->ros_node_->create_subscription<ariac_msgs::msg::AGVStatus>(
            "/ariac/agv1_status", qos.get_subscription_qos("/ariac/agv1_status", rclcpp::QoS(1)),
            std::bind(&TaskManagerPlugin::OnAGV1StatusCallback, this, std::placeholders::_1));

        impl_->agv2_status_ = impl_->ros_node_->create_subscription<ariac_msgs::msg::AGVStatus>(
            "/ariac/agv2_status", qos.get_subscription_qos("/ariac/agv2_status", rclcpp::QoS(1)),
            std::bind(&TaskManagerPlugin::OnAGV2StatusCallback, this, std::placeholders::_1));

        impl_->agv3_status_ = impl_->ros_node_->create_subscription<ariac_msgs::msg::AGVStatus>(
            "/ariac/agv3_status", qos.get_subscription_qos("/ariac/agv3_status", rclcpp::QoS(1)),
            std::bind(&TaskManagerPlugin::OnAGV3StatusCallback, this, std::placeholders::_1));

        impl_->agv4_status_ = impl_->ros_node_->create_subscription<ariac_msgs::msg::AGVStatus>(
            "/ariac/agv4_status", qos.get_subscription_qos("/ariac/agv4_status", rclcpp::QoS(1)),
            std::bind(&TaskManagerPlugin::OnAGV4StatusCallback, this, std::placeholders::_1));

        // Init publishers
        impl_->start_human_pub_ = impl_->ros_node_->create_publisher<std_msgs::msg::Bool>("/ariac/start_human", 10);
        impl_->sensor_health_pub_ = impl_->ros_node_->create_publisher<ariac_msgs::msg::Sensors>("/ariac/sensor_health", 10);
        impl_->robot_health_pub_ = impl_->ros_node_->create_publisher<ariac_msgs::msg::Robots>("/ariac/robot_health", 10);
        impl_->competition_state_pub_ = impl_->ros_node_->create_publisher<ariac_msgs::msg::CompetitionState>("/ariac/competition_state", 10);
        impl_->order_pub_ = impl_->ros_node_->create_publisher<ariac_msgs::msg::Order>("/ariac/orders", 1);
        // Init sensor health
        impl_->break_beam_sensor_health_ = true;
        impl_->proximity_sensor_health_ = true;
        impl_->laser_profiler_sensor_health_ = true;
        impl_->lidar_sensor_health_ = true;
        impl_->camera_sensor_health_ = true;
        impl_->logical_camera_sensor_health_ = true;
        // Init robot health
        impl_->ceiling_robot_health_ = true;
        impl_->floor_robot_health_ = true;
        // Init time limit
        impl_->time_limit_ = -1;
        // Init trial score
        impl_->trial_score_ = 0;
        // Init flag for safe zone penalty
        impl_->safe_zone_penalty_started_ = false;
        // Init safe zone penalty end time
        impl_->safe_zone_penalty_end_time_ = -1;
        // Init safe zone penalty duration
        impl_->safe_zone_penalty_duration_ = 15.0;
        // Init safe zone penalty grace period
        impl_->ceiling_robot_grace_period_ = 10.0;
        // Init elapsed time
        impl_->elapsed_time_ = 0.0;
        double publish_rate = 10;
        impl_->update_ns_ = int((1 / publish_rate) * 1e9);
        impl_->first_publish_ = true;
    }

    //==============================================================================
    void TaskManagerPluginPrivate::AGV1TraySensorCallback(ConstLogicalCameraImagePtr &_msg)
    {
        agv_tray_images_.insert_or_assign(1, *_msg);
    }

    //==============================================================================
    void TaskManagerPluginPrivate::AGV2TraySensorCallback(ConstLogicalCameraImagePtr &_msg)
    {
        agv_tray_images_.insert_or_assign(2, *_msg);
    }

    //==============================================================================
    void TaskManagerPluginPrivate::AGV3TraySensorCallback(ConstLogicalCameraImagePtr &_msg)
    {
        agv_tray_images_.insert_or_assign(3, *_msg);
    }

    //==============================================================================
    void TaskManagerPluginPrivate::AGV4TraySensorCallback(ConstLogicalCameraImagePtr &_msg)
    {
        agv_tray_images_.insert_or_assign(4, *_msg);
    }

    //==============================================================================
    void TaskManagerPluginPrivate::Station1SensorCallback(ConstLogicalCameraImagePtr &_msg)
    {
        assembly_station_images_.insert_or_assign(1, *_msg);
    }
    //==============================================================================
    void TaskManagerPluginPrivate::Station2SensorCallback(ConstLogicalCameraImagePtr &_msg)
    {
        assembly_station_images_.insert_or_assign(2, *_msg);
    }
    //==============================================================================
    void TaskManagerPluginPrivate::Station3SensorCallback(ConstLogicalCameraImagePtr &_msg)
    {
        assembly_station_images_.insert_or_assign(3, *_msg);
    }
    //==============================================================================
    void TaskManagerPluginPrivate::Station4SensorCallback(ConstLogicalCameraImagePtr &_msg)
    {
        assembly_station_images_.insert_or_assign(4, *_msg);
    }

    //==============================================================================
    const ariac_msgs::msg::KittingTask
    TaskManagerPlugin::BuildKittingTaskMsg(std::shared_ptr<ariac_common::KittingTask> _task)
    {
        auto message = ariac_msgs::msg::KittingTask();
        message.agv_number = _task->GetAgvNumber();
        message.tray_id = _task->GetTrayId();
        message.destination = _task->GetDestination();

        for (auto &part : _task->GetProducts())
        {
            auto part_msg = ariac_msgs::msg::KittingPart();
            part_msg.part.color = part.GetPart().GetColor();
            part_msg.part.type = part.GetPart().GetType();
            part_msg.quadrant = part.GetQuadrant();
            message.parts.push_back(part_msg);
        }
        return message;
    }

    //==============================================================================
    const ariac_msgs::msg::AssemblyTask TaskManagerPlugin::BuildAssemblyTaskMsg(std::shared_ptr<ariac_common::AssemblyTask> _task)
    {
        auto message = ariac_msgs::msg::AssemblyTask();
        // station
        message.station = _task->GetStation();
        // agvs
        for (auto &agv : _task->GetAgvNumbers())
        {
            message.agv_numbers.push_back(agv);
        }

        // products
        for (auto &part : _task->GetProducts())
        {
            auto part_msg = ariac_msgs::msg::AssemblyPart();
            part_msg.part.color = part.GetPart().GetColor();
            part_msg.part.type = part.GetPart().GetType();

            auto assembly_pose_msg = geometry_msgs::msg::PoseStamped();
            assembly_pose_msg.pose.position.x = part.GetPartPose().Pos().X();
            assembly_pose_msg.pose.position.y = part.GetPartPose().Pos().Y();
            assembly_pose_msg.pose.position.z = part.GetPartPose().Pos().Z();
            assembly_pose_msg.pose.orientation.x = part.GetPartPose().Rot().X();
            assembly_pose_msg.pose.orientation.y = part.GetPartPose().Rot().Y();
            assembly_pose_msg.pose.orientation.z = part.GetPartPose().Rot().Z();
            assembly_pose_msg.pose.orientation.w = part.GetPartPose().Rot().W();
            part_msg.assembled_pose = assembly_pose_msg;

            auto install_direction_msg = geometry_msgs::msg::Vector3();
            install_direction_msg.x = part.GetPartDirection().X();
            install_direction_msg.y = part.GetPartDirection().Y();
            install_direction_msg.z = part.GetPartDirection().Z();
            part_msg.install_direction = install_direction_msg;

            message.parts.push_back(part_msg);
        }

        return message;
    }

    //==============================================================================
    const ariac_msgs::msg::CombinedTask TaskManagerPlugin::BuildCombinedTaskMsg(std::shared_ptr<ariac_common::CombinedTask> _task)
    {
        auto message = ariac_msgs::msg::CombinedTask();
        // station
        message.station = _task->GetStation();

        // products
        for (auto &part : _task->GetProducts())
        {
            auto part_msg = ariac_msgs::msg::AssemblyPart();
            part_msg.part.color = part.GetPart().GetColor();
            part_msg.part.type = part.GetPart().GetType();

            auto assembly_pose_msg = geometry_msgs::msg::PoseStamped();
            assembly_pose_msg.pose.position.x = part.GetPartPose().Pos().X();
            assembly_pose_msg.pose.position.y = part.GetPartPose().Pos().Y();
            assembly_pose_msg.pose.position.z = part.GetPartPose().Pos().Z();
            assembly_pose_msg.pose.orientation.x = part.GetPartPose().Rot().X();
            assembly_pose_msg.pose.orientation.y = part.GetPartPose().Rot().Y();
            assembly_pose_msg.pose.orientation.z = part.GetPartPose().Rot().Z();
            assembly_pose_msg.pose.orientation.w = part.GetPartPose().Rot().W();
            part_msg.assembled_pose = assembly_pose_msg;

            auto install_direction_msg = geometry_msgs::msg::Vector3();
            install_direction_msg.x = part.GetPartDirection().X();
            install_direction_msg.y = part.GetPartDirection().Y();
            install_direction_msg.z = part.GetPartDirection().Z();
            part_msg.install_direction = install_direction_msg;

            message.parts.push_back(part_msg);
        }

        return message;
    }

    //==============================================================================
    const ariac_msgs::msg::Order
    TaskManagerPlugin::BuildOrderMsg(std::shared_ptr<ariac_common::Order> _order)
    {
        auto order_message = ariac_msgs::msg::Order();
        order_message.id = _order->GetId();
        order_message.type = _order->GetType();
        order_message.priority = _order->IsPriority();
        if (_order->GetKittingTask())
        {
            order_message.kitting_task = BuildKittingTaskMsg(_order->GetKittingTask());
        }
        else if (_order->GetAssemblyTask())
        {
            // RCLCPP_INFO_STREAM_ONCE(impl_->ros_node_->get_logger(), "GetAssemblyTask");
            order_message.assembly_task = BuildAssemblyTaskMsg(_order->GetAssemblyTask());
        }
        else if (_order->GetCombinedTask())
        {
            // RCLCPP_INFO_STREAM_ONCE(impl_->ros_node_->get_logger(), "GetCombinedTask");
            order_message.combined_task = BuildCombinedTaskMsg(_order->GetCombinedTask());
        }
        return order_message;
    }

    //==============================================================================
    void TaskManagerPlugin::ProcessTemporalOrders()
    {
        for (const auto &order : impl_->time_based_orders_)
        {
            if (!order->IsAnnounced())
            {
                if (impl_->elapsed_time_ >= order->GetAnnouncementTime() && !order->IsAnnounced())
                {
                    auto order_message = BuildOrderMsg(order);
                    RCLCPP_INFO_STREAM(impl_->ros_node_->get_logger(), "Announcing order");
                    RCLCPP_INFO_STREAM(impl_->ros_node_->get_logger(), "\n" << *order);

                    impl_->order_pub_->publish(order_message);
                    impl_->announced_orders_.push_back(order_message.id);
                    order->SetAnnouncedTime(impl_->elapsed_time_);
                    order->SetIsAnnounced();
                    impl_->total_orders_--;
                }
            }
        }
    }

    //==============================================================================
    void TaskManagerPlugin::ProcessOnPartPlacementOrders()
    {
        for (const auto &order : impl_->on_part_placement_orders_)
        {
            // RCLCPP_INFO_STREAM(impl_->ros_node_->get_logger(), "Order: " << order->GetId());

            if (!order->IsAnnounced())
            {
                auto part_condition = order->GetPart();
                auto agv_condition = order->GetAgv();

                auto parts = impl_->agv_parts_.find(agv_condition)->second;

                if (parts.empty())
                {
                    return;
                }

                for (const auto &agv_part : parts)
                {
                    // RCLCPP_INFO_STREAM(impl_->ros_node_->get_logger(), "Part type: " << agv_part.GetType());
                    // RCLCPP_INFO_STREAM(impl_->ros_node_->get_logger(), "Part color: " << agv_part.GetColor());
                    // RCLCPP_INFO_STREAM(impl_->ros_node_->get_logger(), "Part condition type: " << part_condition->GetType());
                    // RCLCPP_INFO_STREAM(impl_->ros_node_->get_logger(), "Part condition color: " << part_condition->GetColor());
                    if (agv_part.GetType() == part_condition->GetType() && agv_part.GetColor() == part_condition->GetColor())
                    {
                        auto order_message = BuildOrderMsg(order);

                        RCLCPP_INFO_STREAM(impl_->ros_node_->get_logger(), "Announcing order");
                        RCLCPP_INFO_STREAM(impl_->ros_node_->get_logger(), "\n" << *order);
                        impl_->order_pub_->publish(order_message);
                        impl_->announced_orders_.push_back(order_message.id);
                        order->SetAnnouncedTime(impl_->elapsed_time_);
                        order->SetIsAnnounced();
                        impl_->total_orders_--;
                    }
                }
            }
        }
    }

    //==============================================================================
    void TaskManagerPlugin::ProcessOnSubmissionOrders()
    {
        for (const auto &order : impl_->on_order_submission_orders_)
        {
            if (!order->IsAnnounced())
            {
                // Get the id of the order which will trigger the announcement of order_ins
                auto trigger_order = order->GetOrderId();

                // parse the list of submitted orders to see if the trigger order has been submitted
                if (std::find(impl_->submitted_orders_.begin(), impl_->submitted_orders_.end(), trigger_order) != impl_->submitted_orders_.end())
                {

                    auto order_message = BuildOrderMsg(order);
                    RCLCPP_INFO_STREAM(impl_->ros_node_->get_logger(), "Announcing order");
                    RCLCPP_INFO_STREAM(impl_->ros_node_->get_logger(), "\n" << *order);
                    impl_->order_pub_->publish(order_message);
                    order->SetAnnouncedTime(impl_->elapsed_time_);
                    order->SetIsAnnounced();
                    impl_->total_orders_--;
                }
            }
        }
    }

    //==============================================================================
    void TaskManagerPlugin::ProcessOrdersToAnnounce()
    {
        if (!impl_->time_based_orders_.empty())
        {
            ProcessTemporalOrders();
        }
        // TODO: Implement on part placement and on submission orders
        if (!impl_->on_part_placement_orders_.empty())
        {
            ProcessOnPartPlacementOrders();
        }
        if (!impl_->on_order_submission_orders_.empty())
        {
            ProcessOnSubmissionOrders();
        }
    }

    //==============================================================================
    void TaskManagerPlugin::ProcessInProgressRobotMalfunctions()
    {
        for (const auto &robot_malfunction : impl_->in_progress_robot_malfunctions_)
        {
            if (robot_malfunction->HasStarted() && !robot_malfunction->HasCompleted())
            {
                auto duration = robot_malfunction->GetDuration();
                auto start_time = robot_malfunction->GetStartTime();

                if (impl_->elapsed_time_ >= start_time + duration)
                {
                    RCLCPP_INFO_STREAM(impl_->ros_node_->get_logger(), "Stopping robot malfunction challenge at time: " << impl_->elapsed_time_);
                    robot_malfunction->SetCompleted();

                    impl_->ceiling_robot_health_ = true;
                    impl_->floor_robot_health_ = true;
                    // remove the challenge from the list
                    impl_->in_progress_robot_malfunctions_.erase(std::remove(impl_->in_progress_robot_malfunctions_.begin(),
                                                                             impl_->in_progress_robot_malfunctions_.end(), robot_malfunction),
                                                                 impl_->in_progress_robot_malfunctions_.end());
                }
            }
        }
    }

    //==============================================================================
    void TaskManagerPlugin::ProcessInProgressSensorBlackouts()
    {
        for (const auto &sensor_blackout : impl_->in_progress_sensor_blackouts_)
        {
            if (sensor_blackout->HasStarted() && !sensor_blackout->HasCompleted())
            {
                auto duration = sensor_blackout->GetDuration();
                auto start_time = sensor_blackout->GetStartTime();
                // RCLCPP_INFO_STREAM(impl_->ros_node_->get_logger(), "Sensor blackout challenge in progress");
                // RCLCPP_INFO_STREAM(impl_->ros_node_->get_logger(), start_time);
                // RCLCPP_INFO_STREAM(impl_->ros_node_->get_logger(), duration);

                if (impl_->elapsed_time_ >= start_time + duration)
                {
                    RCLCPP_INFO_STREAM(impl_->ros_node_->get_logger(), "Stopping sensor blackout challenge at time: " << impl_->elapsed_time_);
                    sensor_blackout->SetCompleted();
                    impl_->break_beam_sensor_health_ = true;
                    impl_->proximity_sensor_health_ = true;
                    impl_->laser_profiler_sensor_health_ = true;
                    impl_->lidar_sensor_health_ = true;
                    impl_->camera_sensor_health_ = true;
                    impl_->logical_camera_sensor_health_ = true;
                    // remove the sensor blackout from the list
                    impl_->in_progress_sensor_blackouts_.erase(std::remove(impl_->in_progress_sensor_blackouts_.begin(),
                                                                           impl_->in_progress_sensor_blackouts_.end(), sensor_blackout),
                                                               impl_->in_progress_sensor_blackouts_.end());
                }
            }
        }
    }

    //==============================================================================
    void TaskManagerPlugin::ProcessTemporalRobotMalfunctions()
    {
        for (const auto &robot_malfunction : impl_->time_based_robot_malfunctions_)
        {
            if (impl_->elapsed_time_ >= robot_malfunction->GetTriggerTime() && !robot_malfunction->HasStarted())
            {
                RCLCPP_INFO_STREAM(impl_->ros_node_->get_logger(), "Starting robot malfunction challenge at time: " << impl_->elapsed_time_);
                // Duration of the challenge
                auto duration = robot_malfunction->GetDuration();
                // Get the list of robots to disable for the challenge
                auto robots_to_disable = robot_malfunction->GetRobotsToDisable();
                // Disable the robots according to the list
                SetRobotsHealth(robots_to_disable);
                robot_malfunction->SetStartTime(impl_->elapsed_time_);
                robot_malfunction->SetStarted();
                impl_->in_progress_robot_malfunctions_.push_back(robot_malfunction);
            }
        }
    }

    //==============================================================================
    void
    TaskManagerPlugin::ProcessOnSubmissionRobotMalfunctions()
    {
        for (const auto &rm : impl_->on_submission_robot_malfunctions_)
        {
            // Get the id of the order which will trigger the start of this challenge
            auto trigger_order = rm->GetTriggerOrderId();

            // parse the list of submitted orders to see if the trigger order has been submitted
            if (std::find(impl_->submitted_orders_.begin(), impl_->submitted_orders_.end(), trigger_order) != impl_->submitted_orders_.end())
            {
                if (!rm->HasStarted())
                {
                    RCLCPP_INFO_STREAM(impl_->ros_node_->get_logger(), "Starting robot malfunction challenge at time: " << impl_->elapsed_time_);
                    auto duration = rm->GetDuration();
                    auto robots_to_disable = rm->GetRobotsToDisable();
                    SetRobotsHealth(robots_to_disable);
                    rm->SetStartTime(impl_->elapsed_time_);
                    rm->SetStarted();
                    impl_->in_progress_robot_malfunctions_.push_back(rm);
                }
            }
        }
    }

    //==============================================================================
    void
    TaskManagerPlugin::SetSensorsHealth(const std::vector<std::string> &_sensors_to_disable)
    {
        for (const auto &sensor : _sensors_to_disable)
        {
            if (sensor == "break_beam")
                impl_->break_beam_sensor_health_ = false;
            if (sensor == "proximity")
                impl_->proximity_sensor_health_ = false;
            if (sensor == "laser_profiler")
                impl_->laser_profiler_sensor_health_ = false;
            if (sensor == "lidar")
                impl_->lidar_sensor_health_ = false;
            if (sensor == "camera")
                impl_->camera_sensor_health_ = false;
            if (sensor == "logical_camera")
                impl_->logical_camera_sensor_health_ = false;
        }
    }

    //==============================================================================
    void
    TaskManagerPlugin::SetRobotsHealth(const std::vector<std::string> &_robots_to_disable)
    {
        for (const auto &robot : _robots_to_disable)
        {
            if (robot == "ceiling_robot")
                impl_->ceiling_robot_health_ = false;
            if (robot == "floor_robot")
                impl_->floor_robot_health_ = false;
        }
    }

    //==============================================================================
    void
    TaskManagerPlugin::ProcessOnSubmissionSensorBlackouts()
    {
        for (const auto &sb : impl_->on_submission_sensor_blackouts_)
        {
            // Get the id of the order which will trigger the announcement of order_ins
            auto trigger_order = sb->GetTriggerOrderId();

            // parse the list of submitted orders to see if the trigger order has been submitted
            if (std::find(impl_->submitted_orders_.begin(), impl_->submitted_orders_.end(), trigger_order) != impl_->submitted_orders_.end())
            {
                if (!sb->HasStarted())
                {
                    RCLCPP_INFO_STREAM(impl_->ros_node_->get_logger(), "Starting sensor blackout challenge at time: " << impl_->elapsed_time_);
                    auto duration = sb->GetDuration();
                    auto sensors_to_disable = sb->GetSensorsToDisable();
                    SetSensorsHealth(sensors_to_disable);
                    sb->SetStartTime(impl_->elapsed_time_);
                    sb->SetStarted();
                    impl_->in_progress_sensor_blackouts_.push_back(sb);
                }
            }
        }
    }

    //==============================================================================
    void
    TaskManagerPlugin::ProcessTemporalSensorBlackouts()
    {
        for (const auto &sensor_blackout : impl_->time_based_sensor_blackouts_)
        {
            if (impl_->elapsed_time_ >= sensor_blackout->GetTriggerTime() && !sensor_blackout->HasStarted())
            {
                RCLCPP_INFO_STREAM(impl_->ros_node_->get_logger(), "Starting sensor blackout challenge at time: " << impl_->elapsed_time_);
                auto sensors_to_disable = sensor_blackout->GetSensorsToDisable();

                SetSensorsHealth(sensors_to_disable);
                sensor_blackout->SetStartTime(impl_->elapsed_time_);
                sensor_blackout->SetStarted();
                impl_->in_progress_sensor_blackouts_.push_back(sensor_blackout);
            }
        }
    }

    //==============================================================================
    void TaskManagerPlugin::ProcessTemporalHumanChallenge()
    {
        // RCLCPP_INFO_STREAM(impl_->ros_node_->get_logger(), "Elapsed time: " << impl_->elapsed_time_);
        if (impl_->elapsed_time_ >= impl_->time_based_human_challenge_->GetTriggerTime())
        {
            RCLCPP_INFO_STREAM(impl_->ros_node_->get_logger(), "Starting human challenge");
            std_msgs::msg::Bool msg;
            msg.data = true;
            impl_->start_human_pub_->publish(msg);
            impl_->time_based_human_challenge_->SetStartTime(impl_->elapsed_time_);
            impl_->time_based_human_challenge_->SetStarted();
        }
    }

    //==============================================================================
    void TaskManagerPlugin::ProcessChallengesToAnnounce()
    {
        // time based human challenge
        if (impl_->time_based_human_challenge_)
        {
            if (!impl_->time_based_human_challenge_->HasStarted())
            {
                ProcessTemporalHumanChallenge();
            }
        }
        // on part placement human challenge
        if (impl_->on_part_placement_human_challenge_)
        {
            if (!impl_->on_part_placement_human_challenge_->HasStarted())
            {
                ProcessOnPartPlacementHumanChallenge();
            }
        }
        // on order submission human challenge
        if (impl_->on_order_submission_human_challenge_)
        {
            if (!impl_->on_order_submission_human_challenge_->HasStarted())
            {
                ProcessOnOrderSubmissionHumanChallenge();
            }
        }

        if (!impl_->time_based_sensor_blackouts_.empty())
            ProcessTemporalSensorBlackouts();
        if (!impl_->on_part_placement_sensor_blackouts_.empty())
            ProcessOnPartPlacementSensorBlackouts();
        if (!impl_->on_submission_sensor_blackouts_.empty())
            ProcessOnSubmissionSensorBlackouts();
        if (!impl_->time_based_robot_malfunctions_.empty())
            ProcessTemporalRobotMalfunctions();
        if (!impl_->on_part_placement_robot_malfunctions_.empty())
            ProcessOnPartPlacementRobotMalfunctions();
        if (!impl_->on_submission_robot_malfunctions_.empty())
            ProcessOnSubmissionRobotMalfunctions();
    }

    //==============================================================================
    void
    TaskManagerPlugin::ProcessOnOrderSubmissionHumanChallenge()
    {

        // Get the id of the order which will trigger the start of this challenge
        auto trigger_order = impl_->on_order_submission_human_challenge_->GetTriggerOrderId();

        // parse the list of submitted orders to see if the trigger order has been submitted
        if (std::find(impl_->submitted_orders_.begin(), impl_->submitted_orders_.end(), trigger_order) != impl_->submitted_orders_.end())
        {
            RCLCPP_INFO_STREAM(impl_->ros_node_->get_logger(), "Starting human challenge");
            std_msgs::msg::Bool msg;
            msg.data = true;
            impl_->start_human_pub_->publish(msg);
            impl_->on_order_submission_human_challenge_->SetStartTime(impl_->elapsed_time_);
            impl_->on_order_submission_human_challenge_->SetStarted();
        }
    }

    //==============================================================================
    void TaskManagerPlugin::ProcessOnPartPlacementHumanChallenge()
    {
        auto agv_condition = impl_->on_part_placement_human_challenge_->GetAgv();
        auto part_condition = impl_->on_part_placement_human_challenge_->GetPart();

        // Get the list of parts on the agv
        auto agv_parts = impl_->agv_parts_.find(agv_condition)->second;

        // if the agv is empty, skip it
        if (agv_parts.empty())
            return;

        for (const auto &agv_part : agv_parts)
        {
            if (agv_part.GetType() == part_condition->GetType() && agv_part.GetColor() == part_condition->GetColor())
            {
                RCLCPP_INFO_STREAM(impl_->ros_node_->get_logger(), "Starting human challenge");
                std_msgs::msg::Bool msg;
                msg.data = true;
                impl_->start_human_pub_->publish(msg);
                impl_->on_part_placement_human_challenge_->SetStartTime(impl_->elapsed_time_);
                impl_->on_part_placement_human_challenge_->SetStarted();
            }
        }
    }

    //==============================================================================
    void TaskManagerPlugin::ProcessOnPartPlacementRobotMalfunctions()
    {
        for (const auto &part_placement_challenge : impl_->on_part_placement_robot_malfunctions_)
        {
            // if this challenge is in progress, skip it
            if (part_placement_challenge->HasStarted())
                return;

            auto agv_condition = part_placement_challenge->GetAgv();
            auto part_condition = part_placement_challenge->GetPart();

            // Get the list of parts on the agv
            auto agv_parts = impl_->agv_parts_.find(agv_condition)->second;

            // if the agv is empty, skip it
            if (agv_parts.empty())
                return;

            for (const auto &agv_part : agv_parts)
            {
                if (agv_part.GetType() == part_condition->GetType() && agv_part.GetColor() == part_condition->GetColor())
                {
                    RCLCPP_INFO_STREAM(impl_->ros_node_->get_logger(), "Starting robot malfunction challenge at time: " << impl_->elapsed_time_);
                    auto robots_to_disable = part_placement_challenge->GetRobotsToDisable();
                    // Disable the robots according to the list
                    SetRobotsHealth(robots_to_disable);
                    part_placement_challenge->SetStartTime(impl_->elapsed_time_);
                    part_placement_challenge->SetStarted();
                    impl_->in_progress_robot_malfunctions_.push_back(part_placement_challenge);
                }
            }
        }
    }

    //==============================================================================
    void TaskManagerPlugin::ProcessOnPartPlacementSensorBlackouts()
    {
        for (const auto &part_placement_challenge : impl_->on_part_placement_sensor_blackouts_)
        {
            // if this challenge is in progress, skip it
            if (part_placement_challenge->HasStarted())
                return;

            auto agv_condition = part_placement_challenge->GetAgv();
            auto part_condition = part_placement_challenge->GetPart();

            // Get the list of parts on the agv
            auto agv_parts = impl_->agv_parts_.find(agv_condition)->second;

            // if the agv is empty, skip it
            if (agv_parts.empty())
                return;

            for (const auto &agv_part : agv_parts)
            {
                if (agv_part.GetType() == part_condition->GetType() && agv_part.GetColor() == part_condition->GetColor())
                {
                    RCLCPP_INFO_STREAM(impl_->ros_node_->get_logger(), "Starting sensor blackout challenge at time: " << impl_->elapsed_time_);
                    auto sensors_to_disable = part_placement_challenge->GetSensorsToDisable();

                    SetSensorsHealth(sensors_to_disable);
                    part_placement_challenge->SetStartTime(impl_->elapsed_time_);
                    part_placement_challenge->SetStarted();
                    impl_->in_progress_sensor_blackouts_.push_back(part_placement_challenge);
                }
            }
        }
    }

    //==============================================================================
    void TaskManagerPlugin::PublishCompetitionState(unsigned int _state)
    {
        auto state_message = ariac_msgs::msg::CompetitionState();
        state_message.competition_state = _state;
        impl_->competition_state_pub_->publish(state_message);
    }

    //==============================================================================
    void TaskManagerPlugin::OnUpdate()
    {
        std::lock_guard<std::mutex> lock(impl_->lock_);
        auto current_sim_time = impl_->world_->SimTime();

        // if the competition has ended, disable all sensors and robots
        if (impl_->current_state_ == ariac_msgs::msg::CompetitionState::ENDED)
        {
            // impl_->submit_order_service_->~Service();
            DisableAllSensors();
            DisableAllRobots();
        }

        if (impl_->total_orders_ == 0 && impl_->current_state_ == ariac_msgs::msg::CompetitionState::STARTED)
        {
            RCLCPP_INFO_STREAM_ONCE(impl_->ros_node_->get_logger(), "All orders have been announced.");
            impl_->current_state_ = ariac_msgs::msg::CompetitionState::ORDER_ANNOUNCEMENTS_DONE;
        }

        // publish the competition state
        rclcpp::Time now = impl_->ros_node_->get_clock()->now();
        if (impl_->first_publish_)
        {
            PublishCompetitionState(impl_->current_state_);
        }
        else if (now - impl_->last_publish_time_ >= rclcpp::Duration(0, impl_->update_ns_))
        {
            PublishCompetitionState(impl_->current_state_);
        }

        // Delay advertising the competition start service to avoid a crash.
        // Sometimes if the competition is started before the world is fully loaded, it causes a crash.
        if (!impl_->start_competition_service_ && current_sim_time.Double() >= 5.0)
        {
            // Create the start competition service
            // Now competitors can call this service to start the competition
            impl_->start_competition_service_ = impl_->ros_node_->create_service<std_srvs::srv::Trigger>("/ariac/start_competition", std::bind(&TaskManagerPlugin::StartCompetitionServiceCallback, this, std::placeholders::_1, std::placeholders::_2));

            RCLCPP_INFO_STREAM(impl_->ros_node_->get_logger(), "You can now start the competition!");
            impl_->current_state_ = ariac_msgs::msg::CompetitionState::READY;

            // Create the end competition service
            // Now competitors can call this service to end the competition
            impl_->end_competition_service_ = impl_->ros_node_->create_service<std_srvs::srv::Trigger>(
                "/ariac/end_competition",
                std::bind(&TaskManagerPlugin::EndCompetitionServiceCallback,
                          this, std::placeholders::_1, std::placeholders::_2));

            impl_->human_safe_zone_penalty_service_ = impl_->ros_node_->create_service<std_srvs::srv::Trigger>(
                "/ariac/set_human_safe_zone_penalty",
                std::bind(&TaskManagerPlugin::SafeZonePenaltyServiceCallback,
                          this, std::placeholders::_1, std::placeholders::_2));
        }

        if ((current_sim_time - impl_->last_sim_time_).Double() >= 1.0)
        {
            impl_->last_sim_time_ = current_sim_time;
        }

        // End the competition if:
        // The time limit was set to a positive value
        // The elapsed time since the start of the competition is greater than the time limit
        // The current state is ORDER_ANNOUNCEMENTS_DONE
        if (impl_->time_limit_ >= 0 &&
            (current_sim_time - impl_->start_competition_time_).Double() > impl_->time_limit_ && impl_->current_state_ == ariac_msgs::msg::CompetitionState::ORDER_ANNOUNCEMENTS_DONE)
        {
            RCLCPP_INFO_STREAM(impl_->ros_node_->get_logger(), "Time limit reached. Ending competition.");
            this->impl_->current_state_ = ariac_msgs::msg::CompetitionState::ENDED;
            // deactive robots, sensors, and submission service (de-register)
            // destructor for the service
        }

        // current state was set to STARTED in start competition service callback
        if (impl_->current_state_ == ariac_msgs::msg::CompetitionState::STARTED && !impl_->competition_time_set_)
        {
            impl_->start_competition_time_ = current_sim_time;
            impl_->competition_time_set_ = true;

            impl_->submit_order_service_ = impl_->ros_node_->create_service<ariac_msgs::srv::SubmitOrder>(
                "/ariac/submit_order",
                std::bind(&TaskManagerPlugin::SubmitOrderServiceCallback,
                          this, std::placeholders::_1, std::placeholders::_2));

            impl_->quality_check_service_ = impl_->ros_node_->create_service<ariac_msgs::srv::PerformQualityCheck>(
                "/ariac/perform_quality_check",
                std::bind(&TaskManagerPluginPrivate::PerformQualityCheck,
                          this->impl_.get(),
                          std::placeholders::_1,
                          std::placeholders::_2));

            impl_->pre_assembly_poses_service_ = impl_->ros_node_->create_service<ariac_msgs::srv::GetPreAssemblyPoses>(
                "/ariac/get_pre_assembly_poses",
                std::bind(&TaskManagerPluginPrivate::GetPreAssemblyPoses,
                          this->impl_.get(),
                          std::placeholders::_1,
                          std::placeholders::_2));
        }

        // If the competition has started, do the main work
        if (impl_->current_state_ == ariac_msgs::msg::CompetitionState::STARTED ||
            impl_->current_state_ == ariac_msgs::msg::CompetitionState::ORDER_ANNOUNCEMENTS_DONE)
        {
            // Update the elapsed time
            impl_->elapsed_time_ = (current_sim_time - impl_->start_competition_time_).Double();

            // Store images received by the sensor attached to each AGV
            impl_->StoreAGVParts(1, impl_->agv_tray_images_[1]);
            impl_->StoreAGVParts(2, impl_->agv_tray_images_[2]);
            impl_->StoreAGVParts(3, impl_->agv_tray_images_[3]);
            impl_->StoreAGVParts(4, impl_->agv_tray_images_[4]);

            // Store images received by the sensor attached to each assembly station
            impl_->StoreStationParts(1, impl_->assembly_station_images_[1]);
            impl_->StoreStationParts(2, impl_->assembly_station_images_[2]);
            impl_->StoreStationParts(3, impl_->assembly_station_images_[3]);
            impl_->StoreStationParts(4, impl_->assembly_station_images_[4]);

            ProcessEndSafeZonePenalty();
            ProcessOrdersToAnnounce();
            ProcessChallengesToAnnounce();
            ProcessInProgressSensorBlackouts();
            ProcessInProgressRobotMalfunctions();

            if (impl_->first_publish_)
            {
                UpdateSensorsHealth();
                UpdateRobotsHealth();
            }
            else if (now - impl_->last_publish_time_ >= rclcpp::Duration(0, impl_->update_ns_))
            {
                UpdateSensorsHealth();
                UpdateRobotsHealth();
            }
        }

        if (impl_->first_publish_)
        {
            impl_->last_publish_time_ = now;
            impl_->first_publish_ = false;
        }
        else if (now - impl_->last_publish_time_ >= rclcpp::Duration(0, impl_->update_ns_))
        {
            impl_->last_publish_time_ = now;
        }

        impl_->last_on_update_time_ = current_sim_time;
    }

    //==============================================================================
    void TaskManagerPlugin::DisableAllSensors()
    {
        RCLCPP_WARN_STREAM_ONCE(impl_->ros_node_->get_logger(), "Competition ended: All sensors are now disabled.");
        auto sensor_message = ariac_msgs::msg::Sensors();
        sensor_message.break_beam = false;
        sensor_message.proximity = false;
        sensor_message.laser_profiler = false;
        sensor_message.lidar = false;
        sensor_message.camera = false;
        sensor_message.logical_camera = false;

        impl_->sensor_health_pub_->publish(sensor_message);
    }

    //==============================================================================
    void TaskManagerPlugin::DisableAllRobots()
    {
        RCLCPP_WARN_STREAM_ONCE(impl_->ros_node_->get_logger(), "Competition ended: All robots are now disabled.");
        auto robots_message = ariac_msgs::msg::Robots();
        robots_message.ceiling_robot = false;
        robots_message.floor_robot = false;

        impl_->robot_health_pub_->publish(robots_message);
    }

    //==============================================================================
    void TaskManagerPlugin::OnAGV1StatusCallback(const ariac_msgs::msg::AGVStatus::SharedPtr _msg)
    {

        impl_->agv1_location_ = _msg->location;
    }

    void TaskManagerPlugin::OnAGV2StatusCallback(const ariac_msgs::msg::AGVStatus::SharedPtr _msg)
    {
        impl_->agv2_location_ = _msg->location;
    }

    void TaskManagerPlugin::OnAGV3StatusCallback(const ariac_msgs::msg::AGVStatus::SharedPtr _msg)
    {
        impl_->agv3_location_ = _msg->location;
    }

    void TaskManagerPlugin::OnAGV4StatusCallback(const ariac_msgs::msg::AGVStatus::SharedPtr _msg)
    {
        impl_->agv4_location_ = _msg->location;
    }

    //==============================================================================
    void TaskManagerPlugin::OnTrialCallback(const ariac_msgs::msg::Trial::SharedPtr _msg)
    {
        std::lock_guard<std::mutex> scoped_lock(impl_->lock_);
        // RCLCPP_FATAL_STREAM(impl_->ros_node_->get_logger(), "------Time limit: " << _msg->time_limit);
        // RCLCPP_FATAL_STREAM(impl_->ros_node_->get_logger(), "------Trial name: " << _msg->trial_name);
        impl_->time_limit_ = _msg->time_limit;
        impl_->trial_name_ = _msg->trial_name;

        // Store orders to be processed later
        std::vector<std::shared_ptr<ariac_msgs::msg::OrderCondition>>
            order_conditions;
        for (auto order_condition : _msg->order_conditions)
        {
            order_conditions.push_back(std::make_shared<ariac_msgs::msg::OrderCondition>(order_condition));
        }
        StoreOrders(order_conditions);
        // RCLCPP_INFO_STREAM(impl_->ros_node_->get_logger(), "Number of orders in the trial: " << impl_->all_orders_.size());

        // This attribute is used to keep track of the total number of orders
        impl_->total_orders_ = impl_->time_based_orders_.size() + impl_->on_part_placement_orders_.size() + impl_->on_order_submission_orders_.size();
        // RCLCPP_INFO_STREAM(impl_->ros_node_->get_logger(), "Number of temporal orders: " << impl_->time_based_orders_.size());
        // RCLCPP_INFO_STREAM(impl_->ros_node_->get_logger(), "Number of part place orders: " << impl_->on_part_placement_orders_.size());
        // RCLCPP_INFO_STREAM(impl_->ros_node_->get_logger(), "Number of submission orders: " << impl_->on_order_submission_orders_.size());

        // Store challenges to be processed later
        if (_msg->challenges.size() > 0)
        {
            std::vector<std::shared_ptr<ariac_msgs::msg::Challenge>> challenges;
            for (auto challenge : _msg->challenges)
            {
                challenges.push_back(std::make_shared<ariac_msgs::msg::Challenge>(challenge));
            }
            if (challenges.size() > 0)
            {
                StoreChallenges(challenges);
            }
        }
    }

    //==============================================================================
    std::shared_ptr<ariac_common::KittingTask> TaskManagerPlugin::BuildKittingTask(const ariac_msgs::msg::KittingTask &_kitting_task)
    {
        auto agv_number = _kitting_task.agv_number;
        auto tray_id = _kitting_task.tray_id;
        auto destination = _kitting_task.destination;
        std::vector<ariac_common::KittingPart> kitting_parts;

        for (auto product : _kitting_task.parts)
        {
            auto quadrant = product.quadrant;
            auto part_type = product.part.type;
            auto part_color = product.part.color;
            auto part = ariac_common::Part(part_color, part_type);
            kitting_parts.emplace_back(ariac_common::KittingPart(quadrant, part));
        }
        return std::make_shared<ariac_common::KittingTask>(agv_number, tray_id, destination, kitting_parts);
    }

    //==============================================================================
    std::shared_ptr<ariac_common::AssemblyTask> TaskManagerPlugin::BuildAssemblyTask(const ariac_msgs::msg::AssemblyTask &_assembly_task)
    {
        std::vector<unsigned int> agv_numbers = {};
        for (auto agv_number : _assembly_task.agv_numbers)
        {
            agv_numbers.push_back(agv_number);
        }
        auto station = _assembly_task.station;
        std::vector<ariac_common::AssemblyPart> assembly_parts;

        for (auto product : _assembly_task.parts)
        {
            auto part_type = product.part.type;
            auto part_color = product.part.color;
            auto part = ariac_common::Part(part_color, part_type);

            ignition::math::Pose3d assembled_pose;
            assembled_pose.Pos().X() = product.assembled_pose.pose.position.x;
            assembled_pose.Pos().Y() = product.assembled_pose.pose.position.y;
            assembled_pose.Pos().Z() = product.assembled_pose.pose.position.z;
            assembled_pose.Rot().X() = product.assembled_pose.pose.orientation.x;
            assembled_pose.Rot().Y() = product.assembled_pose.pose.orientation.y;
            assembled_pose.Rot().Z() = product.assembled_pose.pose.orientation.z;
            assembled_pose.Rot().W() = product.assembled_pose.pose.orientation.w;

            ignition::math::Vector3<double> part_direction;
            part_direction.X() = product.install_direction.x;
            part_direction.Y() = product.install_direction.y;
            part_direction.Z() = product.install_direction.z;

            assembly_parts.emplace_back(ariac_common::AssemblyPart(part, assembled_pose, part_direction));
        }
        return std::make_shared<ariac_common::AssemblyTask>(agv_numbers, station, assembly_parts);
    }

    //==============================================================================
    std::shared_ptr<ariac_common::CombinedTask> TaskManagerPlugin::BuildCombinedTask(const ariac_msgs::msg::CombinedTask &_combined_task)
    {
        auto station = _combined_task.station;
        std::vector<ariac_common::AssemblyPart> assembly_parts;

        for (auto product : _combined_task.parts)
        {
            auto part_type = product.part.type;
            auto part_color = product.part.color;
            auto part = ariac_common::Part(part_color, part_type);

            ignition::math::Pose3d assembled_pose;
            assembled_pose.Pos().X() = product.assembled_pose.pose.position.x;
            assembled_pose.Pos().Y() = product.assembled_pose.pose.position.y;
            assembled_pose.Pos().Z() = product.assembled_pose.pose.position.z;
            assembled_pose.Rot().X() = product.assembled_pose.pose.orientation.x;
            assembled_pose.Rot().Y() = product.assembled_pose.pose.orientation.y;
            assembled_pose.Rot().Z() = product.assembled_pose.pose.orientation.z;
            assembled_pose.Rot().W() = product.assembled_pose.pose.orientation.w;

            ignition::math::Vector3<double> part_direction;
            part_direction.X() = product.install_direction.x;
            part_direction.Y() = product.install_direction.y;
            part_direction.Z() = product.install_direction.z;

            assembly_parts.emplace_back(ariac_common::AssemblyPart(part, assembled_pose, part_direction));
        }
        return std::make_shared<ariac_common::CombinedTask>(station, assembly_parts);
    }

    //==============================================================================
    void TaskManagerPlugin::StoreOrders(const std::vector<std::shared_ptr<ariac_msgs::msg::OrderCondition>> &orders)
    {
        for (auto order : orders)
        {
            auto order_id = order->id;
            // Keep track of order ids
            impl_->trial_orders_.push_back(order_id);

            auto order_type = order->type;
            auto order_priority = order->priority;

            // Pointers to the tasks
            std::shared_ptr<ariac_common::KittingTask> kitting_task = nullptr;
            std::shared_ptr<ariac_common::AssemblyTask> assembly_task = nullptr;
            std::shared_ptr<ariac_common::CombinedTask> combined_task = nullptr;

            if (order_type == ariac_msgs::msg::Order::KITTING)
            {
                // RCLCPP_ERROR_STREAM(impl_->ros_node_->get_logger(), "Kitting task: " << order_id);
                kitting_task = BuildKittingTask(order->kitting_task);
            }
            else if (order_type == ariac_msgs::msg::Order::ASSEMBLY)
            {
                // RCLCPP_ERROR_STREAM(impl_->ros_node_->get_logger(), "Assembly task: " << order_id);
                assembly_task = BuildAssemblyTask(order->assembly_task);
            }
            else if (order_type == ariac_msgs::msg::Order::COMBINED)
            {
                // RCLCPP_ERROR_STREAM(impl_->ros_node_->get_logger(), "Combined task: " << order_id);
                combined_task = BuildCombinedTask(order->combined_task);
            }
            else
            {
                RCLCPP_ERROR_STREAM(impl_->ros_node_->get_logger(), "Unknown order type: " << int(order_type));
            }

            // Get the condition
            if (order->condition.type == ariac_msgs::msg::Condition::TIME)
            {
                // RCLCPP_ERROR_STREAM(impl_->ros_node_->get_logger(), "Order ID: " << order_id);
                auto announcement_time = order->condition.time_condition.seconds;
                // RCLCPP_ERROR_STREAM(impl_->ros_node_->get_logger(), "Time based order: " << order_id << " announcement time: " << announcement_time);
                auto order_instance = std::make_shared<ariac_common::OrderTemporal>(order_id, order_type, order_priority, impl_->time_limit_, announcement_time);
                // Set the task
                if (kitting_task)
                    order_instance->SetKittingTask(kitting_task);
                else if (assembly_task)
                    order_instance->SetAssemblyTask(assembly_task);
                else if (combined_task)
                    order_instance->SetCombinedTask(combined_task);

                // Add to the list of time based orders
                impl_->time_based_orders_.push_back(order_instance);
                impl_->all_orders_.push_back(order_instance);
            }
            else if (order->condition.type == ariac_msgs::msg::Condition::PART_PLACE)
            {
                auto agv = order->condition.part_place_condition.agv;
                auto part = std::make_shared<ariac_common::Part>(order->condition.part_place_condition.part.color, order->condition.part_place_condition.part.type);
                auto order_instance = std::make_shared<ariac_common::OrderOnPartPlacement>(order_id, order_type, order_priority, impl_->time_limit_, agv, part);
                // Set the task
                if (kitting_task)
                    order_instance->SetKittingTask(kitting_task);
                else if (assembly_task)
                    order_instance->SetAssemblyTask(assembly_task);
                else if (combined_task)
                    order_instance->SetCombinedTask(combined_task);

                // Add to the list of part placement orders
                impl_->on_part_placement_orders_.push_back(order_instance);
                impl_->all_orders_.push_back(order_instance);
            }
            else if (order->condition.type == ariac_msgs::msg::Condition::SUBMISSION)
            {
                auto submitted_order_id = order->condition.submission_condition.order_id;
                auto order_instance = std::make_shared<ariac_common::OrderOnSubmission>(order_id, order_type, order_priority, impl_->time_limit_, submitted_order_id);
                // Set the task
                if (kitting_task)
                    order_instance->SetKittingTask(kitting_task);
                else if (assembly_task)
                    order_instance->SetAssemblyTask(assembly_task);
                else if (combined_task)
                    order_instance->SetCombinedTask(combined_task);

                // Add to the list of submission orders
                impl_->on_order_submission_orders_.push_back(order_instance);
                impl_->all_orders_.push_back(order_instance);
            }
            else
            {
                RCLCPP_ERROR_STREAM(impl_->ros_node_->get_logger(), "Unknown condition type: " << int(order->condition.type));
            }
        }
    }

    //==============================================================================
    void TaskManagerPlugin::StoreRobotMalfunctionChallenges(const ariac_msgs::msg::RobotMalfunctionChallenge &_challenge)
    {
        // Get the duration of the current challenge
        auto duration = _challenge.duration;
        // Get the announcement condition of the current challenge
        auto condition = _challenge.condition;
        // Get the list of robots to disable
        auto robots_to_disable = _challenge.robots_to_disable;

        // List of robots to disable
        std::vector<std::string> robots_to_disable_vect;

        // Check which robots to disable
        if (robots_to_disable.ceiling_robot)
            robots_to_disable_vect.push_back("ceiling_robot");
        if (robots_to_disable.floor_robot)
            robots_to_disable_vect.push_back("floor_robot");

        // Check the announcement condition for the current challenge
        if (condition.type == ariac_msgs::msg::Condition::TIME)
        {
            auto trigger_time = condition.time_condition.seconds;
            auto robot_malfunction = std::make_shared<ariac_common::RobotMalfunctionTemporal>(duration, robots_to_disable_vect, trigger_time);
            impl_->time_based_robot_malfunctions_.push_back(robot_malfunction);
        }
        else if (condition.type == ariac_msgs::msg::Condition::PART_PLACE)
        {
            auto agv = condition.part_place_condition.agv;
            auto part = std::make_shared<ariac_common::Part>(condition.part_place_condition.part.color, condition.part_place_condition.part.type);
            auto robot_malfunction = std::make_shared<ariac_common::RobotMalfunctionOnPartPlacement>(duration, robots_to_disable_vect, part, agv);
            impl_->on_part_placement_robot_malfunctions_.push_back(robot_malfunction);
        }
        else if (condition.type == ariac_msgs::msg::Condition::SUBMISSION)
        {
            auto submitted_order_id = condition.submission_condition.order_id;
            auto robot_malfunction = std::make_shared<ariac_common::RobotMalfunctionOnSubmission>(duration, robots_to_disable_vect, submitted_order_id);
            impl_->on_submission_robot_malfunctions_.push_back(robot_malfunction);
        }
    }

    //==============================================================================
    void TaskManagerPlugin::StoreSensorBlackoutChallenges(const ariac_msgs::msg::SensorBlackoutChallenge &_challenge)
    {
        // Get the duration of the current challenge
        auto duration = _challenge.duration;
        // Get the announcement condition for the current challenge
        auto condition = _challenge.condition;
        // Get the sensors to disable for the current challenge
        auto sensors_to_disable = _challenge.sensors_to_disable;

        // Vector with only the sensors to disable
        std::vector<std::string> sensors_to_disable_vect;

        // Check which sensors to disable and add them to the vector
        if (sensors_to_disable.break_beam)
            sensors_to_disable_vect.push_back("break_beam");
        if (sensors_to_disable.proximity)
            sensors_to_disable_vect.push_back("proximity");
        if (sensors_to_disable.laser_profiler)
            sensors_to_disable_vect.push_back("laser_profiler");
        if (sensors_to_disable.lidar)
            sensors_to_disable_vect.push_back("lidar");
        if (sensors_to_disable.camera)
            sensors_to_disable_vect.push_back("camera");
        if (sensors_to_disable.logical_camera)
            sensors_to_disable_vect.push_back("logical_camera");

        // Check condition
        if (condition.type == ariac_msgs::msg::Condition::TIME)
        {
            auto trigger_time = condition.time_condition.seconds;
            auto sensor_blackout = std::make_shared<ariac_common::SensorBlackoutTemporal>(duration, sensors_to_disable_vect, trigger_time);
            impl_->time_based_sensor_blackouts_.push_back(sensor_blackout);
        }
        else if (condition.type == ariac_msgs::msg::Condition::PART_PLACE)
        {
            auto agv = condition.part_place_condition.agv;
            auto part = std::make_shared<ariac_common::Part>(condition.part_place_condition.part.color, condition.part_place_condition.part.type);
            auto sensor_blackout = std::make_shared<ariac_common::SensorBlackoutOnPartPlacement>(duration, sensors_to_disable_vect, part, agv);
            impl_->on_part_placement_sensor_blackouts_.push_back(sensor_blackout);
        }
        else if (condition.type == ariac_msgs::msg::Condition::SUBMISSION)
        {
            auto submitted_order_id = condition.submission_condition.order_id;
            auto sensor_blackout = std::make_shared<ariac_common::SensorBlackoutOnSubmission>(duration, sensors_to_disable_vect, submitted_order_id);
            impl_->on_submission_sensor_blackouts_.push_back(sensor_blackout);
        }
    }

    void TaskManagerPlugin::StoreFaultyPartChallenges(const ariac_msgs::msg::FaultyPartChallenge &_challenge)
    {
        auto faulty_part_challenge = std::make_shared<ariac_common::FaultyPartChallenge>(_challenge);
        impl_->faulty_part_challenges_.push_back(faulty_part_challenge);
    }

    void TaskManagerPlugin::StoreDroppedPartChallenges(const ariac_msgs::msg::DroppedPartChallenge &_challenge) {}

    void TaskManagerPlugin::StoreHumanChallenge(const ariac_msgs::msg::HumanChallenge &_challenge)
    {
        auto behavior = _challenge.behavior;
        // Get the announcement condition for the current challenge
        auto condition = _challenge.condition;

        // Check condition
        if (condition.type == ariac_msgs::msg::Condition::TIME)
        {
            auto trigger_time = condition.time_condition.seconds;
            auto human_challenge = std::make_shared<ariac_common::HumanChallengeTemporal>(behavior, trigger_time);
            impl_->time_based_human_challenge_ = human_challenge;
        }
        else if (condition.type == ariac_msgs::msg::Condition::PART_PLACE)
        {
            auto agv = condition.part_place_condition.agv;
            auto part = std::make_shared<ariac_common::Part>(condition.part_place_condition.part.color, condition.part_place_condition.part.type);
            auto human_challenge = std::make_shared<ariac_common::HumanChallengeOnPartPlacement>(behavior, part, agv);
            impl_->on_part_placement_human_challenge_ = human_challenge;
        }
        else if (condition.type == ariac_msgs::msg::Condition::SUBMISSION)
        {
            auto submitted_order_id = condition.submission_condition.order_id;
            auto human_challenge = std::make_shared<ariac_common::HumanChallengeOnSubmission>(behavior, submitted_order_id);
            impl_->on_order_submission_human_challenge_ = human_challenge;
        }
    }

    //==============================================================================
    void
    TaskManagerPlugin::StoreChallenges(const std::vector<ariac_msgs::msg::Challenge::SharedPtr> &challenges)
    {
        for (auto challenge : challenges)
        {
            auto challenge_type = challenge->type;

            if (challenge_type == ariac_msgs::msg::Challenge::SENSOR_BLACKOUT)
                StoreSensorBlackoutChallenges(challenge->sensor_blackout_challenge);
            else if (challenge_type == ariac_msgs::msg::Challenge::ROBOT_MALFUNCTION)
                StoreRobotMalfunctionChallenges(challenge->robot_malfunction_challenge);
            else if (challenge_type == ariac_msgs::msg::Challenge::FAULTY_PART)
                StoreFaultyPartChallenges(challenge->faulty_part_challenge);
            else if (challenge_type == ariac_msgs::msg::Challenge::DROPPED_PART)
                StoreDroppedPartChallenges(challenge->dropped_part_challenge);
            else if (challenge_type == ariac_msgs::msg::Challenge::HUMAN)
                StoreHumanChallenge(challenge->human_challenge);
            else
            {
                RCLCPP_ERROR_STREAM(impl_->ros_node_->get_logger(), "Unknown challenge type: " << int(challenge_type));
            }
        }
    }

    //==============================================================================
    void TaskManagerPlugin::UpdateSensorsHealth()
    {
        auto sensor_message = ariac_msgs::msg::Sensors();
        sensor_message.break_beam = impl_->break_beam_sensor_health_;
        sensor_message.proximity = impl_->proximity_sensor_health_;
        sensor_message.laser_profiler = impl_->laser_profiler_sensor_health_;
        sensor_message.lidar = impl_->lidar_sensor_health_;
        sensor_message.camera = impl_->camera_sensor_health_;
        sensor_message.logical_camera = impl_->logical_camera_sensor_health_;

        impl_->sensor_health_pub_->publish(sensor_message);
    }

    //==============================================================================
    void TaskManagerPlugin::UpdateRobotsHealth()
    {
        auto robots_message = ariac_msgs::msg::Robots();
        robots_message.ceiling_robot = impl_->ceiling_robot_health_;
        robots_message.floor_robot = impl_->floor_robot_health_;

        impl_->robot_health_pub_->publish(robots_message);
    }

    //==============================================================================
    int TaskManagerPlugin::StartAllRobots()
    {
        // publish on /ariac/robot_health topic
        auto robot_health_message = ariac_msgs::msg::Robots();
        robot_health_message.ceiling_robot = true;
        robot_health_message.floor_robot = true;
        impl_->robot_health_pub_->publish(robot_health_message);
        RCLCPP_INFO_STREAM(impl_->ros_node_->get_logger(), "Started all robots");
        return 1;
    }

    //==============================================================================
    // int TaskManagerPluginPrivate::ScoreQuadrant(ariac_msgs::msg::QualityIssue quadrant)
    // {
    //     int quadrant_score = 0;
    //     bool B = !quadrant.incorrect_part_type;
    //     bool C = !quadrant.incorrect_part_color;
    //     bool D = quadrant.flipped_part;
    //     bool E = quadrant.faulty_part;

    //     if (!B || E)
    //         quadrant_score = 0;

    //     if (B && C && !D && !E)
    //         quadrant_score = 3;

    //     if (B && !C && !D && !E)
    //         quadrant_score = 2;

    //     if (B && C && D && !E)
    //         quadrant_score = 2;

    //     if (B && !C && D && !E)
    //         quadrant_score = 1;

    //     return quadrant_score;
    // }

    int TaskManagerPluginPrivate::ScoreQuadrant(bool is_correct_part_type, bool is_correct_part_color, bool is_flipped_part, bool is_faulty_part)
    {
        bool B = is_correct_part_type;
        bool C = is_correct_part_color;
        bool D = is_flipped_part;
        bool E = is_faulty_part;

        if (!B || E)
            return 0;

        if (B && C && !D && !E)
            return 3;

        if (B && !C && !D && !E)
            return 2;

        if (B && C && D && !E)
            return 2;

        if (B && !C && D && !E)
            return 1;

        return 0;
    }

    //==============================================================================
    void TaskManagerPluginPrivate::ScoreKittingTask(std::shared_ptr<ariac_common::Order> _order, std::string _submitted_order_id)
    {

        // RCLCPP_DEBUG_STREAM(ros_node_->get_logger(), "Scoring kitting task for order: " << _submitted_order_id);

        // These shared pointers are uniquely for KittingScore
        std::shared_ptr<ariac_common::Quadrant> quadrant1_ptr = nullptr;
        std::shared_ptr<ariac_common::Quadrant> quadrant2_ptr = nullptr;
        std::shared_ptr<ariac_common::Quadrant> quadrant3_ptr = nullptr;
        std::shared_ptr<ariac_common::Quadrant> quadrant4_ptr = nullptr;

        // Bonus points for completing the order
        int bonus = 0;
        // Points for correct tray
        int tray_score = 0;

        int total_quadrants_score = 0;

        // destination
        int destination_score = 1;

        // penalty for having extra parts
        int penalty = 0;

        // current AGV location
        int agv_current_location = -1;

        // Get the kitting task for this order
        auto kitting_task = _order->GetKittingTask();
        auto expected_destination = kitting_task->GetDestination();

        int expected_number_of_parts = kitting_task->GetProducts().size();

        // Get the AGV for this kitting task
        auto expected_agv = kitting_task->GetAgvNumber();
        // RCLCPP_ERROR_STREAM(ros_node_->get_logger(), "Expected AGV: " << expected_agv);

        // Get AGV current location
        if (expected_agv == 1)
            agv_current_location = agv1_location_;
        else if (expected_agv == 2)
            agv_current_location = agv2_location_;
        else if (expected_agv == 3)
            agv_current_location = agv3_location_;
        else if (expected_agv == 4)
            agv_current_location = agv4_location_;

        // RCLCPP_ERROR_STREAM(ros_node_->get_logger(), "Expected AGV location: " << expected_destination);
        // RCLCPP_ERROR_STREAM(ros_node_->get_logger(), "Current AGV location: " << agv_current_location);

        // Get tray sensor information for this AGV
        auto shipment = ParseAGVTraySensorImage(agv_tray_images_[expected_agv]);

        std::map<int, int> quadrant_scores;
        quadrant_scores[1] = 0;
        quadrant_scores[2] = 0;
        quadrant_scores[3] = 0;
        quadrant_scores[4] = 0;

        // If faulty part challenge is used in the trial
        std::array<bool, 4> faulty_parts = {false, false, false, false};
        if (faulty_part_challenges_.size() > 0)
        {
            for (auto &challenge : faulty_part_challenges_)
            {
                if (challenge->GetOrderId() == _submitted_order_id)
                {
                    faulty_parts = HandleFaultyParts(challenge, *kitting_task, shipment);
                }
            }
        }

        // Parse the kitting task and get the parts
        for (auto product : kitting_task->GetProducts())
        {
            for (auto tray_part : shipment.GetTrayParts())
            {
                if (tray_part.GetQuadrant() == product.GetQuadrant())
                {
                    auto quadrant = product.GetQuadrant();
                    // RCLCPP_ERROR_STREAM(ros_node_->get_logger(), "Inspecting quadrant: " << quadrant);
                    bool is_missing_part = false;
                    bool is_correct_part_type = tray_part.isCorrectType(product.GetPart().GetType());
                    bool is_correct_part_color = tray_part.isCorrectColor(product.GetPart().GetColor());
                    bool is_faulty_part = faulty_parts[quadrant - 1];
                    bool is_flipped_part = tray_part.isFlipped();

                    quadrant_scores[quadrant] = ScoreQuadrant(is_correct_part_type, is_correct_part_color, is_flipped_part, is_faulty_part);

                    // RCLCPP_INFO_STREAM(ros_node_->get_logger(), "Score: " << quadrant_scores[quadrant]);

                    auto quadrant_ptr = std::make_shared<ariac_common::Quadrant>(quadrant, is_correct_part_type, is_correct_part_color, is_faulty_part, is_flipped_part, quadrant_scores[quadrant]);
                    if (quadrant == 1)
                        quadrant1_ptr = quadrant_ptr;
                    else if (quadrant == 2)
                        quadrant2_ptr = quadrant_ptr;
                    else if (quadrant == 3)
                        quadrant3_ptr = quadrant_ptr;
                    else if (quadrant == 4)
                        quadrant4_ptr = quadrant_ptr;
                }
            }
        }

        // Check isCorrectTrayId
        if (kitting_task->GetTrayId() == shipment.GetTrayId())
            tray_score = 3;

        // Sum quadrant scores
        total_quadrants_score = quadrant_scores[1] + quadrant_scores[2] + quadrant_scores[3] + quadrant_scores[4];

        std::cout << "Quadrant 1 score: " << quadrant_scores[1] << std::endl;
        std::cout << "Quadrant 2 score: " << quadrant_scores[2] << std::endl;
        std::cout << "Quadrant 3 score: " << quadrant_scores[3] << std::endl;
        std::cout << "Quadrant 4 score: " << quadrant_scores[4] << std::endl;
        std::cout << "Expected number of parts: " << expected_number_of_parts << std::endl;
        std::cout << "Total quadrant score: " << total_quadrants_score << std::endl;

        // Compute bonus
        if (total_quadrants_score == 3 * expected_number_of_parts)
            bonus = expected_number_of_parts;

        // Compute penalty for having extra parts in the tray
        if (shipment.GetTrayParts().size() > expected_number_of_parts)
            penalty = shipment.GetTrayParts().size() - expected_number_of_parts;

        // Check destination
        if (agv_current_location != expected_destination)
            destination_score = 0;

        // Compute the score for the submitted kit
        int kit_score = std::max(tray_score + total_quadrants_score + bonus - penalty, 0) * destination_score;

        // Create a kitting score object
        auto kitting_score = std::make_shared<ariac_common::KittingScore>(
            _order->GetId(),
            kit_score,
            tray_score,
            quadrant1_ptr,
            quadrant2_ptr,
            quadrant3_ptr,
            quadrant4_ptr,
            bonus,
            penalty,
            expected_agv,
            destination_score);

        // Set the kitting score for this order
        _order->SetKittingScore(kitting_score);

        PrintKittingScore(kitting_score);

        // Display the score on the screen
        // RCLCPP_INFO_STREAM(ros_node_->get_logger(), *kitting_score);
    }

    //==============================================================================
    void TaskManagerPluginPrivate::PrintKittingScore(std::shared_ptr<ariac_common::KittingScore> kitting_score)
    {
        std::string output = "";
        output += "\n========================================\n";
        output += "Order: " + kitting_score->GetOrderId() + "\n";
        output += "Type: Kitting\n";
        output += "AGV: " + std::to_string(kitting_score->GetAGV()) + "\n";
        output += "Tray score: " + std::to_string(kitting_score->GetTrayScore()) + "\n";
        kitting_score->GetDestinationScore() == 1 ? output += "Correct destination: Yes\n" : output += "Correct destination: No\n";
        output += "Bonus: " + std::to_string(kitting_score->GetBonus()) + "\n";
        output += "Score: " + std::to_string(kitting_score->GetScore()) + "\n";

        auto quadrant1 = kitting_score->GetQuadrant1();
        auto quadrant2 = kitting_score->GetQuadrant2();
        auto quadrant3 = kitting_score->GetQuadrant3();
        auto quadrant4 = kitting_score->GetQuadrant4();

        if (quadrant1 != nullptr)
        {
            output += "========================================\n";
            output += "Quadrant: 1\n";
            output += "Quadrant score: " + std::to_string(quadrant1->GetScore()) + "\n";
            output += "----------------------------------------\n";
            quadrant1->GetIsCorrectPartType() ? output += "Correct part type: Yes\n" : output += "Correct part type: No\n";
            quadrant1->GetIsCorrectPartColor() ? output += "Correct part color: Yes\n" : output += "Correct part color: No\n";
            quadrant1->GetIsFaulty() ? output += "Faulty part: Yes\n" : output += "Faulty part: No\n";
            quadrant1->GetIsFlipped() ? output += "Flipped part: Yes\n" : output += "Flipped part: No\n";
        }
        if (quadrant2 != nullptr)
        {
            output += "========================================\n";
            output += "Quadrant: 2\n";
            output += "Quadrant score: " + std::to_string(quadrant2->GetScore()) + "\n";
            output += "----------------------------------------\n";
            quadrant2->GetIsCorrectPartType() ? output += "Correct part type: Yes\n" : output += "Correct part type: No\n";
            quadrant2->GetIsCorrectPartColor() ? output += "Correct part color: Yes\n" : output += "Correct part color: No\n";
            quadrant2->GetIsFaulty() ? output += "Faulty: Yes\n" : output += "Faulty: No\n";
            quadrant2->GetIsFlipped() ? output += "Flipped: Yes\n" : output += "Flipped: No\n";
        }
        if (quadrant3 != nullptr)
        {
            output += "========================================\n";
            output += "Quadrant: 3\n";
            output += "Quadrant score: " + std::to_string(quadrant3->GetScore()) + "\n";
            output += "----------------------------------------\n";
            quadrant3->GetIsCorrectPartType() ? output += "Correct part type: Yes\n" : output += "Correct part type: No\n";
            quadrant3->GetIsCorrectPartColor() ? output += "Correct part color: Yes\n" : output += "Correct part color: No\n";
            quadrant3->GetIsFaulty() ? output += "Faulty: Yes\n" : output += "Faulty: No\n";
            quadrant3->GetIsFlipped() ? output += "Flipped: Yes\n" : output += "Flipped: No\n";
        }
        if (quadrant4 != nullptr)
        {
            output += "========================================\n";
            output += "Quadrant: 4\n";
            output += "Quadrant score: " + std::to_string(quadrant4->GetScore()) + "\n";
            output += "----------------------------------------\n";
            quadrant4->GetIsCorrectPartType() ? output += "Correct part type: Yes\n" : output += "Correct part type: No\n";
            quadrant4->GetIsCorrectPartColor() ? output += "Correct part color: Yes\n" : output += "Correct part color: No\n";
            quadrant4->GetIsFaulty() ? output += "Faulty: Yes\n" : output += "Faulty: No\n";
            quadrant4->GetIsFlipped() ? output += "Flipped: Yes\n" : output += "Flipped: No\n";
        }

        RCLCPP_INFO_STREAM(ros_node_->get_logger(), output);
        log_message_ += output;
    }

    //==============================================================================
    void TaskManagerPluginPrivate::ScoreAssemblyTask(std::shared_ptr<ariac_common::Order> _order, std::string _submitted_order_id)
    {
        // These shared pointers are uniquely for KittingScore
        std::shared_ptr<ariac_common::ScoredAssemblyPart> battery_ptr = nullptr;
        std::shared_ptr<ariac_common::ScoredAssemblyPart> pump_ptr = nullptr;
        std::shared_ptr<ariac_common::ScoredAssemblyPart> regulator_ptr = nullptr;
        std::shared_ptr<ariac_common::ScoredAssemblyPart> sensor_ptr = nullptr;

        // 2 cm tolerance for the translation
        double translation_target{0.02};
        // 15 deg tolerance for the rotation
        double rotation_target{0.261799};

        // Bonus points for completing the order
        int bonus = 0;

        int parts_score = 0;

        // score for correct station
        int station_score = 0;

        // Get the assembly task for this order
        auto assembly_task = _order->GetAssemblyTask();
        auto expected_station = assembly_task->GetStation();

        int expected_number_of_parts = assembly_task->GetProducts().size();

        // Get insert sensor information for this station
        auto shipment = ParseAssemblyStationImage(assembly_station_images_[expected_station]);

        // Parse the assembly task and get the parts
        for (auto order_product : assembly_task->GetProducts())
        {
            // Parse the shipment and get the parts
            for (auto shipment_product : shipment.GetInsertParts())
            {
                auto shipment_part = shipment_product.GetPart();
                auto shipment_part_pose = shipment_product.GetPoseInInsert();
                auto order_part_pose = order_product.GetPartPose();

                bool is_correct_pose = true;
                bool is_correct_part_type = shipment_product.isCorrectType(order_product.GetPart().GetType());
                bool is_correct_part_color = shipment_product.isCorrectColor(order_product.GetPart().GetColor());

                if (is_correct_part_type)
                {
                    int part_score = 0;

                    // compute translation distance
                    ignition::math::Vector3d position_diff(
                        order_part_pose.X() - shipment_part_pose.position.x,
                        order_part_pose.Y() - shipment_part_pose.position.y,
                        order_part_pose.Z() - shipment_part_pose.position.z);

                    const double translation_distance = position_diff.Length();
                    if (translation_distance > translation_target)
                    {
                        is_correct_pose = false;
                    }

                    // compute orientation distance
                    // quaternion for the part in the order
                    ignition::math::Quaterniond order_part_q(
                        order_part_pose.Rot().W(),
                        order_part_pose.Rot().X(),
                        order_part_pose.Rot().Y(),
                        order_part_pose.Rot().Z());

                    // quaternion for the part in the shipment
                    ignition::math::Quaterniond shipment_part_q(
                        shipment_part_pose.orientation.w,
                        shipment_part_pose.orientation.x,
                        shipment_part_pose.orientation.y,
                        shipment_part_pose.orientation.z);

                    // Filter products that aren't in the appropriate orientation (loosely).
                    // If the quaternions represent the same orientation, q1 = +-q2 => q1.dot(q2) = +-1
                    const double orientation_diff = shipment_part_q.Dot(order_part_q);
                    const double orientation_threshold = 0.05;

                    if (std::abs(orientation_diff) < (1.0 - orientation_threshold))
                    {
                        is_correct_pose = false;
                    }

                    // Compute the score for this part
                    if (is_correct_part_type && is_correct_part_color && is_correct_pose)
                    {
                        part_score = 3;
                    }
                    else if (is_correct_part_type && (is_correct_part_color || is_correct_pose))
                    {
                        part_score = 2;
                    }
                    else if (is_correct_part_type && (!is_correct_part_color && !is_correct_pose))
                    {
                        part_score = 1;
                    }
                    else
                    {
                        part_score = 0;
                    }

                    // Part orientation in RPY
                    tf2::Quaternion q(
                        shipment_part_pose.orientation.x,
                        shipment_part_pose.orientation.y,
                        shipment_part_pose.orientation.z,
                        shipment_part_pose.orientation.w);

                    tf2::Matrix3x3 m(q);
                    double roll, pitch, yaw;
                    m.getRPY(roll, pitch, yaw);

                    geometry_msgs::msg::Vector3 part_position;
                    part_position.x = shipment_part_pose.position.x;
                    part_position.y = shipment_part_pose.position.y;
                    part_position.z = shipment_part_pose.position.z;

                    geometry_msgs::msg::Vector3 part_orientation;
                    part_orientation.x = roll;
                    part_orientation.y = pitch;
                    part_orientation.z = yaw;

                    if (order_product.GetPart().GetType() == ariac_msgs::msg::Part::BATTERY)
                    {
                        bool faulty_part = false;
                        battery_ptr = std::make_shared<ariac_common::ScoredAssemblyPart>(
                            shipment_part, is_correct_part_color, is_correct_pose, part_position, part_orientation, part_score, faulty_part);
                    }
                    else if (order_product.GetPart().GetType() == ariac_msgs::msg::Part::PUMP)
                    {
                        bool faulty_part = false;
                        pump_ptr = std::make_shared<ariac_common::ScoredAssemblyPart>(shipment_part, is_correct_part_color, is_correct_pose, part_position, part_orientation, part_score, faulty_part);
                    }
                    else if (order_product.GetPart().GetType() == ariac_msgs::msg::Part::REGULATOR)
                    {
                        bool faulty_part = false;
                        regulator_ptr = std::make_shared<ariac_common::ScoredAssemblyPart>(shipment_part, is_correct_part_color, is_correct_pose, part_position, part_orientation, part_score, faulty_part);
                    }
                    else if (order_product.GetPart().GetType() == ariac_msgs::msg::Part::SENSOR)
                    {
                        bool faulty_part = false;
                        sensor_ptr = std::make_shared<ariac_common::ScoredAssemblyPart>(shipment_part, is_correct_part_color, is_correct_pose, part_position, part_orientation, part_score, faulty_part);
                    }

                    parts_score += part_score;
                }
            }
        }
        // Check isCorrectStation
        if (assembly_task->GetStation() == shipment.GetStation())
            station_score = 1;

        // Compute bonus points
        if (parts_score == expected_number_of_parts * 3)
            bonus = expected_number_of_parts * 4;

        // Compute the score for the submitted shipment
        int insert_score = (parts_score + bonus) * station_score;

        // Create a shared pointer to an AssemblyScore object
        auto assembly_score = std::make_shared<ariac_common::AssemblyScore>(
            _order->GetId(),
            insert_score,
            shipment.GetStation(),
            battery_ptr,
            pump_ptr,
            regulator_ptr,
            sensor_ptr,
            bonus);

        // Set the score for this assembly task
        _order->SetAssemblyScore(assembly_score);

        // Display the score in the terminal
        PrintAssemblyScore(assembly_score);
    }

    //==============================================================================
    void TaskManagerPluginPrivate::PrintAssemblyScore(std::shared_ptr<ariac_common::AssemblyScore> assembly_score)
    {
        std::string output = "";
        output += "\n========================================\n";
        output += "Order: " + assembly_score->GetOrderId() + "\n";
        output += "Type: Assembly\n";
        output += "Station: " + std::to_string(assembly_score->GetStation()) + "\n";
        output += "Bonus: " + std::to_string(assembly_score->GetBonus()) + "\n";
        output += "Score: " + std::to_string(assembly_score->GetScore()) + "\n";

        auto battery = assembly_score->GetBatteryPtr();
        auto pump = assembly_score->GetPumpPtr();
        auto regulator = assembly_score->GetRegulatorPtr();
        auto sensor = assembly_score->GetSensorPtr();
        if (battery != nullptr)
        {
            auto battery_type = ariac_common::ConvertPartTypeToString(battery->GetPart().GetType());
            auto battery_color = ariac_common::ConvertPartColorToString(battery->GetPart().GetColor());
            output += "========================================\n";
            output += "Part: [" + battery_type + "," + battery_color + "]\n";
            output += "----------------------------------------\n";
            output += "Type: Correct\n";
            battery->GetCorrectColor() ? output += "Color: Correct\n" : output += "Color: Incorrect\n";
            battery->GetCorrectPose() ? output += "Pose: Correct\n" : output += "Pose: Incorrect\n";
            output += "Position: " + std::to_string(battery->GetPosition().x) + ", " + std::to_string(battery->GetPosition().y) + ", " + std::to_string(battery->GetPosition().z) + "\n";
            output += "Orientation: " + std::to_string(battery->GetOrientation().x) + ", " + std::to_string(battery->GetOrientation().y) + ", " + std::to_string(battery->GetOrientation().z) + "\n";
            output += "Part score: " + std::to_string(battery->GetScore()) + "\n";
        }
        if (pump != nullptr)
        {
            auto pump_type = ariac_common::ConvertPartTypeToString(pump->GetPart().GetType());
            auto pump_color = ariac_common::ConvertPartColorToString(pump->GetPart().GetColor());
            output += "========================================\n";
            output += "Part: [" + pump_type + "," + pump_color + "]\n";
            output += "----------------------------------------\n";
            output += "Type: Correct\n";
            pump->GetCorrectColor() ? output += "Color: Correct\n" : output += "Color: Incorrect\n";
            pump->GetCorrectPose() ? output += "Pose: Correct\n" : output += "Pose: Incorrect\n";
            output += "Position: " + std::to_string(pump->GetPosition().x) + ", " + std::to_string(pump->GetPosition().y) + ", " + std::to_string(pump->GetPosition().z) + "\n";
            output += "Orientation: " + std::to_string(pump->GetOrientation().x) + ", " + std::to_string(pump->GetOrientation().y) + ", " + std::to_string(pump->GetOrientation().z) + "\n";
            output += "Part score: " + std::to_string(pump->GetScore()) + "\n";
        }
        if (regulator != nullptr)
        {
            auto regulator_type = ariac_common::ConvertPartTypeToString(regulator->GetPart().GetType());
            auto regulator_color = ariac_common::ConvertPartColorToString(regulator->GetPart().GetColor());
            output += "========================================\n";
            output += "Part: [" + regulator_type + "," + regulator_color + "]\n";
            output += "----------------------------------------\n";
            output += "Type: Correct\n";
            regulator->GetCorrectColor() ? output += "Color: Correct\n" : output += "Color: Incorrect\n";
            regulator->GetCorrectPose() ? output += "Pose: Correct\n" : output += "Pose: Incorrect\n";
            output += "Position: " + std::to_string(regulator->GetPosition().x) + ", " + std::to_string(regulator->GetPosition().y) + ", " + std::to_string(regulator->GetPosition().z) + "\n";
            output += "Orientation: " + std::to_string(regulator->GetOrientation().x) + ", " + std::to_string(regulator->GetOrientation().y) + ", " + std::to_string(regulator->GetOrientation().z) + "\n";
            output += "Part score: " + std::to_string(regulator->GetScore()) + "\n";
        }
        if (sensor != nullptr)
        {
            auto sensor_type = ariac_common::ConvertPartTypeToString(sensor->GetPart().GetType());
            auto sensor_color = ariac_common::ConvertPartColorToString(sensor->GetPart().GetColor());
            output += "========================================\n";
            output += "Part: [" + sensor_type + "," + sensor_color + "]\n";
            output += "----------------------------------------\n";
            output += "Type: Correct\n";
            sensor->GetCorrectColor() ? output += "Color: Correct\n" : output += "Color: Incorrect\n";
            sensor->GetCorrectPose() ? output += "Pose: Correct\n" : output += "Pose: Incorrect\n";
            output += "Position: " + std::to_string(sensor->GetPosition().x) + ", " + std::to_string(sensor->GetPosition().y) + ", " + std::to_string(sensor->GetPosition().z) + "\n";
            output += "Orientation: " + std::to_string(sensor->GetOrientation().x) + ", " + std::to_string(sensor->GetOrientation().y) + ", " + std::to_string(sensor->GetOrientation().z) + "\n";
            output += "Part score: " + std::to_string(sensor->GetScore()) + "\n";
            output += "----------------------------------------\n";
        }

        RCLCPP_INFO_STREAM(ros_node_->get_logger(), output);
        log_message_ += output;
    }

    //==============================================================================
    void TaskManagerPluginPrivate::PrintCombinedScore(std::shared_ptr<ariac_common::CombinedScore> combined_score)
    {
        std::string output = "";
        output += "\n========================================\n";
        output += "Order: " + combined_score->GetOrderId() + "\n";
        output += "Type: Combined\n";
        output += "Station: " + std::to_string(combined_score->GetStation()) + "\n";
        output += "Bonus: " + std::to_string(combined_score->GetBonus()) + "\n";
        output += "Score: " + std::to_string(combined_score->GetScore()) + "\n";

        auto battery = combined_score->GetBatteryPtr();
        auto pump = combined_score->GetPumpPtr();
        auto regulator = combined_score->GetRegulatorPtr();
        auto sensor = combined_score->GetSensorPtr();
        if (battery != nullptr)
        {
            auto battery_type = ariac_common::ConvertPartTypeToString(battery->GetPart().GetType());
            auto battery_color = ariac_common::ConvertPartColorToString(battery->GetPart().GetColor());
            output += "========================================\n";
            output += "Part: [" + battery_type + "," + battery_color + "]\n";
            output += "----------------------------------------\n";
            output += "Type: Correct\n";
            battery->GetCorrectColor() ? output += "Color: Correct\n" : output += "Color: Incorrect\n";
            battery->GetCorrectPose() ? output += "Pose: Correct\n" : output += "Pose: Incorrect\n";
            battery->GetIsFaulty() ? output += "Faulty: Yes\n" : output += "Faulty: No\n";
            output += "Position: " + std::to_string(battery->GetPosition().x) + ", " + std::to_string(battery->GetPosition().y) + ", " + std::to_string(battery->GetPosition().z) + "\n";
            output += "Orientation: " + std::to_string(battery->GetOrientation().x) + ", " + std::to_string(battery->GetOrientation().y) + ", " + std::to_string(battery->GetOrientation().z) + "\n";
            output += "Part score: " + std::to_string(battery->GetScore()) + "\n";
        }
        if (pump != nullptr)
        {
            auto pump_type = ariac_common::ConvertPartTypeToString(pump->GetPart().GetType());
            auto pump_color = ariac_common::ConvertPartColorToString(pump->GetPart().GetColor());
            output += "========================================\n";
            output += "Part: [" + pump_type + "," + pump_color + "]\n";
            output += "----------------------------------------\n";
            output += "Type: Correct\n";
            pump->GetCorrectColor() ? output += "Color: Correct\n" : output += "Color: Incorrect\n";
            pump->GetCorrectPose() ? output += "Pose: Correct\n" : output += "Pose: Incorrect\n";
            pump->GetIsFaulty() ? output += "Faulty: Yes\n" : output += "Faulty: No\n";
            output += "Position: " + std::to_string(pump->GetPosition().x) + ", " + std::to_string(pump->GetPosition().y) + ", " + std::to_string(pump->GetPosition().z) + "\n";
            output += "Orientation: " + std::to_string(pump->GetOrientation().x) + ", " + std::to_string(pump->GetOrientation().y) + ", " + std::to_string(pump->GetOrientation().z) + "\n";
            output += "Part score: " + std::to_string(pump->GetScore()) + "\n";
        }
        if (regulator != nullptr)
        {
            auto regulator_type = ariac_common::ConvertPartTypeToString(regulator->GetPart().GetType());
            auto regulator_color = ariac_common::ConvertPartColorToString(regulator->GetPart().GetColor());
            output += "========================================\n";
            output += "Part: [" + regulator_type + "," + regulator_color + "]\n";
            output += "----------------------------------------\n";
            output += "Type: Correct\n";
            regulator->GetCorrectColor() ? output += "Color: Correct\n" : output += "Color: Incorrect\n";
            regulator->GetCorrectPose() ? output += "Pose: Correct\n" : output += "Pose: Incorrect\n";
            regulator->GetIsFaulty() ? output += "Faulty: Yes\n" : output += "Faulty: No\n";
            output += "Position: " + std::to_string(regulator->GetPosition().x) + ", " + std::to_string(regulator->GetPosition().y) + ", " + std::to_string(regulator->GetPosition().z) + "\n";
            output += "Orientation: " + std::to_string(regulator->GetOrientation().x) + ", " + std::to_string(regulator->GetOrientation().y) + ", " + std::to_string(regulator->GetOrientation().z) + "\n";
            output += "Part score: " + std::to_string(regulator->GetScore()) + "\n";
        }
        if (sensor != nullptr)
        {
            auto sensor_type = ariac_common::ConvertPartTypeToString(sensor->GetPart().GetType());
            auto sensor_color = ariac_common::ConvertPartColorToString(sensor->GetPart().GetColor());
            output += "========================================\n";
            output += "Part: [" + sensor_type + "," + sensor_color + "]\n";
            output += "----------------------------------------\n";
            output += "Type: Correct\n";
            sensor->GetCorrectColor() ? output += "Color: Correct\n" : output += "Color: Incorrect\n";
            sensor->GetCorrectPose() ? output += "Pose: Correct\n" : output += "Pose: Incorrect\n";
            sensor->GetIsFaulty() ? output += "Faulty: Yes\n" : output += "Faulty: No\n";
            output += "Position: " + std::to_string(sensor->GetPosition().x) + ", " + std::to_string(sensor->GetPosition().y) + ", " + std::to_string(sensor->GetPosition().z) + "\n";
            output += "Orientation: " + std::to_string(sensor->GetOrientation().x) + ", " + std::to_string(sensor->GetOrientation().y) + ", " + std::to_string(sensor->GetOrientation().z) + "\n";
            output += "Part score: " + std::to_string(sensor->GetScore()) + "\n";
            output += "----------------------------------------\n";
        }

        RCLCPP_INFO_STREAM(ros_node_->get_logger(), output);
        log_message_ += output;
    }

    //==============================================================================
    void TaskManagerPluginPrivate::ScoreCombinedTask(std::shared_ptr<ariac_common::Order> _order, std::string _submitted_order_id)
    {
        // These shared pointers are uniquely for CombinedTask
        std::shared_ptr<ariac_common::ScoredAssemblyPart> battery_ptr = nullptr;
        std::shared_ptr<ariac_common::ScoredAssemblyPart> pump_ptr = nullptr;
        std::shared_ptr<ariac_common::ScoredAssemblyPart> regulator_ptr = nullptr;
        std::shared_ptr<ariac_common::ScoredAssemblyPart> sensor_ptr = nullptr;

        // 2 cm tolerance for the translation
        double translation_target{0.02};
        // 15 deg tolerance for the rotation
        double rotation_target{0.261799};

        // Bonus points for completing the order
        int bonus = 0;

        int parts_score = 0;

        // score for correct station
        int station_score = 0;

        // Get the assembly task for this order
        auto combined_task = _order->GetCombinedTask();
        auto expected_station = combined_task->GetStation();

        int expected_number_of_parts = combined_task->GetProducts().size();

        // Get insert sensor information for this station
        auto shipment = ParseAssemblyStationImage(assembly_station_images_[expected_station]);

        // If faulty part challenge is used in the trial
        // std::array<bool, 4> faulty_parts = {false, false, false, false};
        // if (faulty_part_challenges_.size() > 0)
        // {
        //     for (auto &challenge : faulty_part_challenges_)
        //     {
        //         if (challenge->GetOrderId() == _submitted_order_id)
        //         {
        //             faulty_parts = HandleFaultyParts(challenge, *kitting_task, shipment);
        //         }
        //     }
        // }

        // Parse the assembly task and get the parts
        for (auto order_product : combined_task->GetProducts())
        {
            // Parse the shipment and get the parts
            for (auto shipment_product : shipment.GetInsertParts())
            {

                auto shipment_part = shipment_product.GetPart();
                auto shipment_part_pose = shipment_product.GetPoseInInsert();
                auto order_part_pose = order_product.GetPartPose();

                bool is_correct_pose = true;
                bool is_correct_part_type = shipment_product.isCorrectType(order_product.GetPart().GetType());
                bool is_correct_part_color = shipment_product.isCorrectColor(order_product.GetPart().GetColor());

                if (is_correct_part_type)
                {
                    int part_score = 0;
                    // needs to be computed and not hard coded
                    bool is_faulty = false;

                    // compute translation distance
                    ignition::math::Vector3d position_diff(
                        order_part_pose.X() - shipment_part_pose.position.x,
                        order_part_pose.Y() - shipment_part_pose.position.y,
                        order_part_pose.Z() - shipment_part_pose.position.z);

                    const double translation_distance = position_diff.Length();
                    if (translation_distance > translation_target)
                    {
                        is_correct_pose = false;
                    }

                    // compute orientation distance
                    // quaternion for the part in the order
                    ignition::math::Quaterniond order_part_q(
                        order_part_pose.Rot().W(),
                        order_part_pose.Rot().X(),
                        order_part_pose.Rot().Y(),
                        order_part_pose.Rot().Z());

                    // quaternion for the part in the shipment
                    ignition::math::Quaterniond shipment_part_q(
                        shipment_part_pose.orientation.w,
                        shipment_part_pose.orientation.x,
                        shipment_part_pose.orientation.y,
                        shipment_part_pose.orientation.z);

                    // Filter products that aren't in the appropriate orientation (loosely).
                    // If the quaternions represent the same orientation, q1 = +-q2 => q1.dot(q2) = +-1
                    const double orientation_diff = shipment_part_q.Dot(order_part_q);
                    const double orientation_threshold = 0.05;

                    if (std::abs(orientation_diff) < (1.0 - orientation_threshold))
                    {
                        is_correct_pose = false;
                    }

                    // Compute the score for this part
                    if (!is_correct_part_type || is_faulty)
                    {
                        part_score = 0;
                    }
                    else if (is_correct_part_type && is_correct_part_color && is_correct_pose)
                    {
                        part_score = 5;
                    }
                    else if (is_correct_part_type && (is_correct_part_color || is_correct_pose))
                    {
                        part_score = 4;
                    }
                    else if (is_correct_part_type && (!is_correct_part_color && !is_correct_pose))
                    {
                        part_score = 3;
                    }
                    else
                    {
                        part_score = 0;
                    }

                    // Convert the position and orientation to RPY for display in the terminal
                    tf2::Quaternion q(
                        shipment_part_pose.orientation.x,
                        shipment_part_pose.orientation.y,
                        shipment_part_pose.orientation.z,
                        shipment_part_pose.orientation.w);

                    tf2::Matrix3x3 m(q);
                    double roll, pitch, yaw;
                    m.getRPY(roll, pitch, yaw);

                    geometry_msgs::msg::Vector3 part_position;
                    part_position.x = shipment_part_pose.position.x;
                    part_position.y = shipment_part_pose.position.y;
                    part_position.z = shipment_part_pose.position.z;

                    geometry_msgs::msg::Vector3 part_orientation;
                    part_orientation.x = roll;
                    part_orientation.y = pitch;
                    part_orientation.z = yaw;

                    if (order_product.GetPart().GetType() == ariac_msgs::msg::Part::BATTERY)
                    {
                        battery_ptr = std::make_shared<ariac_common::ScoredAssemblyPart>(
                            shipment_part, is_correct_part_color, is_correct_pose, part_position, part_orientation, part_score, is_faulty);
                    }
                    else if (order_product.GetPart().GetType() == ariac_msgs::msg::Part::PUMP)
                    {
                        pump_ptr = std::make_shared<ariac_common::ScoredAssemblyPart>(shipment_part, is_correct_part_color, is_correct_pose, part_position, part_orientation, part_score, is_faulty);
                    }
                    else if (order_product.GetPart().GetType() == ariac_msgs::msg::Part::REGULATOR)
                    {
                        regulator_ptr = std::make_shared<ariac_common::ScoredAssemblyPart>(shipment_part, is_correct_part_color, is_correct_pose, part_position, part_orientation, part_score, is_faulty);
                    }
                    else if (order_product.GetPart().GetType() == ariac_msgs::msg::Part::SENSOR)
                    {
                        sensor_ptr = std::make_shared<ariac_common::ScoredAssemblyPart>(shipment_part, is_correct_part_color, is_correct_pose, part_position, part_orientation, part_score, is_faulty);
                    }

                    parts_score += part_score;
                }
            }
        }
        // Check isCorrectStation
        if (combined_task->GetStation() == shipment.GetStation())
            station_score = 1;

        // Compute bonus points
        if (parts_score == expected_number_of_parts * 5)
            bonus = expected_number_of_parts * 4;

        // Compute the score for the submitted shipment
        int insert_score = (parts_score + bonus) * station_score;

        // Create a combined score object
        auto combined_score = std::make_shared<ariac_common::CombinedScore>(
            _order->GetId(),
            insert_score,
            shipment.GetStation(),
            battery_ptr,
            pump_ptr,
            regulator_ptr,
            sensor_ptr,
            bonus);

        // Set the score for this combined task
        _order->SetCombinedScore(combined_score);

        // Display the score in the terminal
        PrintCombinedScore(combined_score);
    }

    // ==================================== //
    // Services
    // ==================================== //

    //==============================================================================
    bool TaskManagerPlugin::SubmitOrderServiceCallback(
        const std::shared_ptr<ariac_msgs::srv::SubmitOrder::Request> request,
        std::shared_ptr<ariac_msgs::srv::SubmitOrder::Response> response)
    {

        std::lock_guard<std::mutex> lock(impl_->lock_);

        if (this->impl_->current_state_ == ariac_msgs::msg::CompetitionState::ENDED)
        {
            RCLCPP_WARN(impl_->ros_node_->get_logger(), "Competition has ended. No more orders can be submitted.");
            response->success = false;
            response->message = "Competition has ended. No more orders can be submitted.";
            return true;
        }

        auto submitted_order_id = request->order_id;

        // If the order id is part of the trial orders, then it is submitted
        if (std::find(impl_->trial_orders_.begin(), impl_->trial_orders_.end(), submitted_order_id) != impl_->trial_orders_.end())
        {
            // We need to set the submission time to compute the overall score against other competitors
            for (auto &order : impl_->all_orders_)
            {
                if (order->GetId() == submitted_order_id)
                {
                    order->SetIsSubmitted();
                    order->SetSubmittedTime(impl_->elapsed_time_);
                    // Score the kitting task
                    if (order->GetKittingTask())
                    {
                        impl_->ScoreKittingTask(order, request->order_id);
                    }
                    else if (order->GetAssemblyTask())
                    {
                        impl_->ScoreAssemblyTask(order, request->order_id);
                    }
                    else if (order->GetCombinedTask())
                    {
                        impl_->ScoreCombinedTask(order, request->order_id);
                    }
                }

                // Push the order id to the list of submitted orders
                impl_->submitted_orders_.push_back(submitted_order_id);
                response->success = true;
                response->message = "Order submitted successfully";
                impl_->trial_orders_.erase(std::remove(impl_->trial_orders_.begin(), impl_->trial_orders_.end(), submitted_order_id), impl_->trial_orders_.end());
            }
        }
        else
        {
            response->success = false;
            response->message = "Order is not part of the trial or has already been submitted";
        }
        return true;
    }

    //==============================================================================
    bool TaskManagerPlugin::StartCompetitionServiceCallback(
        const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response)
    {
        std::lock_guard<std::mutex> lock(impl_->lock_);

        // gzdbg << "\n";
        // gzdbg << "StartCompetitionServiceCallback\n";

        (void)request;

        if (impl_->current_state_ == ariac_msgs::msg::CompetitionState::READY)
        {
            impl_->current_state_ = ariac_msgs::msg::CompetitionState::STARTED;
            response->success = true;
            response->message = "Competition started successfully!";

            // Activate all sensors
            // ActivateAllSensors();
            // Start all robot controllers
            // StartAllRobots();

            return true;
        }
        response->success = false;
        response->message = "ERROR: Cannot start competition if current state is not READY";
        return true;
    }
    //==============================================================================
    void TaskManagerPlugin::ComputeTrialScore()
    {
        std::shared_ptr<ariac_common::KittingScore> kitting_score = nullptr;
        std::shared_ptr<ariac_common::AssemblyScore> assembly_score = nullptr;
        std::shared_ptr<ariac_common::CombinedScore> combined_score = nullptr;

        std::shared_ptr<ariac_common::Order> kitting_order = nullptr;
        std::shared_ptr<ariac_common::Order> assembly_order = nullptr;
        std::shared_ptr<ariac_common::Order> combined_order = nullptr;

        for (auto &order : impl_->all_orders_)
        {
            if (order->GetKittingScore())
            {
                kitting_order = order;
                kitting_score = order->GetKittingScore();
                impl_->trial_score_ += kitting_score->GetScore();
            }

            if (order->GetAssemblyScore())
            {
                assembly_order = order;
                assembly_score = order->GetAssemblyScore();
                impl_->trial_score_ += assembly_score->GetScore();
            }

            if (order->GetCombinedScore())
            {
                combined_order = order;
                combined_score = order->GetCombinedScore();
                impl_->trial_score_ += combined_score->GetScore();
            }
        }

        impl_->PrintTrialScore(kitting_order, assembly_order, combined_order);
    }

    //==============================================================================
    void TaskManagerPluginPrivate::PrintTrialScore(
        std::shared_ptr<ariac_common::Order> kitting_order,
        std::shared_ptr<ariac_common::Order> assembly_order, std::shared_ptr<ariac_common::Order> combined_order)
    {

        double kitting_submitted_time = 0.0;
        double assembly_submitted_time = 0.0;
        double combined_submitted_time = 0.0;

        double kitting_completion_time = 0.0;
        double assembly_completion_time = 0.0;
        double combined_completion_time = 0.0;

        if (kitting_order != nullptr)
        {
            kitting_submitted_time = kitting_order->GetSubmittedTime();
            kitting_completion_time = kitting_submitted_time - kitting_order->GetAnnouncedTime();
        }

        if (assembly_order != nullptr)
        {
            assembly_submitted_time = assembly_order->GetSubmittedTime();
            assembly_completion_time = assembly_submitted_time - assembly_order->GetAnnouncedTime();
        }

        if (combined_order != nullptr)
        {
            combined_submitted_time = combined_order->GetSubmittedTime();
            combined_completion_time = combined_submitted_time - combined_order->GetAnnouncedTime();
        }

        auto trial_completion_time = std::max({kitting_submitted_time, assembly_submitted_time, combined_submitted_time});

        std::string output = "";
        output += "\n\n\n\n========================================\n";
        output += "END OF TRIAL\n";
        output += "========================================\n";
        output += "Trial file: " + trial_name_ + "\n";
        output += "Trial time limit: " + std::to_string(time_limit_) + "\n";
        output += "Trial completion time: " + std::to_string(trial_completion_time) + "\n";
        output += "Trial score: " + std::to_string((int)trial_score_) + "\n";
        output += "========================================\n";
        output += "ORDERS\n";

        if (kitting_order != nullptr)
        {
            output += "========================================\n";
            output += "Kitting: " + kitting_order->GetId() + "\n";
            output += "Announcement time: " + std::to_string(kitting_order->GetAnnouncedTime()) + "\n";
            output += "Submission time: " + std::to_string(kitting_order->GetSubmittedTime()) + "\n";
            output += "Completion time: " + std::to_string(kitting_completion_time) + "\n";
            output += "Score: " + std::to_string(kitting_order->GetKittingScore()->GetScore()) + "\n";
        }

        if (assembly_order != nullptr)
        {
            output += "========================================\n";
            output += "Assembly: " + assembly_order->GetId() + "\n";
            output += "Announcement time: " + std::to_string(assembly_order->GetAnnouncedTime()) + "\n";
            output += "Submission time: " + std::to_string(assembly_order->GetSubmittedTime()) + "\n";
            output += "Completion time: " + std::to_string(assembly_completion_time) + "\n";
            output += "Score: " + std::to_string(assembly_order->GetAssemblyScore()->GetScore()) + "\n";
        }

        if (combined_order != nullptr)
        {
            output += "========================================\n";
            output += "Combined: " + combined_order->GetId() + "\n";
            output += "Announcement time: " + std::to_string(combined_order->GetAnnouncedTime()) + "\n";
            output += "Submission time: " + std::to_string(combined_order->GetSubmittedTime()) + "\n";
            output += "Completion time: " + std::to_string(combined_completion_time) + "\n";
            output += "Score: " + std::to_string(combined_order->GetCombinedScore()->GetScore()) + "\n";
        }

        RCLCPP_INFO_STREAM(ros_node_->get_logger(), output);
        log_message_ += output;
        WriteToLog();
    }

    //==============================================================================
    void TaskManagerPlugin::ProcessEndSafeZonePenalty()
    {
        if (impl_->safe_zone_penalty_started_)
        {
            // RCLCPP_INFO_STREAM(impl_->ros_node_->get_logger(), "Elapsed time: " << impl_->elapsed_time_);
            // RCLCPP_INFO_STREAM(impl_->ros_node_->get_logger(), "Penalty time: " << impl_->safe_zone_penalty_start_time_ + impl_->safe_zone_penalty_duration_);
            if (impl_->elapsed_time_ >= impl_->safe_zone_penalty_start_time_ + impl_->safe_zone_penalty_duration_)
            {
                impl_->safe_zone_penalty_started_ = false;
                impl_->ceiling_robot_health_ = true;
                // Track the end time of the safe zone penalty
                impl_->safe_zone_penalty_end_time_ = impl_->elapsed_time_;
            }
        }
    }

    //==============================================================================
    void TaskManagerPlugin::StartSafeZonePenalty()
    {
        // check if the ceiling robot is not just coming out of the penalty
        // give it a 10 second grace period

        if (impl_->safe_zone_penalty_end_time_ > 0.0) // penalty was previously ended
        {
            if (impl_->elapsed_time_ < impl_->safe_zone_penalty_end_time_ + impl_->ceiling_robot_grace_period_)
            {
                return;
            }
        }

        impl_->ceiling_robot_health_ = false;
        impl_->safe_zone_penalty_start_time_ = impl_->elapsed_time_;
        impl_->safe_zone_penalty_started_ = true;
    }

    //==============================================================================
    bool TaskManagerPlugin::SafeZonePenaltyServiceCallback(
        const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response)
    {
        std::lock_guard<std::mutex> lock(this->impl_->lock_);

        (void)request;

        response->success = true;
        response->message = "Safe zone penalty service called";

        // Function which handles the safe zone penalty
        StartSafeZonePenalty();

        return true;
    }
    //==============================================================================
    bool TaskManagerPlugin::EndCompetitionServiceCallback(
        const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response)
    {
        std::lock_guard<std::mutex> lock(this->impl_->lock_);

        (void)request;

        this->impl_->current_state_ = ariac_msgs::msg::CompetitionState::ENDED;

        response->success = true;
        response->message = "Competition ended successfully!";

        // Display the trial score
        ComputeTrialScore();

        return true;
    }

    //==============================================================================
    void TaskManagerPluginPrivate::PerformQualityCheck(
        ariac_msgs::srv::PerformQualityCheck::Request::SharedPtr request,
        ariac_msgs::srv::PerformQualityCheck::Response::SharedPtr response)
    {
        response->valid_id = false;
        int idx;

        // Check if order id matches any of the orders
        for (int i = 0; i < all_orders_.size(); i++)
        {
            if (request->order_id == all_orders_.at(i)->GetId())
            {
                // Check if that order is kitting
                if (all_orders_.at(i)->GetType() == ariac_msgs::msg::Order::KITTING)
                {
                    // Retrieve the kitting task
                    idx = i;
                    response->valid_id = true;
                    break;
                }
            }
        }

        if (!response->valid_id)
            return;

        auto task = all_orders_.at(idx)->GetKittingTask();

        // Instantiate a KittingShipment from the agv_tray_sensor data
        auto shipment = ParseAGVTraySensorImage(agv_tray_images_[task->GetAgvNumber()]);
        // RCLCPP_INFO(ros_node_->get_logger(), shipment.DebugString().c_str());

        if (task->GetTrayId() != shipment.GetTrayId())
            response->incorrect_tray = true;

        // Fill a quality msg for the shipment given a task
        response->quadrant1 = CheckQuadrantQuality(1, *task, shipment, request->order_id);
        response->quadrant2 = CheckQuadrantQuality(2, *task, shipment, request->order_id);
        response->quadrant3 = CheckQuadrantQuality(3, *task, shipment, request->order_id);
        response->quadrant4 = CheckQuadrantQuality(4, *task, shipment, request->order_id);

        if (response->quadrant1.all_passed &&
            response->quadrant2.all_passed &&
            response->quadrant3.all_passed &&
            response->quadrant4.all_passed)
        {

            response->all_passed = true;
        }
    }

    //==============================================================================
    void TaskManagerPluginPrivate::GetPreAssemblyPoses(
        ariac_msgs::srv::GetPreAssemblyPoses::Request::SharedPtr request,
        ariac_msgs::srv::GetPreAssemblyPoses::Response::SharedPtr response)
    {
        // Check if service has already been checked for requested order
        response->valid_id = false;

        std::shared_ptr<ariac_common::Order> order = nullptr;

        // Check if order id matches any of the orders
        for (int i = 0; i < all_orders_.size(); i++)
        {
            if (request->order_id == all_orders_.at(i)->GetId())
            {
                // Check if that order is assembly or combined
                if (all_orders_.at(i)->GetType() == ariac_msgs::msg::Order::ASSEMBLY ||
                    all_orders_.at(i)->GetType() == ariac_msgs::msg::Order::COMBINED)
                {
                    // Retrieve the kitting task
                    order = all_orders_.at(i);

                    if (order->WasPreAssemblyServiceCalled())
                    {
                        RCLCPP_WARN(ros_node_->get_logger(), "Cannot call service twice for the same order");
                        response->valid_id = false;
                        return;
                    }

                    response->valid_id = true;
                    break;
                }
            }
        }

        if (!response->valid_id)
            return;

        // Find what AGVs are at the assembly station for that order
        int order_station;
        if (order->GetType() == ariac_msgs::msg::Order::ASSEMBLY)
        {
            order_station = order->GetAssemblyTask()->GetStation();
        }
        else if (order->GetType() == ariac_msgs::msg::Order::COMBINED)
        {
            order_station = order->GetCombinedTask()->GetStation();
        }

        std::vector<int> agvs_to_check;
        if (order_station == ariac_msgs::msg::AssemblyTask::AS1)
        {
            if (agv1_location_ == ariac_msgs::msg::AGVStatus::ASSEMBLY_FRONT)
                agvs_to_check.push_back(1);
            if (agv2_location_ == ariac_msgs::msg::AGVStatus::ASSEMBLY_FRONT)
                agvs_to_check.push_back(2);
        }
        else if (order_station == ariac_msgs::msg::AssemblyTask::AS2)
        {
            if (agv1_location_ == ariac_msgs::msg::AGVStatus::ASSEMBLY_BACK)
                agvs_to_check.push_back(1);
            if (agv2_location_ == ariac_msgs::msg::AGVStatus::ASSEMBLY_BACK)
                agvs_to_check.push_back(2);
        }
        else if (order_station == ariac_msgs::msg::AssemblyTask::AS3)
        {
            if (agv3_location_ == ariac_msgs::msg::AGVStatus::ASSEMBLY_FRONT)
                agvs_to_check.push_back(3);
            if (agv4_location_ == ariac_msgs::msg::AGVStatus::ASSEMBLY_FRONT)
                agvs_to_check.push_back(4);
        }
        else if (order_station == ariac_msgs::msg::AssemblyTask::AS4)
        {
            if (agv3_location_ == ariac_msgs::msg::AGVStatus::ASSEMBLY_BACK)
                agvs_to_check.push_back(3);
            if (agv4_location_ == ariac_msgs::msg::AGVStatus::ASSEMBLY_BACK)
                agvs_to_check.push_back(4);
        }

        if (agvs_to_check.size() == 0)
        {
            response->agv_at_station = false;
            return;
        }
        else
        {
            order->SetPreAssemblyServiceCalled();
            response->agv_at_station = true;
        }

        // Get part poses on AGVs to check
        for (auto agv : agvs_to_check)
        {
            KDL::Frame world_to_sensor;
            tf2::fromMsg(gazebo_ros::Convert<geometry_msgs::msg::Pose>(
                             gazebo::msgs::ConvertIgn(agv_tray_images_[agv].pose())),
                         world_to_sensor);

            for (int i = 0; i < agv_tray_images_[agv].model_size(); i++)
            {
                const auto &lc_model = agv_tray_images_[agv].model(i);
                std::string model_name = lc_model.name();

                bool classified_part = false;
                // Determine part type
                for (const auto &part_type : part_types_)
                {
                    if (model_name.find(part_type.first) != std::string::npos)
                    {
                        // Determine part color
                        for (const auto &color : part_colors_)
                        {
                            if (model_name.find(color.first) != std::string::npos)
                            {
                                ariac_msgs::msg::PartPose part_pose;
                                part_pose.part.type = part_type.second;
                                part_pose.part.color = color.second;

                                KDL::Frame sensor_to_part;
                                tf2::fromMsg(gazebo_ros::Convert<geometry_msgs::msg::Pose>(
                                                 gazebo::msgs::ConvertIgn(lc_model.pose())),
                                             sensor_to_part);

                                part_pose.pose = tf2::toMsg(world_to_sensor * sensor_to_part);

                                response->parts.push_back(part_pose);
                                classified_part = true;
                                break;
                            }
                        }
                    }
                    if (classified_part)
                        break;
                }
            }
        }
    }

    //==============================================================================
    std::array<bool, 4> TaskManagerPluginPrivate::HandleFaultyParts(std::shared_ptr<ariac_common::FaultyPartChallenge> _challenge, ariac_common::KittingTask _task, ariac_common::KittingShipment _shipment)
    {
        // quadrant1, quadrant2, quadrant3, quadrant4
        std::array<bool, 4> faulty_parts = {false, false, false, false};

        for (int i = 1; i < 5; i++)
        {
            if (_challenge->IsQuadrantFaulty(i))
            {
                // RCLCPP_INFO_STREAM(ros_node_->get_logger(), "Quadrant " + std::to_string(i) + " is faulty");
                if (!_challenge->WasQuadrantChecked(i))
                {
                    // RCLCPP_INFO_STREAM(ros_node_->get_logger(), "Quadrant was not checked");
                    // Rename part in gazebo
                    for (auto &product : _task.GetProducts())
                    {
                        if (product.GetQuadrant() == i)
                        {
                            // RCLCPP_INFO_STREAM(ros_node_->get_logger(), "Quadrant: " << product.GetQuadrant());
                            for (auto tray_part : _shipment.GetTrayParts())
                            {
                                if (tray_part.GetQuadrant() == i)
                                {
                                    // RCLCPP_INFO_STREAM(ros_node_->get_logger(), "Quadrant " << i);
                                    if (tray_part.isCorrectType(product.GetPart().GetType()) &&
                                        tray_part.isCorrectColor(product.GetPart().GetColor()))
                                    {
                                        std::string original_name = tray_part.GetModelName();
                                        // RCLCPP_INFO_STREAM(ros_node_->get_logger(), "Changing name to (" + original_name + "_faulty)");
                                        world_->ModelByName(original_name)->SetName(original_name + "_faulty");
                                        _challenge->SetQuadrantChecked(i);

                                        // Set faulty part for the quadrant
                                        RCLCPP_INFO_STREAM(ros_node_->get_logger(), "Part in quadrant " << std::to_string(i) << " is faulty");
                                        faulty_parts[i - 1] = true;
                                    }
                                }
                            }
                        }
                    }
                }
                else
                {
                    for (auto tray_part : _shipment.GetTrayParts())
                    {
                        if (tray_part.GetQuadrant() == i)
                        {
                            faulty_parts[i - 1] = tray_part.isFaulty();
                        }
                    }
                }
            }
        }
        return faulty_parts;
    }

    //==============================================================================
    void TaskManagerPluginPrivate::StoreStationParts(int station, gazebo::msgs::LogicalCameraImage &_msg)
    {
        insert_parts_.find(station)->second.clear();
        int assembly_insert = -1;
        std::vector<ariac_common::Part> insert_parts;

        for (int i = 0; i < _msg.model_size(); i++)
        {
            const auto &lc_model = _msg.model(i);
            std::string model_name = lc_model.name();

            // name of the insert is "assembly_insert_XX" where XX indicates the id of the insert
            if (model_name.find("assembly_insert") != std::string::npos)
            {
                std::string id_string = model_name.substr(16, 1);
                assembly_insert = std::stoi(id_string);
                // RCLCPP_INFO_STREAM(ros_node_->get_logger(), "Assembly insert: " << id_string);
            }
        }
        // If no insert is detected, return
        if (assembly_insert == -1)
            return;

        // Fill vector of AssemblyParts
        for (int i = 0; i < _msg.model_size(); i++)
        {
            const auto &lc_model = _msg.model(i);
            std::string model_name = lc_model.name();

            bool classified_part = false;
            // Determine part type
            for (const auto &type : part_types_)
            {
                if (model_name.find(type.first) != std::string::npos)
                {
                    // Determine part color
                    for (const auto &color : part_colors_)
                    {
                        if (model_name.find(color.first) != std::string::npos)
                        {
                            ariac_common::Part part(color.second, type.second);
                            insert_parts.push_back(part);

                            classified_part = true;
                            break;
                        }
                    }
                }
                if (classified_part)
                    break;
            }
        }
        insert_parts_[assembly_insert] = insert_parts;
    }

    //==============================================================================
    void TaskManagerPluginPrivate::StoreAGVParts(int agv_id, gazebo::msgs::LogicalCameraImage &_msg)
    {
        agv_parts_.find(agv_id)->second.clear();
        std::vector<ariac_common::Part> kit_tray_parts;

        int kit_tray_id = -1;

        for (int i = 0; i < _msg.model_size(); i++)
        {
            const auto &lc_model = _msg.model(i);

            std::string model_name = lc_model.name();
            // name of kit tray model is "kit_tray_XX_YY" where XX indicates
            // the marker_id for the tray
            if (model_name.find("kit_tray") != std::string::npos)
            {
                std::string id_string = model_name.substr(9, 2);
                kit_tray_id = std::stoi(id_string);
            }
        }

        // If no kit tray is detected return
        if (kit_tray_id == -1)
            return;

        // Fill vector of KitTrayParts
        for (int i = 0; i < _msg.model_size(); i++)
        {
            const auto &lc_model = _msg.model(i);
            std::string model_name = lc_model.name();

            bool classified_part = false;
            // Determine part type
            for (const auto &type : part_types_)
            {
                if (model_name.find(type.first) != std::string::npos)
                {
                    // Determine part color
                    for (const auto &color : part_colors_)
                    {
                        if (model_name.find(color.first) != std::string::npos)
                        {
                            ariac_common::Part part(color.second, type.second);
                            kit_tray_parts.push_back(part);

                            classified_part = true;
                            break;
                        }
                    }
                }
                if (classified_part)
                    break;
            }
        }
        agv_parts_[agv_id] = kit_tray_parts;

        // if (agv_id == 4)
        // {
        //     if (agv_parts_[agv_id].size() > 0)
        //     {
        //         std::cout << "AGV4: " << std::endl;
        //         for (auto part : agv_parts_[agv_id])
        //         {
        //             std::cout << part.GetType() << " " << part.GetColor() << std::endl;
        //         }
        //     }
        // }
    }

    //==============================================================================
    ariac_common::AssemblyShipment TaskManagerPluginPrivate::ParseAssemblyStationImage(gazebo::msgs::LogicalCameraImage &_msg)
    {
        // Create an assembly shipment from data in the msg
        KDL::Frame sensor_to_insert;
        int insert_id = -1;
        geometry_msgs::msg::Pose insert_pose;
        std::vector<ariac_common::InsertPart> insert_parts;

        // find insert
        for (int i = 0; i < _msg.model_size(); i++)
        {
            const auto &lc_model = _msg.model(i);
            std::string model_name = lc_model.name();

            // name of the insert is "assembly_insert_XX" where XX indicates the id of the insert
            if (model_name.find("assembly_insert") != std::string::npos)
            {
                std::string id_string = model_name.substr(16, 1);
                insert_id = std::stoi(id_string);
                auto pose = gazebo_ros::Convert<geometry_msgs::msg::Pose>(gazebo::msgs::ConvertIgn(lc_model.pose()));

                tf2::fromMsg(pose, sensor_to_insert);
            }
        }
        // If no insert is detected return an empty shipment
        if (insert_id == -1)
            return ariac_common::AssemblyShipment(insert_id, insert_parts);

        // Fill vector of ariac_common::AssemblyPart
        for (int i = 0; i < _msg.model_size(); i++)
        {
            const auto &lc_model = _msg.model(i);
            std::string model_name = lc_model.name();

            bool classified_part = false;
            // Determine part type
            for (const auto &type : part_types_)
            {
                if (model_name.find(type.first) != std::string::npos)
                {
                    // Determine part color
                    for (const auto &color : part_colors_)
                    {
                        if (model_name.find(color.first) != std::string::npos)
                        {
                            ariac_common::Part part(color.second, type.second);

                            // Get pose in insert
                            KDL::Frame sensor_to_part;
                            tf2::fromMsg(gazebo_ros::Convert<geometry_msgs::msg::Pose>(
                                             gazebo::msgs::ConvertIgn(lc_model.pose())),
                                         sensor_to_part);

                            KDL::Frame insert_to_part;
                            insert_to_part = sensor_to_insert.Inverse() * sensor_to_part;

                            geometry_msgs::msg::Pose pose_in_insert = tf2::toMsg(insert_to_part);

                            // Create AssemblyPart
                            ariac_common::InsertPart insert_part(part, model_name, pose_in_insert);

                            insert_parts.push_back(insert_part);
                            classified_part = true;
                            break;
                        }
                    }
                }
                if (classified_part)
                    break;
            }
        }

        return ariac_common::AssemblyShipment(insert_id, insert_parts);
    }

    //==============================================================================
    ariac_common::KittingShipment
    TaskManagerPluginPrivate::ParseAGVTraySensorImage(gazebo::msgs::LogicalCameraImage &_msg)
    {
        // Create a kitting shipment from data in the msg
        KDL::Frame sensor_to_tray;
        int kit_tray_id = -1;
        geometry_msgs::msg::Pose kit_tray_pose;
        std::vector<ariac_common::KitTrayPart> tray_parts;

        // Find Kit Tray
        for (int i = 0; i < _msg.model_size(); i++)
        {
            const auto &lc_model = _msg.model(i);

            std::string model_name = lc_model.name();
            // name of kit tray model is "kit_tray_XX_YY" where XX indicates
            // the marker_id for the tray
            if (model_name.find("kit_tray") != std::string::npos)
            {

                std::string id_string = model_name.substr(9, 2);
                kit_tray_id = std::stoi(id_string);
                auto pose = gazebo_ros::Convert<geometry_msgs::msg::Pose>(gazebo::msgs::ConvertIgn(lc_model.pose()));

                tf2::fromMsg(pose, sensor_to_tray);
            }
        }

        // If no kit tray is detected return an empty shipment
        if (kit_tray_id == -1)
            return ariac_common::KittingShipment(kit_tray_id, tray_parts);

        // Fill vector of KitTrayParts
        for (int i = 0; i < _msg.model_size(); i++)
        {
            const auto &lc_model = _msg.model(i);
            std::string model_name = lc_model.name();

            bool classified_part = false;
            // Determine part type
            for (const auto &type : part_types_)
            {
                if (model_name.find(type.first) != std::string::npos)
                {
                    // Determine part color
                    for (const auto &color : part_colors_)
                    {
                        if (model_name.find(color.first) != std::string::npos)
                        {
                            ariac_common::Part part(color.second, type.second);

                            // Get pose on tray
                            KDL::Frame sensor_to_part;
                            tf2::fromMsg(gazebo_ros::Convert<geometry_msgs::msg::Pose>(
                                             gazebo::msgs::ConvertIgn(lc_model.pose())),
                                         sensor_to_part);

                            KDL::Frame tray_to_part;
                            tray_to_part = sensor_to_tray.Inverse() * sensor_to_part;

                            geometry_msgs::msg::Pose pose_on_tray = tf2::toMsg(tray_to_part);

                            // Create KitTrayPart
                            ariac_common::KitTrayPart tray_part(part, model_name, pose_on_tray);

                            tray_parts.push_back(tray_part);
                            classified_part = true;
                            break;
                        }
                    }
                }
                if (classified_part)
                    break;
            }
        }

        return ariac_common::KittingShipment(kit_tray_id, tray_parts);
    }

    //==============================================================================
    ariac_msgs::msg::QualityIssue TaskManagerPluginPrivate::CheckQuadrantQuality(int quadrant,
                                                                                 ariac_common::KittingTask task,
                                                                                 ariac_common::KittingShipment shipment,
                                                                                 std::string order_id)
    {
        ariac_msgs::msg::QualityIssue issue;

        bool task_has_part_in_quadrant = false;

        // // Check if the order has a faulty part challenge
        std::array<bool, 4> faulty_parts = {false, false, false, false};
        if (faulty_part_challenges_.size() > 0)
        {
            for (auto &challenge : faulty_part_challenges_)
            {
                if (challenge->GetOrderId() == order_id) // order has a faulty part challenge
                {
                    faulty_parts = HandleFaultyParts(challenge, task, shipment);
                }
            }
        }

        for (auto product : task.GetProducts())
        {
            if (product.GetQuadrant() == quadrant)
            {
                task_has_part_in_quadrant = true;
                bool shipment_has_part_in_quadrant;
                for (auto tray_part : shipment.GetTrayParts())
                {
                    if (tray_part.GetQuadrant() == quadrant)
                    {
                        shipment_has_part_in_quadrant = true;

                        issue.incorrect_part_type = !tray_part.isCorrectType(product.GetPart().GetType());
                        issue.incorrect_part_color = !tray_part.isCorrectColor(product.GetPart().GetColor());
                        issue.faulty_part = faulty_parts[quadrant - 1];
                        issue.flipped_part = tray_part.isFlipped();
                    }
                }

                if (!shipment_has_part_in_quadrant)
                    issue.missing_part = true;
            }
        }

        if (issue.missing_part || issue.incorrect_part_type || issue.incorrect_part_color || issue.faulty_part || issue.flipped_part)
            issue.all_passed = false;
        else
            issue.all_passed = true;

        return issue;
    }

    // Register this plugin with the simulator
    GZ_REGISTER_WORLD_PLUGIN(TaskManagerPlugin)
} // namespace ariac_plugins