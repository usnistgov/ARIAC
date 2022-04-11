/*
 * Copyright 2016 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/
#include "ROSAGVPlugin.hh"

#include <gazebo/common/common.hh>
#include <gazebo/common/UpdateInfo.hh>
#include <gazebo/common/Time.hh>
#include <gazebo/transport/transport.hh>
#include <ignition/math.hh>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
#include <std_srvs/Trigger.h>
#include <mutex>
#include <string>
#include <iostream>
#include <cstring>

namespace gazebo {
    /// \internal
    /// \brief Private data for the ROSAGVPlugin class.
    struct ROSAGVPluginPrivate
    {
        /// \brief Name of the AGV
    public:
        std::string agv_name;
        std::mutex mutex;

        /// \brief Name of the assembly station
        std::string assembly_station_name;

        /// \brief Name of the kitting station
        std::string kitting_station_name;

        /// \brief Scoped name of the link of the tray on the AGV
        std::string tray_link_name;

        /// \brief World pointer
        physics::WorldPtr world_ptr;

        /// \brief Pointer to the update event connection
        event::ConnectionPtr update_connection_ptr;

        /// \brief for setting ROS name space
        std::string robot_namespace;

        /// \brief ros node handle
        ros::NodeHandle* ros_node;

        /// \brief Receives service calls for controlling the AGV
        // ros::ServiceServer rosService;
        ros::ServiceServer ks_to_as1_service;
        ros::ServiceServer ks_to_as2_service;
        ros::ServiceServer as1_to_ks_service;
        ros::ServiceServer as2_to_ks_service;
        ros::ServiceServer as1_to_as2_service;
        ros::ServiceServer as2_to_as1_service;

        ros::ServiceServer ks_to_as3_service;
        ros::ServiceServer ks_to_as4_service;
        ros::ServiceServer as3_to_ks_service;
        ros::ServiceServer as4_to_ks_service;
        ros::ServiceServer as3_to_as4_service;
        ros::ServiceServer as4_to_as3_service;
        // ros::ServiceServer rosServiceKitting;

        /// \brief Transportation node.
        transport::NodePtr gzNode;

        /// \brief Gazebo publish for locking/unlocking parts to this AGV's tray
        // transport::PublisherPtr tray_lock_unlock_publisher;
        transport::PublisherPtr lock_unlock_models_gz_pub;

        /// \brief Client for clearing this AGV's tray
        // ros::ServiceClient rosClearTrayClient;
        ros::ServiceClient rosAGVToAssemblyClient;

        //! \brief pose animation pointer for AGV1 going from as1 to ks1
        gazebo::common::PoseAnimationPtr agv1_from_as1_to_ks1_animation_ptr;
        //! \brief pose animation pointer for AGV1 going from as2 to ks1
        gazebo::common::PoseAnimationPtr agv1_from_as2_to_ks1_animation_ptr;
        //! \brief pose animation pointer for AGV1 going from as1 to as2
        gazebo::common::PoseAnimationPtr agv1_from_as1_to_as2_animation_ptr;
        //! \brief pose animation pointer for AGV1 going from as2 to as1
        gazebo::common::PoseAnimationPtr agv1_from_as2_to_as1_animation_ptr;
        //! \brief pose animation pointer for AGV1 going from ks1 to as1
        gazebo::common::PoseAnimationPtr agv1_from_ks1_to_as1_animation_ptr;
        //! \brief pose animation pointer for AGV1 going from ks1 to as2
        gazebo::common::PoseAnimationPtr agv1_from_ks1_to_as2_animation_ptr;

        //! \brief pose animation pointer for AGV1 going from as1 to ks2
        gazebo::common::PoseAnimationPtr agv2_from_as1_to_ks2_animation_ptr;
        //! \brief pose animation pointer for AGV1 going from as2 to ks2
        gazebo::common::PoseAnimationPtr agv2_from_as2_to_ks2_animation_ptr;
        //! \brief pose animation pointer for AGV1 going from as1 to as2
        gazebo::common::PoseAnimationPtr agv2_from_as1_to_as2_animation_ptr;
        //! \brief pose animation pointer for AGV1 going from as2 to as1
        gazebo::common::PoseAnimationPtr agv2_from_as2_to_as1_animation_ptr;
        //! \brief pose animation pointer for AGV1 going from ks2 to as1
        gazebo::common::PoseAnimationPtr agv2_from_ks2_to_as1_animation_ptr;
        //! \brief pose animation pointer for AGV1 going from ks2 to as2
        gazebo::common::PoseAnimationPtr agv2_from_ks2_to_as2_animation_ptr;

        //! \brief pose animation pointer for AGV3 going from as3 to ks3
        gazebo::common::PoseAnimationPtr agv3_from_as3_to_ks3_animation_ptr;
        //! \brief pose animation pointer for AGV1 going from as4 to ks3
        gazebo::common::PoseAnimationPtr agv3_from_as4_to_ks3_animation_ptr;
        //! \brief pose animation pointer for AGV1 going from as3 to as4
        gazebo::common::PoseAnimationPtr agv3_from_as3_to_as4_animation_ptr;
        //! \brief pose animation pointer for AGV1 going from as4 to as3
        gazebo::common::PoseAnimationPtr agv3_from_as4_to_as3_animation_ptr;
        //! \brief pose animation pointer for AGV1 going from ks3 to as3
        gazebo::common::PoseAnimationPtr agv3_from_ks3_to_as3_animation_ptr;
        //! \brief pose animation pointer for AGV1 going from ks3 to as4
        gazebo::common::PoseAnimationPtr agv3_from_ks3_to_as4_animation_ptr;

        //! \brief pose animation pointer for AGV4 going from as3 to ks4
        gazebo::common::PoseAnimationPtr agv4_from_as3_to_ks4_animation_ptr;
        //! \brief pose animation pointer for AGV4 going from as4 to ks4
        gazebo::common::PoseAnimationPtr agv4_from_as4_to_ks4_animation_ptr;
        //! \brief pose animation pointer for AGV4 going from as3 to as4
        gazebo::common::PoseAnimationPtr agv4_from_as3_to_as4_animation_ptr;
        //! \brief pose animation pointer for AGV4 going from as4 to as3
        gazebo::common::PoseAnimationPtr agv4_from_as4_to_as3_animation_ptr;
        //! \brief pose animation pointer for AGV4 going from ks4 to as3
        gazebo::common::PoseAnimationPtr agv4_from_ks4_to_as3_animation_ptr;
        //! \brief pose animation pointer for AGV4 going from ks4 to as4
        gazebo::common::PoseAnimationPtr agv4_from_ks4_to_as4_animation_ptr;

        /// \brief Pointer to the model
        gazebo::physics::ModelPtr model;

        /// \brief The state of the AGV
        std::string current_state;

        /// \brief The time the last tray delivery was triggered
        // common::Time delivery_trigger_time; //currently not being used

        common::Time goto_as_trigger_time;
        common::Time goto_ks_trigger_time;

        /// \brief Whether or not gravity of the AGV has been disabled
        // bool gravity_disabled; //currently not being used

        /// \brief Flag for triggering tray delivery from the service callback
        // bool delivery_triggered = false; //currently not being used

        // bool goto_as_triggered = false;
        bool goto_ks_triggered;
        bool goto_as1_triggered;
        bool goto_as2_triggered;
        bool goto_as3_triggered;
        bool goto_as4_triggered;

        double agv_delay_move_time{ 3.0 };

        /// \brief Publishes the AGV state.
        ros::Publisher agv_state_publisher;

        /**
         * @brief Publishes the status of as2
         *
         * This is used by the ROSPersonByStation plugin.
         * The person will move away from as2 when there is an agv at as2
         */
        ros::Publisher as2StatusPublisher;
        /**
         * @brief Publishes the status of as4
         *
         * This is used by the ROSPersonByStation plugin.
         * The person will move away from as4 when there is an agv at as4
         */
        ros::Publisher as4StatusPublisher;
        /// \brief Publishes the station where the AGV is located
        ros::Publisher agv_station_publisher;
        // Subscriber for agv location
        ros::Subscriber agv_location_subscriber;
        std::string current_station;
    };
}  // namespace gazebo

using namespace gazebo;

// register the plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(ROSAGVPlugin);

/**
 * @brief Construct a new ROSAGVPlugin::ROSAGVPlugin object
 *
 */
ROSAGVPlugin::ROSAGVPlugin()
    : dataPtr(new ROSAGVPluginPrivate)
{
}

/**
 * @brief Destroy the ROSAGVPlugin::ROSAGVPlugin object
 *
 */
ROSAGVPlugin::~ROSAGVPlugin()
{
    this->dataPtr->ros_node->shutdown();
}

/**
 * @brief Receives an SDF element that contains the elements and attributes specified in loaded SDF file.
 *
 * @param _parent Pointer to the model
 * @param _sdf Pointer to the SDF elements
 */
void ROSAGVPlugin::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
{
    this->dataPtr->ros_node = new ros::NodeHandle(this->dataPtr->robot_namespace);

    // Initialize Gazebo transport
    this->dataPtr->gzNode = transport::NodePtr(new transport::Node());
    this->dataPtr->gzNode->Init();

    // Make sure the ROS node for Gazebo has already been initialized
    if (!ros::isInitialized()) {
        ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized,"
            << "unable to load plugin. Load the Gazebo system plugin "
            << "'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
        return;
    }

    std::string index{};
    std::string agv_station_topic_name{};
    std::string ks_to_as1_service_name{};
    std::string ks_to_as2_service_name{};
    std::string as1_to_ks_service_name{};
    std::string as2_to_ks_service_name{};
    std::string as1_to_as2_service_name{};
    std::string as2_to_as1_service_name{};
    std::string ks_to_as3_service_name{};
    std::string ks_to_as4_service_name{};
    std::string as3_to_ks_service_name{};
    std::string as4_to_ks_service_name{};
    std::string as3_to_as4_service_name{};
    std::string as4_to_as3_service_name{};

    this->dataPtr->goto_ks_triggered = false;
    this->dataPtr->goto_as1_triggered = false;
    this->dataPtr->goto_as2_triggered = false;
    this->dataPtr->goto_as3_triggered = false;
    this->dataPtr->goto_as4_triggered = false;


    std::string go_to_assembly_station_service{ "to_assembly_station" };
    // time the AGV takes to move from ks to as1 or to as3
    double move_time_ks_to_as_1_3{};
    // time the AGV takes to move from ks to as2 or to as4
    double move_time_ks_to_as_2_4{};
    // time the AGV takes to move between as (as1<->as2 and as3<->as4)
    double move_time_as_to_as{};
    // time the AGV takes to move between as1/as3 to ks1/ks2/ks3/ks4
    double move_time_as_1_3_to_ks{};
    // time the AGV takes to move between as2/as4 to ks1/ks2/ks3/ks4
    double move_time_as_2_4_to_ks{};


    this->dataPtr->world_ptr = _parent->GetWorld();

    // load parameters
    if (_sdf->HasElement("index"))
        index = _sdf->Get<std::string>("index");
    else {
        ROS_FATAL("AGV is missing an index. The AGV will not work.");
        return;
    }

    // agv name
    this->dataPtr->agv_name = std::string("agv") + index;

    // robot namespace
    this->dataPtr->robot_namespace = "";
    if (_sdf->HasElement("robotNamespace")) {
        this->dataPtr->robot_namespace = _sdf->GetElement("robotNamespace")->Get<std::string>() + "/";
        ROS_WARN_STREAM("robotNamespace: " << this->dataPtr->robot_namespace << "\n");
    }

  

    // tray link name
    this->dataPtr->tray_link_name =
        this->dataPtr->agv_name + "::kit_tray_" + index + "::kit_tray_" + index + "::tray";

    // topics to store the current location of AGVs
    if (_sdf->HasElement("agv_location_topic_name"))
        agv_station_topic_name = _sdf->Get<std::string>("agv_location_topic_name");
    else {
        ROS_FATAL("Missing agv_location_topic_name in ariac.world.template ");
        return;
    }

    if (_sdf->HasElement("agv_location_topic_name"))
        agv_station_topic_name = _sdf->Get<std::string>("agv_location_topic_name");
    else {
        ROS_FATAL("Missing agv_location_topic_name in ariac.world.template ");
        return;
    }

    if (_sdf->HasElement("ks_to_as1_name"))
        ks_to_as1_service_name = _sdf->Get<std::string>("ks_to_as1_name");

    if (_sdf->HasElement("ks_to_as2_name"))
        ks_to_as2_service_name = _sdf->Get<std::string>("ks_to_as2_name");

    if (_sdf->HasElement("as1_to_ks_name"))
        as1_to_ks_service_name = _sdf->Get<std::string>("as1_to_ks_name");

    if (_sdf->HasElement("as2_to_ks_name"))
        as2_to_ks_service_name = _sdf->Get<std::string>("as2_to_ks_name");

    if (_sdf->HasElement("as1_to_as2_name"))
        as1_to_as2_service_name = _sdf->Get<std::string>("as1_to_as2_name");

    if (_sdf->HasElement("as2_to_as1_name"))
        as2_to_as1_service_name = _sdf->Get<std::string>("as2_to_as1_name");

    if (_sdf->HasElement("ks_to_as3_name"))
        ks_to_as3_service_name = _sdf->Get<std::string>("ks_to_as3_name");

    if (_sdf->HasElement("ks_to_as4_name"))
        ks_to_as4_service_name = _sdf->Get<std::string>("ks_to_as4_name");

    if (_sdf->HasElement("as3_to_ks_name"))
        as3_to_ks_service_name = _sdf->Get<std::string>("as3_to_ks_name");

    if (_sdf->HasElement("as4_to_ks_name"))
        as4_to_ks_service_name = _sdf->Get<std::string>("as4_to_ks_name");

    if (_sdf->HasElement("as3_to_as4_name"))
        as3_to_as4_service_name = _sdf->Get<std::string>("as3_to_as4_name");

    if (_sdf->HasElement("as4_to_as3_name"))
        as4_to_as3_service_name = _sdf->Get<std::string>("as4_to_as3_name");


    if (_sdf->HasElement("move_time_ks_to_as_1_3")) {
        move_time_ks_to_as_1_3 = _sdf->Get<double>("move_time_ks_to_as_1_3");
        move_time_as_1_3_to_ks = move_time_ks_to_as_1_3;
    }
    else {
        ROS_FATAL("Missing move_time_ks_to_as_1_3 in ariac.world.template ");
        return;
    }

    if (_sdf->HasElement("move_time_ks_to_as_2_4")) {
        move_time_ks_to_as_2_4 = _sdf->Get<double>("move_time_ks_to_as_2_4");
        move_time_as_2_4_to_ks = move_time_ks_to_as_2_4;
    }
    else {
        ROS_FATAL("Missing move_time_ks_to_as_2_4 in ariac.world.template ");
        return;
    }

    if (_sdf->HasElement("move_time_as_to_as"))
        move_time_as_to_as = _sdf->Get<double>("move_time_as_to_as");
    else {
        ROS_FATAL("Missing move_time_as_to_as in ariac.world.template ");
        return;
    }

    // //TODO Get rid of this one
    // std::string lock_unlock_movable_tray_on_agv{};
    // if (_sdf->HasElement("lock_unlock_movable_tray_on_agv"))
    //     lock_unlock_movable_tray_on_agv = _sdf->Get<std::string>("lock_unlock_movable_tray_on_agv");

    // rosservice call /ariac/lock_unlock_models kit_tray_1

    std::string lock_unlock_gz{};
    if (index == "1") {
        if (_sdf->HasElement("lock_unlock_kt1_topic"))
            lock_unlock_gz = _sdf->Get<std::string>("lock_unlock_kt1_topic");
        this->dataPtr->lock_unlock_models_gz_pub =
            this->dataPtr->gzNode->Advertise<msgs::GzString>(lock_unlock_gz);
    }
    else if (index == "2") {
        if (_sdf->HasElement("lock_unlock_kt2_topic"))
            lock_unlock_gz = _sdf->Get<std::string>("lock_unlock_kt1_topic");
        this->dataPtr->lock_unlock_models_gz_pub =
            this->dataPtr->gzNode->Advertise<msgs::GzString>(lock_unlock_gz);
    }
    else if (index == "3") {
        if (_sdf->HasElement("lock_unlock_kt3_topic"))
            lock_unlock_gz = _sdf->Get<std::string>("lock_unlock_kt1_topic");
        this->dataPtr->lock_unlock_models_gz_pub =
            this->dataPtr->gzNode->Advertise<msgs::GzString>(lock_unlock_gz);
    }
    else if (index == "4") {
        if (_sdf->HasElement("lock_unlock_kt4_topic"))
            lock_unlock_gz = _sdf->Get<std::string>("lock_unlock_kt1_topic");
        this->dataPtr->lock_unlock_models_gz_pub =
            this->dataPtr->gzNode->Advertise<msgs::GzString>(lock_unlock_gz);
    }
    

    //TODO
    //ARIAC2022 changes
    // std::string lock_unlock_parts_on_movable_tray{};
    // if (_sdf->HasElement("lock_unlock_models_on_movable_tray"))
    //     lock_unlock_parts_on_movable_tray = _sdf->Get<std::string>("lock_unlock_models_on_movable_tray");



    if (_sdf->HasElement("agv_to_as_service_name"))
        go_to_assembly_station_service = _sdf->Get<std::string>("agv_to_as_service_name");

    

    // ROS Subscriber
    this->dataPtr->agv_location_subscriber =
        this->dataPtr->ros_node->subscribe(agv_station_topic_name, 1000, &ROSAGVPlugin::OnAGVLocation, this);

    // // Gazebo Publisher
    // this->dataPtr->lock_unlock_movable_tray_on_agv_publisher =
    //     this->dataPtr->gzNode->Advertise<msgs::GzString>(lock_unlock_movable_tray_on_agv);

    double speed_factor{ 1.5 };

    // All 4 AGVs have the same y position and the same rpy during animations
    float yaw = -1.570796;
    float height = -0.02f;
    double current_x{};
    double current_y{};
    // length (in meter) between ks1-as1, ks2-as1, ks3-as3, ks4-as3
    float length_ks_as_1_3{ 3.334315 };
    // length (in meter) between ks and as2 or as4
    float length_ks_as_2_4{ 8.324589 };
    // length (in meter) between as1 and as2 and between as43 and as4
    float length_as_as{ 4.990274 };

    /*
    * ======================================
    * AGV1: From KS1 to AS1
    * ======================================
    */
    current_x = -2.265685;
    current_y = 4.675404;

    this->dataPtr->agv1_from_ks1_to_as1_animation_ptr.reset(
        new gazebo::common::PoseAnimation(this->dataPtr->agv_name, move_time_as_1_3_to_ks / speed_factor, false));
    gazebo::common::PoseKeyFrame* key = this->dataPtr->agv1_from_ks1_to_as1_animation_ptr->CreateKeyFrame(0);
    for (int i = 1; i < 11; i++) {
        key = this->dataPtr->agv1_from_ks1_to_as1_animation_ptr->CreateKeyFrame(i / 10 * move_time_as_1_3_to_ks / speed_factor);
        key->Translation(ignition::math::Vector3d(current_x - i / 10 * length_ks_as_1_3, current_y, height));
        key->Rotation(ignition::math::Quaterniond(0, 0, yaw));
    }

    /*
    * ======================================
    * AGV1: From KS1 to AS2
    * ======================================
    */
    current_x = -2.265685;
    current_y = 4.675404;
    this->dataPtr->agv1_from_ks1_to_as2_animation_ptr.reset(
        new gazebo::common::PoseAnimation(this->dataPtr->agv_name, move_time_as_2_4_to_ks / speed_factor, false));
    key = this->dataPtr->agv1_from_ks1_to_as2_animation_ptr->CreateKeyFrame(0);

    for (int i = 1; i < 11; i++) {
        key = this->dataPtr->agv1_from_ks1_to_as2_animation_ptr->CreateKeyFrame(i / 10 * move_time_as_2_4_to_ks / speed_factor);
        key->Translation(ignition::math::Vector3d(current_x - i / 10 * length_ks_as_2_4, current_y, height));
        key->Rotation(ignition::math::Quaterniond(0, 0, yaw));
    }

    /*
    * ======================================
    * AGV1: From AS1 to AS2
    * ======================================
    */
    current_x = -5.600000;
    current_y = 4.675404;
    this->dataPtr->agv1_from_as1_to_as2_animation_ptr.reset(
        new gazebo::common::PoseAnimation(this->dataPtr->agv_name, move_time_as_to_as / speed_factor, false));
    key = this->dataPtr->agv1_from_as1_to_as2_animation_ptr->CreateKeyFrame(0);
    for (int i = 1; i < 11; i++) {
        key = this->dataPtr->agv1_from_as1_to_as2_animation_ptr->CreateKeyFrame(i / 10 * move_time_as_to_as / speed_factor);
        key->Translation(ignition::math::Vector3d(current_x - i / 10 * length_as_as, current_y, height));
        key->Rotation(ignition::math::Quaterniond(0, 0, yaw));
    }

    /*
    * ======================================
    * AGV1: From AS2 to AS1
    * ======================================
    */

    current_x = -10.590274;
    current_y = 4.675404;
    this->dataPtr->agv1_from_as2_to_as1_animation_ptr.reset(
        new gazebo::common::PoseAnimation(this->dataPtr->agv_name, move_time_as_to_as / speed_factor, false));
    key = this->dataPtr->agv1_from_as2_to_as1_animation_ptr->CreateKeyFrame(0);
    for (int i = 1; i < 11; i++) {
        key = this->dataPtr->agv1_from_as2_to_as1_animation_ptr->CreateKeyFrame(i / 10 * move_time_as_to_as / speed_factor);
        key->Translation(ignition::math::Vector3d(current_x + i / 10 * length_as_as, current_y, height));
        key->Rotation(ignition::math::Quaterniond(0, 0, yaw));
    }

    /*
      * ======================================
      * AGV1: From AS1 to KS1
      * ======================================
      */

    current_x = -5.6;
    current_y = 4.675404;
    this->dataPtr->agv1_from_as1_to_ks1_animation_ptr.reset(
        new gazebo::common::PoseAnimation(this->dataPtr->agv_name, move_time_as_1_3_to_ks / speed_factor, false));
    key = this->dataPtr->agv1_from_as1_to_ks1_animation_ptr->CreateKeyFrame(0);
    for (int i = 1; i < 11; i++) {
        key = this->dataPtr->agv1_from_as1_to_ks1_animation_ptr->CreateKeyFrame(i / 10 * move_time_as_1_3_to_ks / speed_factor);
        key->Translation(ignition::math::Vector3d(current_x + i / 10 * length_ks_as_1_3, current_y, height));
        key->Rotation(ignition::math::Quaterniond(0, 0, yaw));
    }

    /*
      * ======================================
      * AGV1: From AS2 to KS1
      * ======================================
      */

    current_x = -10.590274;
    current_y = 4.675404;
    this->dataPtr->agv1_from_as2_to_ks1_animation_ptr.reset(
        new gazebo::common::PoseAnimation(this->dataPtr->agv_name, move_time_as_2_4_to_ks / speed_factor, false));
    key = this->dataPtr->agv1_from_as2_to_ks1_animation_ptr->CreateKeyFrame(0);
    for (int i = 1; i < 11; i++) {
        key = this->dataPtr->agv1_from_as2_to_ks1_animation_ptr->CreateKeyFrame(i / 10 * move_time_as_2_4_to_ks / speed_factor);
        key->Translation(ignition::math::Vector3d(current_x + i / 10 * length_ks_as_2_4, current_y, height));
        key->Rotation(ignition::math::Quaterniond(0, 0, yaw));
    }


    /*
    * ======================================
    * AGV2: From KS2 to AS1
    * ======================================
    */
    current_x = -2.265685;
    current_y = 1.367643;
    this->dataPtr->agv2_from_ks2_to_as1_animation_ptr.reset(
        new gazebo::common::PoseAnimation(this->dataPtr->agv_name, move_time_as_1_3_to_ks / speed_factor, false));
    key = this->dataPtr->agv2_from_ks2_to_as1_animation_ptr->CreateKeyFrame(0);
    for (int i = 1; i < 11; i++) {
        key = this->dataPtr->agv2_from_ks2_to_as1_animation_ptr->CreateKeyFrame(i / 10 * move_time_as_1_3_to_ks / speed_factor);
        key->Translation(ignition::math::Vector3d(current_x - i / 10 * length_ks_as_1_3, current_y, height));
        key->Rotation(ignition::math::Quaterniond(0, 0, yaw));
    }

    /*
    * ======================================
    * AGV2: From KS2 to AS2
    * ======================================
    */
    current_x = -2.265685;
    current_y = 1.367643;
    this->dataPtr->agv2_from_ks2_to_as2_animation_ptr.reset(
        new gazebo::common::PoseAnimation(this->dataPtr->agv_name, move_time_as_2_4_to_ks / speed_factor, false));
    key = this->dataPtr->agv2_from_ks2_to_as2_animation_ptr->CreateKeyFrame(0);
    for (int i = 1; i < 11; i++) {
        key = this->dataPtr->agv2_from_ks2_to_as2_animation_ptr->CreateKeyFrame(i / 10 * move_time_as_2_4_to_ks / speed_factor);
        key->Translation(ignition::math::Vector3d(current_x - i / 10 * length_ks_as_2_4, current_y, height));
        key->Rotation(ignition::math::Quaterniond(0, 0, yaw));
    }

    /*
    * ======================================
    * AGV2: From AS1 to AS2
    * ======================================
    */
    current_x = -5.6;
    current_y = 1.367643;
    this->dataPtr->agv2_from_as1_to_as2_animation_ptr.reset(
        new gazebo::common::PoseAnimation(this->dataPtr->agv_name, move_time_as_to_as / speed_factor, false));
    key = this->dataPtr->agv2_from_as1_to_as2_animation_ptr->CreateKeyFrame(0);
    for (int i = 1; i < 11; i++) {
        key = this->dataPtr->agv2_from_as1_to_as2_animation_ptr->CreateKeyFrame(i / 10 * move_time_as_to_as / speed_factor);
        key->Translation(ignition::math::Vector3d(current_x - i / 10 * length_as_as, current_y, height));
        key->Rotation(ignition::math::Quaterniond(0, 0, yaw));
    }

    /*
    * ======================================
    * AGV2: From AS2 to AS1
    * ======================================
    */

    current_x = -10.590274;
    current_y = 1.367643;
    this->dataPtr->agv2_from_as2_to_as1_animation_ptr.reset(
        new gazebo::common::PoseAnimation(this->dataPtr->agv_name, move_time_as_to_as / speed_factor, false));
    key = this->dataPtr->agv2_from_as2_to_as1_animation_ptr->CreateKeyFrame(0);
    for (int i = 1; i < 11; i++) {
        key = this->dataPtr->agv2_from_as2_to_as1_animation_ptr->CreateKeyFrame(i / 10 * move_time_as_to_as / speed_factor);
        key->Translation(ignition::math::Vector3d(current_x + i / 10 * length_as_as, current_y, height));
        key->Rotation(ignition::math::Quaterniond(0, 0, yaw));
    }

    /*
      * ======================================
      * AGV2: From AS1 to KS2
      * =====================================
      */

    current_x = -5.6;
    current_y = 1.367643;
    this->dataPtr->agv2_from_as1_to_ks2_animation_ptr.reset(
        new gazebo::common::PoseAnimation(this->dataPtr->agv_name, move_time_as_1_3_to_ks / speed_factor, false));
    key = this->dataPtr->agv2_from_as1_to_ks2_animation_ptr->CreateKeyFrame(0);
    for (int i = 1; i < 11; i++) {
        key = this->dataPtr->agv2_from_as1_to_ks2_animation_ptr->CreateKeyFrame(i / 10 * move_time_as_1_3_to_ks / speed_factor);
        key->Translation(ignition::math::Vector3d(current_x + i / 10 * length_ks_as_1_3, current_y, height));
        key->Rotation(ignition::math::Quaterniond(0, 0, yaw));
    }

    /*
      * ======================================
      * AGV2: From AS2 to KS2
      * ======================================
      */

    current_x = -10.590274;
    current_y = 1.367643;
    this->dataPtr->agv2_from_as2_to_ks2_animation_ptr.reset(
        new gazebo::common::PoseAnimation(this->dataPtr->agv_name, move_time_as_2_4_to_ks / speed_factor, false));
    key = this->dataPtr->agv2_from_as2_to_ks2_animation_ptr->CreateKeyFrame(0);
    for (int i = 1; i < 11; i++) {
        key = this->dataPtr->agv2_from_as2_to_ks2_animation_ptr->CreateKeyFrame(i / 10 * move_time_as_2_4_to_ks / speed_factor);
        key->Translation(ignition::math::Vector3d(current_x + i / 10 * length_ks_as_2_4, current_y, height));
        key->Rotation(ignition::math::Quaterniond(0, 0, yaw));
    }

    ////////////////////////////////
    /*
    * ======================================
    * AGV3: From KS3 to AS3
    * ======================================
    */
    current_x = -2.265685;
    current_y = -1.333917;
    this->dataPtr->agv3_from_ks3_to_as3_animation_ptr.reset(
        new gazebo::common::PoseAnimation(this->dataPtr->agv_name, move_time_as_1_3_to_ks / speed_factor, false));
    key = this->dataPtr->agv3_from_ks3_to_as3_animation_ptr->CreateKeyFrame(0);
    for (int i = 1; i < 11; i++) {
        key = this->dataPtr->agv3_from_ks3_to_as3_animation_ptr->CreateKeyFrame(i / 10 * move_time_as_1_3_to_ks / speed_factor);
        key->Translation(ignition::math::Vector3d(current_x - i / 10 * length_ks_as_1_3, current_y, height));
        key->Rotation(ignition::math::Quaterniond(0, 0, yaw));
    }

    /*
    * ======================================
    * AGV3: From KS3 to AS4
    * ======================================
    */
    current_x = -2.265685;
    current_y = -1.333917;
    this->dataPtr->agv3_from_ks3_to_as4_animation_ptr.reset(
        new gazebo::common::PoseAnimation(this->dataPtr->agv_name, move_time_as_2_4_to_ks / speed_factor, false));
    key = this->dataPtr->agv3_from_ks3_to_as4_animation_ptr->CreateKeyFrame(0);
    for (int i = 1; i < 11; i++) {
        key = this->dataPtr->agv3_from_ks3_to_as4_animation_ptr->CreateKeyFrame(i / 10 * move_time_as_2_4_to_ks / speed_factor);
        key->Translation(ignition::math::Vector3d(current_x - i / 10 * length_ks_as_2_4, current_y, height));
        key->Rotation(ignition::math::Quaterniond(0, 0, yaw));
    }

    /*
    * ======================================
    * AGV3: From AS3 to AS4
    * ======================================
    */
    current_x = -5.6;
    current_y = -1.333917;
    this->dataPtr->agv3_from_as3_to_as4_animation_ptr.reset(
        new gazebo::common::PoseAnimation(this->dataPtr->agv_name, move_time_as_to_as / speed_factor, false));
    key = this->dataPtr->agv3_from_as3_to_as4_animation_ptr->CreateKeyFrame(0);
    for (int i = 1; i < 11; i++) {
        key = this->dataPtr->agv3_from_as3_to_as4_animation_ptr->CreateKeyFrame(i / 10 * move_time_as_to_as / speed_factor);
        key->Translation(ignition::math::Vector3d(current_x - i / 10 * length_as_as, current_y, height));
        key->Rotation(ignition::math::Quaterniond(0, 0, yaw));
    }

    /*
    * ======================================
    * AGV3: From AS4 to AS3
    * ======================================
    */

    current_x = -10.590274;
    current_y = -1.333917;
    this->dataPtr->agv3_from_as4_to_as3_animation_ptr.reset(
        new gazebo::common::PoseAnimation(this->dataPtr->agv_name, move_time_as_to_as / speed_factor, false));
    key = this->dataPtr->agv3_from_as4_to_as3_animation_ptr->CreateKeyFrame(0);
    for (int i = 1; i < 11; i++) {
        key = this->dataPtr->agv3_from_as4_to_as3_animation_ptr->CreateKeyFrame(i / 10 * move_time_as_to_as / speed_factor);
        key->Translation(ignition::math::Vector3d(current_x + i / 10 * length_as_as, current_y, height));
        key->Rotation(ignition::math::Quaterniond(0, 0, yaw));
    }

    /*
      * ======================================
      * AGV3: From AS3 to KS3
      * =====================================
      */

    current_x = -5.6;
    current_y = -1.333917;
    this->dataPtr->agv3_from_as3_to_ks3_animation_ptr.reset(
        new gazebo::common::PoseAnimation(this->dataPtr->agv_name, move_time_as_1_3_to_ks / speed_factor, false));
    key = this->dataPtr->agv3_from_as3_to_ks3_animation_ptr->CreateKeyFrame(0);
    for (int i = 1; i < 11; i++) {
        key = this->dataPtr->agv3_from_as3_to_ks3_animation_ptr->CreateKeyFrame(i / 10 * move_time_as_1_3_to_ks / speed_factor);
        key->Translation(ignition::math::Vector3d(current_x + i / 10 * length_ks_as_1_3, current_y, height));
        key->Rotation(ignition::math::Quaterniond(0, 0, yaw));
    }

    /*
      * ======================================
      * AGV3: From AS4 to KS3
      * ======================================
      */

    current_x = -10.590274;
    current_y = -1.333917;
    this->dataPtr->agv3_from_as4_to_ks3_animation_ptr.reset(
        new gazebo::common::PoseAnimation(this->dataPtr->agv_name, move_time_as_2_4_to_ks / speed_factor, false));
    key = this->dataPtr->agv3_from_as4_to_ks3_animation_ptr->CreateKeyFrame(0);
    for (int i = 1; i < 11; i++) {
        key = this->dataPtr->agv3_from_as4_to_ks3_animation_ptr->CreateKeyFrame(i / 10 * move_time_as_2_4_to_ks / speed_factor);
        key->Translation(ignition::math::Vector3d(current_x + i / 10 * length_ks_as_2_4, current_y, height));
        key->Rotation(ignition::math::Quaterniond(0, 0, yaw));
    }


    ////////////////////////////////
    /*
    * ======================================
    * AGV4: From KS4 to AS3
    * ======================================
    */
    current_x = -2.265685;
    current_y = -4.696062;
    this->dataPtr->agv4_from_ks4_to_as3_animation_ptr.reset(
        new gazebo::common::PoseAnimation(this->dataPtr->agv_name, move_time_as_1_3_to_ks / speed_factor, false));
    key = this->dataPtr->agv4_from_ks4_to_as3_animation_ptr->CreateKeyFrame(0);
    for (int i = 1; i < 11; i++) {
        key = this->dataPtr->agv4_from_ks4_to_as3_animation_ptr->CreateKeyFrame(i / 10 * move_time_as_1_3_to_ks / speed_factor);
        key->Translation(ignition::math::Vector3d(current_x - i / 10 * length_ks_as_1_3, current_y, height));
        key->Rotation(ignition::math::Quaterniond(0, 0, yaw));
    }

    /*
    * ======================================
    * AGV4: From KS4 to AS4
    * ======================================
    */
    current_x = -2.265685;
    current_y = -4.696062;
    this->dataPtr->agv4_from_ks4_to_as4_animation_ptr.reset(
        new gazebo::common::PoseAnimation(this->dataPtr->agv_name, move_time_as_2_4_to_ks / speed_factor, false));
    key = this->dataPtr->agv4_from_ks4_to_as4_animation_ptr->CreateKeyFrame(0);
    for (int i = 1; i < 11; i++) {
        key = this->dataPtr->agv4_from_ks4_to_as4_animation_ptr->CreateKeyFrame(i / 10 * move_time_as_2_4_to_ks / speed_factor);
        key->Translation(ignition::math::Vector3d(current_x - i / 10 * length_ks_as_2_4, current_y, height));
        key->Rotation(ignition::math::Quaterniond(0, 0, yaw));
    }

    /*
    * ======================================
    * AGV4: From AS3 to AS4
    * ======================================
    */
    current_x = -5.6;
    current_y = -4.696062;
    this->dataPtr->agv4_from_as3_to_as4_animation_ptr.reset(
        new gazebo::common::PoseAnimation(this->dataPtr->agv_name, move_time_as_to_as / speed_factor, false));
    key = this->dataPtr->agv4_from_as3_to_as4_animation_ptr->CreateKeyFrame(0);
    for (int i = 1; i < 11; i++) {
        key = this->dataPtr->agv4_from_as3_to_as4_animation_ptr->CreateKeyFrame(i / 10 * move_time_as_to_as / speed_factor);
        key->Translation(ignition::math::Vector3d(current_x - i / 10 * length_as_as, current_y, height));
        key->Rotation(ignition::math::Quaterniond(0, 0, yaw));
    }

    /*
    * ======================================
    * AGV4: From AS4 to AS3
    * ======================================
    */

    current_x = -10.590274;
    current_y = -4.696062;
    this->dataPtr->agv4_from_as4_to_as3_animation_ptr.reset(
        new gazebo::common::PoseAnimation(this->dataPtr->agv_name, move_time_as_to_as / speed_factor, false));
    key = this->dataPtr->agv4_from_as4_to_as3_animation_ptr->CreateKeyFrame(0);
    for (int i = 1; i < 11; i++) {
        key = this->dataPtr->agv4_from_as4_to_as3_animation_ptr->CreateKeyFrame(i / 10 * move_time_as_to_as / speed_factor);
        key->Translation(ignition::math::Vector3d(current_x + i / 10 * length_as_as, current_y, height));
        key->Rotation(ignition::math::Quaterniond(0, 0, yaw));
    }

    /*
      * ======================================
      * AGV4: From AS3 to KS4
      * =====================================
      */

    current_x = -5.6;
    current_y = -4.696062;
    this->dataPtr->agv4_from_as3_to_ks4_animation_ptr.reset(
        new gazebo::common::PoseAnimation(this->dataPtr->agv_name, move_time_as_1_3_to_ks / speed_factor, false));
    key = this->dataPtr->agv4_from_as3_to_ks4_animation_ptr->CreateKeyFrame(0);
    for (int i = 1; i < 11; i++) {
        key = this->dataPtr->agv4_from_as3_to_ks4_animation_ptr->CreateKeyFrame(i / 10 * move_time_as_1_3_to_ks / speed_factor);
        key->Translation(ignition::math::Vector3d(current_x + i / 10 * length_ks_as_1_3, current_y, height));
        key->Rotation(ignition::math::Quaterniond(0, 0, yaw));
    }

    /*
      * ======================================
      * AGV4: From AS4 to KS4
      * ======================================
      */

    current_x = -10.590274;
    current_y = -4.696062;
    this->dataPtr->agv4_from_as4_to_ks4_animation_ptr.reset(
        new gazebo::common::PoseAnimation(this->dataPtr->agv_name, move_time_as_2_4_to_ks / speed_factor, false));
    key = this->dataPtr->agv4_from_as4_to_ks4_animation_ptr->CreateKeyFrame(0);
    for (int i = 1; i < 11; i++) {
        key = this->dataPtr->agv4_from_as4_to_ks4_animation_ptr->CreateKeyFrame(i / 10 * move_time_as_2_4_to_ks / speed_factor);
        key->Translation(ignition::math::Vector3d(current_x + i / 10 * length_ks_as_2_4, current_y, height));
        key->Rotation(ignition::math::Quaterniond(0, 0, yaw));
    }

    this->dataPtr->model = _parent;

    /**
     * =========================================
     * Advertise services
     * =========================================
     */
    if (this->dataPtr->model->GetName() == "agv1" || this->dataPtr->model->GetName() == "agv2") {
        this->dataPtr->ks_to_as1_service = this->dataPtr->ros_node->advertiseService(ks_to_as1_service_name,
            &ROSAGVPlugin::ProcessKsToAs1ServiceCallback, this);
        this->dataPtr->ks_to_as2_service = this->dataPtr->ros_node->advertiseService(ks_to_as2_service_name,
            &ROSAGVPlugin::ProcessKsToAs2ServiceCallback, this);
        this->dataPtr->as1_to_as2_service = this->dataPtr->ros_node->advertiseService(as1_to_as2_service_name,
            &ROSAGVPlugin::ProcessAs1ToAs2ServiceCallback, this);
        this->dataPtr->as2_to_as1_service = this->dataPtr->ros_node->advertiseService(as2_to_as1_service_name,
            &ROSAGVPlugin::ProcessAs2ToAs1ServiceCallback, this);
        this->dataPtr->as1_to_ks_service = this->dataPtr->ros_node->advertiseService(as1_to_ks_service_name,
            &ROSAGVPlugin::ProcessAs1ToKsServiceCallback, this);
        this->dataPtr->as2_to_ks_service = this->dataPtr->ros_node->advertiseService(as2_to_ks_service_name,
            &ROSAGVPlugin::ProcessAs2ToKsServiceCallback, this);
    }

    if (this->dataPtr->model->GetName() == "agv3" || this->dataPtr->model->GetName() == "agv4") {
        this->dataPtr->ks_to_as3_service = this->dataPtr->ros_node->advertiseService(ks_to_as3_service_name,
            &ROSAGVPlugin::ProcessKsToAs3ServiceCallback, this);
        this->dataPtr->ks_to_as4_service = this->dataPtr->ros_node->advertiseService(ks_to_as4_service_name,
            &ROSAGVPlugin::ProcessKsToAs4ServiceCallback, this);
        this->dataPtr->as3_to_as4_service = this->dataPtr->ros_node->advertiseService(as3_to_as4_service_name,
            &ROSAGVPlugin::ProcessAs3ToAs4ServiceCallback, this);
        this->dataPtr->as4_to_as3_service = this->dataPtr->ros_node->advertiseService(as4_to_as3_service_name,
            &ROSAGVPlugin::ProcessAs4ToAs3ServiceCallback, this);
        this->dataPtr->as3_to_ks_service = this->dataPtr->ros_node->advertiseService(as3_to_ks_service_name,
            &ROSAGVPlugin::ProcessAs3ToKsServiceCallback, this);
        this->dataPtr->as4_to_ks_service = this->dataPtr->ros_node->advertiseService(as4_to_ks_service_name,
            &ROSAGVPlugin::ProcessAs4ToKsServiceCallback, this);
    }

    // Publisher for the status of the AGV.
    std::string state_topic = "/ariac/" + this->dataPtr->agv_name + "/state";
    this->dataPtr->agv_state_publisher = this->dataPtr->ros_node->advertise<
        std_msgs::String>(state_topic, 1000);

    std::string station_topic = "/ariac/" + this->dataPtr->agv_name + "/station";
    this->dataPtr->agv_station_publisher = this->dataPtr->ros_node->advertise<
        std_msgs::String>(station_topic, 1000);

    this->dataPtr->as2StatusPublisher = this->dataPtr->ros_node->advertise<
        std_msgs::Bool>("/ariac/as2/occupied", 10, true);

    this->dataPtr->as4StatusPublisher = this->dataPtr->ros_node->advertise<
        std_msgs::Bool>("/ariac/as4/occupied", 10, true);


    this->dataPtr->current_state = "READY_TO_DELIVER";

    // Listen to the update event. This event is broadcast every
    // simulation iteration.
    this->dataPtr->update_connection_ptr = event::Events::ConnectWorldUpdateBegin(
        boost::bind(&ROSAGVPlugin::OnUpdate, this, _1));
}

void ROSAGVPlugin::OnAGVLocation(std_msgs::String::ConstPtr _msg)
{
    std::lock_guard<std::mutex> lock(this->dataPtr->mutex);
    this->dataPtr->current_station = _msg->data;
}


/////////////////////////////////////////////////
void ROSAGVPlugin::OnUpdate(const common::UpdateInfo& _info)
{
    auto current_sim_time = _info.simTime;
    if (this->dataPtr->current_state == "READY_TO_DELIVER") {

        // tasking an AGV to go to an assembly station
        if (this->dataPtr->goto_as1_triggered) {
            this->dataPtr->current_state = "GOTO_AS1";
            this->dataPtr->goto_as_trigger_time = current_sim_time;
            this->dataPtr->goto_as1_triggered = false;
        }
        if (this->dataPtr->goto_as2_triggered) {
            this->dataPtr->current_state = "GOTO_AS2";
            this->dataPtr->goto_as_trigger_time = current_sim_time;
            this->dataPtr->goto_as2_triggered = false;
        }
        if (this->dataPtr->goto_as3_triggered) {
            this->dataPtr->current_state = "GOTO_AS3";
            this->dataPtr->goto_as_trigger_time = current_sim_time;
            this->dataPtr->goto_as3_triggered = false;
        }
        if (this->dataPtr->goto_as4_triggered) {
            this->dataPtr->current_state = "GOTO_AS4";
            this->dataPtr->goto_as_trigger_time = current_sim_time;
            this->dataPtr->goto_as4_triggered = false;
        }

        // tasking an AGV to go to an assembly station
        if (this->dataPtr->goto_ks_triggered) {
            this->dataPtr->current_state = "GOTO_KS";
            this->dataPtr->goto_ks_trigger_time = current_sim_time;
            this->dataPtr->goto_ks_triggered = false;
        }
    }

    ////// GOTO_KS
    if (this->dataPtr->current_state == "GOTO_KS") {
        if (current_sim_time - this->dataPtr->goto_ks_trigger_time > 1.0) {

            // select animation based on the name of the station
            std::string current_station = this->dataPtr->current_station;
            // ROS_WARN_STREAM("Current station: " << current_station);

            if (this->dataPtr->agv_name == "agv1") {
                if (current_station == "as1") {
                    this->dataPtr->agv1_from_as1_to_ks1_animation_ptr->SetTime(0);
                    this->dataPtr->model->SetAnimation(this->dataPtr->agv1_from_as1_to_ks1_animation_ptr);
                    this->dataPtr->current_state = "AGV1_AS1_KS";
                }
                else if (current_station == "as2") {
                    this->dataPtr->agv1_from_as2_to_ks1_animation_ptr->SetTime(0);
                    this->dataPtr->model->SetAnimation(this->dataPtr->agv1_from_as2_to_ks1_animation_ptr);
                    this->dataPtr->current_state = "AGV1_AS2_KS";
                }
            }

            if (this->dataPtr->agv_name == "agv2") {
                if (current_station == "as1") {
                    this->dataPtr->agv2_from_as1_to_ks2_animation_ptr->SetTime(0);
                    this->dataPtr->model->SetAnimation(this->dataPtr->agv2_from_as1_to_ks2_animation_ptr);
                    this->dataPtr->current_state = "AGV2_AS1_KS";
                }
                else if (current_station == "as2") {
                    this->dataPtr->agv2_from_as2_to_ks2_animation_ptr->SetTime(0);
                    this->dataPtr->model->SetAnimation(this->dataPtr->agv2_from_as2_to_ks2_animation_ptr);
                    this->dataPtr->current_state = "AGV2_AS2_KS";
                }
            }

            if (this->dataPtr->agv_name == "agv3") {
                if (current_station == "as3") {
                    this->dataPtr->agv3_from_as3_to_ks3_animation_ptr->SetTime(0);
                    this->dataPtr->model->SetAnimation(this->dataPtr->agv3_from_as3_to_ks3_animation_ptr);
                    this->dataPtr->current_state = "AGV3_AS3_KS";
                }
                else if (current_station == "as4") {
                    this->dataPtr->agv3_from_as4_to_ks3_animation_ptr->SetTime(0);
                    this->dataPtr->model->SetAnimation(this->dataPtr->agv3_from_as4_to_ks3_animation_ptr);
                    this->dataPtr->current_state = "AGV3_AS4_KS";
                }
            }

            if (this->dataPtr->agv_name == "agv4") {
                if (current_station == "as3") {
                    this->dataPtr->agv4_from_as3_to_ks4_animation_ptr->SetTime(0);
                    this->dataPtr->model->SetAnimation(this->dataPtr->agv4_from_as3_to_ks4_animation_ptr);
                    this->dataPtr->current_state = "AGV4_AS3_KS";
                }
                else if (current_station == "as4") {
                    this->dataPtr->agv4_from_as4_to_ks4_animation_ptr->SetTime(0);
                    this->dataPtr->model->SetAnimation(this->dataPtr->agv4_from_as4_to_ks4_animation_ptr);
                    this->dataPtr->current_state = "AGV4_AS4_KS";
                }
            }

        }
    }

    if (this->dataPtr->current_state == "AGV1_AS1_KS") {
        bool animation_done = this->dataPtr->agv1_from_as1_to_ks1_animation_ptr->GetTime() >=
            this->dataPtr->agv1_from_as1_to_ks1_animation_ptr->GetLength();
        if (animation_done) {
            gzdbg << "AGV1 reached KS1." << std::endl;
            this->dataPtr->current_state = "DOCKED_TO_KS";
        }
    }

    if (this->dataPtr->current_state == "AGV1_AS2_KS") {
        bool animation_done = this->dataPtr->agv1_from_as2_to_ks1_animation_ptr->GetTime() >=
            this->dataPtr->agv1_from_as2_to_ks1_animation_ptr->GetLength();
        if (animation_done) {
            gzdbg << "AGV1 reached KS1." << std::endl;
            this->dataPtr->current_state = "DOCKED_TO_KS";
        }
    }

    if (this->dataPtr->current_state == "AGV2_AS1_KS") {
        bool animation_done = this->dataPtr->agv2_from_as1_to_ks2_animation_ptr->GetTime() >=
            this->dataPtr->agv2_from_as1_to_ks2_animation_ptr->GetLength();
        if (animation_done) {
            gzdbg << "AGV2 reached KS2." << std::endl;
            this->dataPtr->current_state = "DOCKED_TO_KS";
        }
    }

    if (this->dataPtr->current_state == "AGV2_AS2_KS") {
        bool animation_done = this->dataPtr->agv2_from_as2_to_ks2_animation_ptr->GetTime() >=
            this->dataPtr->agv2_from_as2_to_ks2_animation_ptr->GetLength();
        if (animation_done) {
            gzdbg << "AGV2 reached KS2." << std::endl;
            this->dataPtr->current_state = "DOCKED_TO_KS";
        }
    }

    if (this->dataPtr->current_state == "AGV3_AS3_KS") {
        bool animation_done = this->dataPtr->agv3_from_as3_to_ks3_animation_ptr->GetTime() >=
            this->dataPtr->agv3_from_as3_to_ks3_animation_ptr->GetLength();
        if (animation_done) {
            gzdbg << "AGV3 reached KS3." << std::endl;
            this->dataPtr->current_state = "DOCKED_TO_KS";
        }
    }

    if (this->dataPtr->current_state == "AGV3_AS4_KS") {
        bool animation_done = this->dataPtr->agv3_from_as4_to_ks3_animation_ptr->GetTime() >=
            this->dataPtr->agv3_from_as4_to_ks3_animation_ptr->GetLength();
        if (animation_done) {
            gzdbg << "AGV3 reached KS3." << std::endl;
            this->dataPtr->current_state = "DOCKED_TO_KS";
        }
    }

    if (this->dataPtr->current_state == "AGV4_AS3_KS") {
        bool animation_done = this->dataPtr->agv4_from_as3_to_ks4_animation_ptr->GetTime() >=
            this->dataPtr->agv4_from_as3_to_ks4_animation_ptr->GetLength();
        if (animation_done) {
            gzdbg << "AGV4 reached KS4." << std::endl;
            this->dataPtr->current_state = "DOCKED_TO_KS";
        }
    }

    if (this->dataPtr->current_state == "AGV4_AS4_KS") {
        bool animation_done = this->dataPtr->agv4_from_as4_to_ks4_animation_ptr->GetTime() >=
            this->dataPtr->agv4_from_as4_to_ks4_animation_ptr->GetLength();
        if (animation_done) {
            gzdbg << "AGV4 reached KS4." << std::endl;
            this->dataPtr->current_state = "DOCKED_TO_KS";
        }
    }

    ////// GOTO_AS1

    if (this->dataPtr->current_state == "GOTO_AS1") {
        if (current_sim_time - this->dataPtr->goto_as_trigger_time > 1.0) {
            // select animation based on the name of the station
            std::string current_station = this->dataPtr->current_station;

            // AGV1
            if (this->dataPtr->agv_name == "agv1") {
                if (current_station == "ks1") {
                    this->dataPtr->agv1_from_ks1_to_as1_animation_ptr->SetTime(0);
                    this->dataPtr->model->SetAnimation(this->dataPtr->agv1_from_ks1_to_as1_animation_ptr);
                    this->dataPtr->current_state = "AGV1_KS1_AS1";
                }
                else if (current_station == "as2") {
                    this->dataPtr->agv1_from_as2_to_as1_animation_ptr->SetTime(0);
                    this->dataPtr->model->SetAnimation(this->dataPtr->agv1_from_as2_to_as1_animation_ptr);
                    this->dataPtr->current_state = "AGV1_AS2_AS1";
                }
            }

            // AGV2
            if (this->dataPtr->agv_name == "agv2") {
                if (current_station == "ks2") {
                    this->dataPtr->agv2_from_ks2_to_as1_animation_ptr->SetTime(0);
                    this->dataPtr->model->SetAnimation(this->dataPtr->agv2_from_ks2_to_as1_animation_ptr);
                    this->dataPtr->current_state = "AGV2_KS2_AS1";
                }
                else if (current_station == "as2") {
                    this->dataPtr->agv2_from_as2_to_as1_animation_ptr->SetTime(0);
                    this->dataPtr->model->SetAnimation(this->dataPtr->agv2_from_as2_to_as1_animation_ptr);
                    this->dataPtr->current_state = "AGV2_AS2_AS1";
                }
            }

        }
    }

    if (this->dataPtr->current_state == "AGV1_KS1_AS1") {
        bool animation_done = this->dataPtr->agv1_from_ks1_to_as1_animation_ptr->GetTime() >=
            this->dataPtr->agv1_from_ks1_to_as1_animation_ptr->GetLength();
        if (animation_done) {
            // gzdbg << "AGV1 reached AS1." << std::endl;
            this->dataPtr->current_state = "DOCKED_TO_AS";
        }
    }

    if (this->dataPtr->current_state == "AGV1_AS2_AS1") {
        bool animation_done = this->dataPtr->agv1_from_as2_to_as1_animation_ptr->GetTime() >=
            this->dataPtr->agv1_from_as2_to_as1_animation_ptr->GetLength();
        if (animation_done) {
            // gzdbg << "AGV1 reached AS1." << std::endl;
            this->dataPtr->current_state = "DOCKED_TO_AS";
        }
    }

    if (this->dataPtr->current_state == "AGV2_KS2_AS1") {
        bool animation_done = this->dataPtr->agv2_from_ks2_to_as1_animation_ptr->GetTime() >=
            this->dataPtr->agv2_from_ks2_to_as1_animation_ptr->GetLength();
        if (animation_done) {
            // gzdbg << "AGV2 reached AS1." << std::endl;
            this->dataPtr->current_state = "DOCKED_TO_AS";
        }
    }

    if (this->dataPtr->current_state == "AGV2_AS2_AS1") {
        bool animation_done = this->dataPtr->agv2_from_as2_to_as1_animation_ptr->GetTime() >=
            this->dataPtr->agv2_from_as2_to_as1_animation_ptr->GetLength();
        if (animation_done) {
            // gzdbg << "AGV2 reached AS1." << std::endl;
            this->dataPtr->current_state = "DOCKED_TO_AS";
        }
    }

    ////// GOTO_AS2

    if (this->dataPtr->current_state == "GOTO_AS2") {
        if (current_sim_time - this->dataPtr->goto_as_trigger_time > 1.0) {

            // select animation based on the name of the station
            std::string current_station = this->dataPtr->current_station;

            // AGV1
            if (this->dataPtr->agv_name == "agv1") {
                if (current_station == "ks1") {
                    this->dataPtr->agv1_from_ks1_to_as2_animation_ptr->SetTime(0);
                    this->dataPtr->model->SetAnimation(this->dataPtr->agv1_from_ks1_to_as2_animation_ptr);
                    this->dataPtr->current_state = "AGV1_KS1_AS2";
                }
                else if (current_station == "as1") {
                    this->dataPtr->agv1_from_as1_to_as2_animation_ptr->SetTime(0);
                    this->dataPtr->model->SetAnimation(this->dataPtr->agv1_from_as1_to_as2_animation_ptr);
                    this->dataPtr->current_state = "AGV1_AS1_AS2";
                }
            }

            // AGV2
            if (this->dataPtr->agv_name == "agv2") {
                if (current_station == "ks2") {
                    this->dataPtr->agv2_from_ks2_to_as2_animation_ptr->SetTime(0);
                    this->dataPtr->model->SetAnimation(this->dataPtr->agv2_from_ks2_to_as2_animation_ptr);
                    this->dataPtr->current_state = "AGV2_KS2_AS2";
                }
                else if (current_station == "as1") {
                    this->dataPtr->agv2_from_as1_to_as2_animation_ptr->SetTime(0);
                    this->dataPtr->model->SetAnimation(this->dataPtr->agv2_from_as1_to_as2_animation_ptr);
                    this->dataPtr->current_state = "AGV2_AS1_AS2";
                }
            }
        }
    }

    if (this->dataPtr->current_state == "AGV1_KS1_AS2") {
        bool animation_done = this->dataPtr->agv1_from_ks1_to_as2_animation_ptr->GetTime() >=
            this->dataPtr->agv1_from_ks1_to_as2_animation_ptr->GetLength();
        if (animation_done) {
            // gzdbg << "AGV1 reached AS2." << std::endl;
            this->dataPtr->current_state = "DOCKED_TO_AS";
        }
    }

    if (this->dataPtr->current_state == "AGV1_AS1_AS2") {
        bool animation_done = this->dataPtr->agv1_from_as1_to_as2_animation_ptr->GetTime() >=
            this->dataPtr->agv1_from_as1_to_as2_animation_ptr->GetLength();
        if (animation_done) {
            // gzdbg << "AGV1 reached AS2." << std::endl;
            this->dataPtr->current_state = "DOCKED_TO_AS";
        }
    }

    if (this->dataPtr->current_state == "AGV2_KS2_AS2") {
        bool animation_done = this->dataPtr->agv2_from_ks2_to_as2_animation_ptr->GetTime() >=
            this->dataPtr->agv2_from_ks2_to_as2_animation_ptr->GetLength();
        if (animation_done) {
            // gzdbg << "AGV2 reached AS2." << std::endl;
            this->dataPtr->current_state = "DOCKED_TO_AS";
        }
    }

    if (this->dataPtr->current_state == "AGV2_AS1_AS2") {
        bool animation_done = this->dataPtr->agv2_from_as1_to_as2_animation_ptr->GetTime() >=
            this->dataPtr->agv2_from_as1_to_as2_animation_ptr->GetLength();
        if (animation_done) {
            // gzdbg << "AGV2 reached AS2." << std::endl;
            this->dataPtr->current_state = "DOCKED_TO_AS";
        }
    }

    ////// GOTO_AS3

    if (this->dataPtr->current_state == "GOTO_AS3") {
        if (current_sim_time - this->dataPtr->goto_as_trigger_time > 1.0) {

            // select animation based on the name of the station
            std::string current_station = this->dataPtr->current_station;

            // AGV3
            if (this->dataPtr->agv_name == "agv3") {
                if (current_station == "ks3") {
                    this->dataPtr->agv3_from_ks3_to_as3_animation_ptr->SetTime(0);
                    this->dataPtr->model->SetAnimation(this->dataPtr->agv3_from_ks3_to_as3_animation_ptr);
                    this->dataPtr->current_state = "AGV3_KS3_AS3";
                }
                else if (current_station == "as4") {
                    this->dataPtr->agv3_from_as4_to_as3_animation_ptr->SetTime(0);
                    this->dataPtr->model->SetAnimation(this->dataPtr->agv3_from_as4_to_as3_animation_ptr);
                    this->dataPtr->current_state = "AGV3_AS4_AS3";
                }
            }

            // AGV4
            if (this->dataPtr->agv_name == "agv4") {
                if (current_station == "ks4") {
                    this->dataPtr->agv4_from_ks4_to_as3_animation_ptr->SetTime(0);
                    this->dataPtr->model->SetAnimation(this->dataPtr->agv4_from_ks4_to_as3_animation_ptr);
                    this->dataPtr->current_state = "AGV4_KS4_AS3";
                }
                else if (current_station == "as4") {
                    this->dataPtr->agv4_from_as4_to_as3_animation_ptr->SetTime(0);
                    this->dataPtr->model->SetAnimation(this->dataPtr->agv4_from_as4_to_as3_animation_ptr);
                    this->dataPtr->current_state = "AGV4_AS4_AS3";
                }
            }
        }
    }

    if (this->dataPtr->current_state == "AGV3_KS3_AS3") {
        bool animation_done = this->dataPtr->agv3_from_ks3_to_as3_animation_ptr->GetTime() >=
            this->dataPtr->agv3_from_ks3_to_as3_animation_ptr->GetLength();
        if (animation_done) {
            // gzdbg << "AGV3 reached AS3." << std::endl;
            this->dataPtr->current_state = "DOCKED_TO_AS";
        }
    }

    if (this->dataPtr->current_state == "AGV3_AS4_AS3") {
        bool animation_done = this->dataPtr->agv3_from_as4_to_as3_animation_ptr->GetTime() >=
            this->dataPtr->agv3_from_as4_to_as3_animation_ptr->GetLength();
        if (animation_done) {
            // gzdbg << "AGV3 reached AS3." << std::endl;
            this->dataPtr->current_state = "DOCKED_TO_AS";
        }
    }

    if (this->dataPtr->current_state == "AGV4_KS4_AS3") {
        bool animation_done = this->dataPtr->agv4_from_ks4_to_as3_animation_ptr->GetTime() >=
            this->dataPtr->agv4_from_ks4_to_as3_animation_ptr->GetLength();
        if (animation_done) {
            // gzdbg << "AGV4 reached AS3." << std::endl;
            this->dataPtr->current_state = "DOCKED_TO_AS";
        }
    }

    if (this->dataPtr->current_state == "AGV4_AS4_AS3") {
        bool animation_done = this->dataPtr->agv4_from_as4_to_as3_animation_ptr->GetTime() >=
            this->dataPtr->agv4_from_as4_to_as3_animation_ptr->GetLength();
        if (animation_done) {
            // gzdbg << "AGV4 reached AS3." << std::endl;
            this->dataPtr->current_state = "DOCKED_TO_AS";
        }
    }

    ////// GOTO_AS4

    if (this->dataPtr->current_state == "GOTO_AS4") {
        if (current_sim_time - this->dataPtr->goto_as_trigger_time > 1.0) {
            // select animation based on the name of the station
            std::string current_station = this->dataPtr->current_station;

            // AGV3
            if (this->dataPtr->agv_name == "agv3") {
                if (current_station == "ks3") {
                    this->dataPtr->agv3_from_ks3_to_as4_animation_ptr->SetTime(0);
                    this->dataPtr->model->SetAnimation(this->dataPtr->agv3_from_ks3_to_as4_animation_ptr);
                    this->dataPtr->current_state = "AGV3_KS3_AS4";
                }
                else if (current_station == "as3") {
                    this->dataPtr->agv3_from_as3_to_as4_animation_ptr->SetTime(0);
                    this->dataPtr->model->SetAnimation(this->dataPtr->agv3_from_as3_to_as4_animation_ptr);
                    this->dataPtr->current_state = "AGV3_AS3_AS4";
                }
            }

            // AGV4
            if (this->dataPtr->agv_name == "agv4") {
                if (current_station == "ks4") {
                    this->dataPtr->agv4_from_ks4_to_as4_animation_ptr->SetTime(0);
                    this->dataPtr->model->SetAnimation(this->dataPtr->agv4_from_ks4_to_as4_animation_ptr);
                    this->dataPtr->current_state = "AGV4_KS4_AS4";
                }
                else if (current_station == "as3") {
                    this->dataPtr->agv4_from_as3_to_as4_animation_ptr->SetTime(0);
                    this->dataPtr->model->SetAnimation(this->dataPtr->agv4_from_as3_to_as4_animation_ptr);
                    this->dataPtr->current_state = "AGV4_AS3_AS4";
                }
            }
        }
    }

    if (this->dataPtr->current_state == "AGV3_KS3_AS4") {
        bool animation_done = this->dataPtr->agv3_from_ks3_to_as4_animation_ptr->GetTime() >=
            this->dataPtr->agv3_from_ks3_to_as4_animation_ptr->GetLength();
        if (animation_done) {
            // gzdbg << "AGV3 reached AS4." << std::endl;
            this->dataPtr->current_state = "DOCKED_TO_AS";
        }
    }

    if (this->dataPtr->current_state == "AGV3_AS3_AS4") {
        bool animation_done = this->dataPtr->agv3_from_as3_to_as4_animation_ptr->GetTime() >=
            this->dataPtr->agv3_from_as3_to_as4_animation_ptr->GetLength();
        if (animation_done) {
            // gzdbg << "AGV3 reached AS4." << std::endl;
            this->dataPtr->current_state = "DOCKED_TO_AS";
        }
    }

    if (this->dataPtr->current_state == "AGV4_KS4_AS4") {
        bool animation_done = this->dataPtr->agv4_from_ks4_to_as4_animation_ptr->GetTime() >=
            this->dataPtr->agv4_from_ks4_to_as4_animation_ptr->GetLength();
        if (animation_done) {
            gzdbg << "AGV4 reached AS4." << std::endl;
            this->dataPtr->current_state = "DOCKED_TO_AS";
        }
    }

    if (this->dataPtr->current_state == "AGV4_AS3_AS4") {
        bool animation_done = this->dataPtr->agv4_from_as3_to_as4_animation_ptr->GetTime() >=
            this->dataPtr->agv4_from_as3_to_as4_animation_ptr->GetLength();
        if (animation_done) {
            // gzdbg << "AGV4 reached AS4." << std::endl;
            this->dataPtr->current_state = "DOCKED_TO_AS";
        }
    }


    if (this->dataPtr->current_state == "DOCKED_TO_AS") {
        gzdbg << "Docked to assembly station." << std::endl;
        // publish the new station on the appropriate topic
        std_msgs::String msg;
        msg.data = this->dataPtr->assembly_station_name;
        this->dataPtr->agv_station_publisher.publish(msg);

        //publish whether or not this assembly station has an agv
        //this is used by the ROSPersonByStation plugin
        if (this->dataPtr->assembly_station_name == "as2") {
            std_msgs::Bool as_msg;
            as_msg.data = true;
            this->dataPtr->as2StatusPublisher.publish(as_msg);
            gazebo::msgs::GzString unlock_msg;
            unlock_msg.set_data("unlock");
            this->dataPtr->lock_unlock_models_gz_pub->Publish(unlock_msg);
        }

        //publish whether or not this assembly station has an agv
        //this is used by the ROSPersonByStation plugin
        if (this->dataPtr->assembly_station_name == "as4") {
            std_msgs::Bool as_msg;
            as_msg.data = true;
            this->dataPtr->as4StatusPublisher.publish(as_msg);
            gazebo::msgs::GzString unlock_msg;
            unlock_msg.set_data("unlock");
            this->dataPtr->lock_unlock_models_gz_pub->Publish(unlock_msg);

            
        }

        // wait for 2 s and then issue idle state for lock/unlock topic
        

        this->dataPtr->current_state = "READY_TO_DELIVER";
    }

    if (this->dataPtr->current_state == "DOCKED_TO_KS") {
        gzdbg << "Docked to kitting station." << std::endl;

        // publish the new station on the appropriate topic
        std_msgs::String msg;
        msg.data = this->dataPtr->kitting_station_name;
        this->dataPtr->agv_station_publisher.publish(msg);

        //FIXME
        gazebo::msgs::GzString unlock_msg;
        unlock_msg.set_data("unlock");
        this->dataPtr->lock_unlock_models_gz_pub->Publish(unlock_msg);

        this->dataPtr->current_state = "READY_TO_DELIVER";
    }

    std_msgs::String stateMsg;
    stateMsg.data = this->dataPtr->current_state;
    this->dataPtr->agv_state_publisher.publish(stateMsg);
}

/////////////////////////////////////////////////////
bool ROSAGVPlugin::ProcessKsToAs1ServiceCallback(
    std_srvs::Trigger::Request&,
    std_srvs::Trigger::Response& res) {

    gzdbg << "\n";
    gzdbg << "[Service Call] Move AGV to as1\n";
    gzdbg << "^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^\n";
    gzdbg << "\n";

    std::string destination = "as1";
    std::string expected_current_station{};

    if (this->dataPtr->agv_name == "agv1")
        expected_current_station = "ks1";
    else if (this->dataPtr->agv_name == "agv2")
        expected_current_station = "ks2";
    else if (this->dataPtr->agv_name == "agv3")
        expected_current_station = "ks3";
    else if (this->dataPtr->agv_name == "agv4")
        expected_current_station = "ks4";

    std::string current_station{};

    std::string parameter = "/ariac/" + this->dataPtr->agv_name + "/current_station";

    if (this->dataPtr->ros_node->getParam(parameter, current_station)) {
        if (current_station == destination) {
            res.message = "[FAILURE triggering service][" + this->dataPtr->agv_name + " is already at station '" + destination + "']";
            ROS_ERROR_STREAM(res.message);
            res.success = false;
            return true;
        }

        if (current_station != expected_current_station) {
            res.message = "[FAILURE triggering service] [" + this->dataPtr->agv_name + " has to be at station '" + expected_current_station + "' to call this service]";
            ROS_ERROR_STREAM(res.message);
            res.success = false;
            return true;
        }

        if (this->dataPtr->current_state != "READY_TO_DELIVER") {
            res.message = "[FAILURE triggering service] [Current state is not READY_TO_DELIVER]";
            ROS_ERROR_STREAM(res.message);
            res.success = false;
            return true;
        }
    }
    else {
        ROS_ERROR_STREAM("[FAILURE triggering service][The parameter " << parameter << " does not exist]");
        return true;
    }

    this->dataPtr->goto_as1_triggered = true;
    // make a request to lock the models to the tray
    gazebo::msgs::GzString lock_msg;
    lock_msg.set_data("lock");
    this->dataPtr->lock_unlock_models_gz_pub->Publish(lock_msg);

    this->dataPtr->assembly_station_name = destination;
    this->dataPtr->ros_node->setParam(parameter, destination);
    res.success = true;
    res.message = "[SUCCESS] [" + this->dataPtr->agv_name + ": " + expected_current_station + "->" + destination + " triggered]";
    ROS_INFO_STREAM(res.message);
    return true;
}

/////////////////////////////////////////////////////
bool ROSAGVPlugin::ProcessKsToAs2ServiceCallback(
    std_srvs::Trigger::Request&,
    std_srvs::Trigger::Response& res) {

    gzdbg << "\n";
    gzdbg << "[Service Call] Move AGV to as2\n";
    gzdbg << "^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^\n";
    gzdbg << "\n";

    // request is empty for this service
    std::string destination = "as2";
    std::string expected_current_station{};

    if (this->dataPtr->agv_name == "agv1")
        expected_current_station = "ks1";
    else if (this->dataPtr->agv_name == "agv2")
        expected_current_station = "ks2";
    else if (this->dataPtr->agv_name == "agv3")
        expected_current_station = "ks3";
    else if (this->dataPtr->agv_name == "agv4")
        expected_current_station = "ks4";

    std::string current_station{};

    std::string parameter = "/ariac/" + this->dataPtr->agv_name + "/current_station";

    if (this->dataPtr->ros_node->getParam(parameter, current_station)) {
        if (current_station == destination) {
            res.message = "[FAILURE triggering service][" + this->dataPtr->agv_name + " is already at station '" + destination + "']";
            ROS_ERROR_STREAM(res.message);
            res.success = false;
            return true;
        }

        if (current_station != expected_current_station) {
            res.message = "[FAILURE triggering service] [" + this->dataPtr->agv_name + " has to be at station '" + expected_current_station + "' to call this service]";
            ROS_ERROR_STREAM(res.message);
            res.success = false;
            return true;
        }

        if (this->dataPtr->current_state != "READY_TO_DELIVER") {
            res.message = "[FAILURE triggering service] [Current state is not READY_TO_DELIVER]";
            ROS_ERROR_STREAM(res.message);
            res.success = false;
            return true;
        }
    }
    else {
        ROS_ERROR_STREAM("[FAILURE triggering service][The parameter " << parameter << " does not exist]");
        return true;
    }


    res.success = true;
    res.message = "[Shipping " + this->dataPtr->agv_name + " to " + destination + "]";
    ROS_INFO_STREAM(res.message);

    this->dataPtr->goto_as2_triggered = true;
    // make a request to lock the models to the tray
    gazebo::msgs::GzString lock_msg;
    lock_msg.set_data("lock");
    this->dataPtr->lock_unlock_models_gz_pub->Publish(lock_msg);

    this->dataPtr->assembly_station_name = destination;
    this->dataPtr->ros_node->setParam(parameter, destination);

    return true;
}

/////////////////////////////////////////////////////
bool ROSAGVPlugin::ProcessKsToAs3ServiceCallback(
    std_srvs::Trigger::Request&,
    std_srvs::Trigger::Response& res) {

    gzdbg << "\n";
    gzdbg << "[Service Call] Move AGV to as3\n";
    gzdbg << "^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^\n";
    gzdbg << "\n";

    std::string destination = "as3";
    std::string expected_current_station{};

    if (this->dataPtr->agv_name == "agv1")
        expected_current_station = "ks1";
    else if (this->dataPtr->agv_name == "agv2")
        expected_current_station = "ks2";
    else if (this->dataPtr->agv_name == "agv3")
        expected_current_station = "ks3";
    else if (this->dataPtr->agv_name == "agv4")
        expected_current_station = "ks4";

    std::string current_station{};

    std::string parameter = "/ariac/" + this->dataPtr->agv_name + "/current_station";

    if (this->dataPtr->ros_node->getParam(parameter, current_station)) {
        if (current_station == destination) {
            res.message = "[FAILURE triggering service][" + this->dataPtr->agv_name + " is already at station '" + destination + "']";
            ROS_ERROR_STREAM(res.message);
            res.success = false;
            return true;
        }

        if (current_station != expected_current_station) {
            res.message = "[FAILURE triggering service] [" + this->dataPtr->agv_name + " has to be at station '" + expected_current_station + "' to call this service]";
            ROS_ERROR_STREAM(res.message);
            res.success = false;
            return true;
        }

        if (this->dataPtr->current_state != "READY_TO_DELIVER") {
            res.message = "[FAILURE triggering service] [Current state is not READY_TO_DELIVER]";
            ROS_ERROR_STREAM(res.message);
            res.success = false;
            return true;
        }
    }
    else {
        ROS_ERROR_STREAM("[FAILURE triggering service][The parameter " << parameter << " does not exist]");
        return true;
    }


    res.success = true;
    res.message = "[SUCCESS] [" + this->dataPtr->agv_name + ": " + expected_current_station + "->" + destination + " triggered]";
    ROS_INFO_STREAM(res.message);

    this->dataPtr->goto_as3_triggered = true;
    // make a request to lock the models to the tray
    gazebo::msgs::GzString lock_msg;
    lock_msg.set_data("lock");
    this->dataPtr->lock_unlock_models_gz_pub->Publish(lock_msg);

    this->dataPtr->assembly_station_name = destination;
    this->dataPtr->ros_node->setParam(parameter, destination);

    return true;
}

/////////////////////////////////////////////////////
bool ROSAGVPlugin::ProcessKsToAs4ServiceCallback(
    std_srvs::Trigger::Request&,
    std_srvs::Trigger::Response& res) {

    gzdbg << "\n";
    gzdbg << "[Service Call] Move AGV to as4\n";
    gzdbg << "^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^\n";
    gzdbg << "\n";

    std::string destination = "as4";
    std::string expected_current_station{};

    if (this->dataPtr->agv_name == "agv1")
        expected_current_station = "ks1";
    else if (this->dataPtr->agv_name == "agv2")
        expected_current_station = "ks2";
    else if (this->dataPtr->agv_name == "agv3")
        expected_current_station = "ks3";
    else if (this->dataPtr->agv_name == "agv4")
        expected_current_station = "ks4";

    std::string current_station{};

    std::string parameter = "/ariac/" + this->dataPtr->agv_name + "/current_station";

    if (this->dataPtr->ros_node->getParam(parameter, current_station)) {
        if (current_station == destination) {
            res.message = "[FAILURE triggering service][" + this->dataPtr->agv_name + " is already at station '" + destination + "']";
            ROS_ERROR_STREAM(res.message);
            res.success = false;
            return true;
        }

        if (current_station != expected_current_station) {
            res.message = "[FAILURE triggering service] [" + this->dataPtr->agv_name + " has to be at station '" + expected_current_station + "' to call this service]";
            ROS_ERROR_STREAM(res.message);
            res.success = false;
            return true;
        }

        if (this->dataPtr->current_state != "READY_TO_DELIVER") {
            res.message = "[FAILURE triggering service] [Current state is not READY_TO_DELIVER]";
            ROS_ERROR_STREAM(res.message);
            res.success = false;
            return true;
        }
    }
    else {
        ROS_ERROR_STREAM("[FAILURE triggering service][The parameter " << parameter << " does not exist]");
        return true;
    }


    res.success = true;
    res.message = "[SUCCESS] [" + this->dataPtr->agv_name + ": " + expected_current_station + "->" + destination + " triggered]";
    ROS_INFO_STREAM(res.message);

    this->dataPtr->goto_as4_triggered = true;
    // make a request to lock the models to the tray
    gazebo::msgs::GzString lock_msg;
    lock_msg.set_data("lock");
    this->dataPtr->lock_unlock_models_gz_pub->Publish(lock_msg);

    this->dataPtr->assembly_station_name = destination;
    this->dataPtr->ros_node->setParam(parameter, destination);

    return true;
}

/////////////////////////////////////////////////////
bool ROSAGVPlugin::ProcessAs1ToAs2ServiceCallback(
    std_srvs::Trigger::Request&,
    std_srvs::Trigger::Response& res) {

    gzdbg << "\n";
    gzdbg << "[Service Call] Move AGV to as2\n";
    gzdbg << "^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^\n";
    gzdbg << "\n";

    std::string destination = "as2";
    std::string current_station{};

    std::string parameter = "/ariac/" + this->dataPtr->agv_name + "/current_station";

    if (this->dataPtr->ros_node->getParam(parameter, current_station)) {
        if (current_station == destination) {
            res.message = "[FAILURE triggering service][" + this->dataPtr->agv_name + " is already at station 'as2']";
            ROS_ERROR_STREAM(res.message);
            res.success = false;
            return true;
        }

        if (current_station != "as1") {
            res.message = "[FAILURE triggering service] [" + this->dataPtr->agv_name + " has to be at station 'as1' to call this service]";
            ROS_ERROR_STREAM(res.message);
            res.success = false;
            return true;
        }

        if (this->dataPtr->current_state != "READY_TO_DELIVER") {
            res.message = "[FAILURE triggering service] [Current state is not READY_TO_DELIVER]";
            ROS_ERROR_STREAM(res.message);
            res.success = false;
            return true;
        }
    }
    else {
        ROS_ERROR_STREAM("[FAILURE triggering service][The parameter " << parameter << " does not exist]");
        return true;
    }


    res.success = true;
    res.message = "[SUCCESS] [" + this->dataPtr->agv_name + ": as1->as2 triggered]";
    ROS_INFO_STREAM(res.message);

    this->dataPtr->goto_as2_triggered = true;
    // make a request to lock the models to the tray
    gazebo::msgs::GzString lock_msg;
    lock_msg.set_data("lock");
    this->dataPtr->lock_unlock_models_gz_pub->Publish(lock_msg);

    this->dataPtr->assembly_station_name = destination;
    this->dataPtr->ros_node->setParam(parameter, "as2");

    return true;
}

/////////////////////////////////////////////////////
bool ROSAGVPlugin::ProcessAs2ToAs1ServiceCallback(
    std_srvs::Trigger::Request&,
    std_srvs::Trigger::Response& res) {

    gzdbg << "\n";
    gzdbg << "[Service Call] Move AGV to as1\n";
    gzdbg << "^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^\n";
    gzdbg << "\n";

    std::string destination = "as1";
    std::string current_station{};

    std::string parameter = "/ariac/" + this->dataPtr->agv_name + "/current_station";

    if (this->dataPtr->ros_node->getParam(parameter, current_station)) {
        if (current_station == destination) {
            res.message = "[FAILURE triggering service][" + this->dataPtr->agv_name + " is already at station 'as1']";
            ROS_ERROR_STREAM(res.message);
            res.success = false;
            return true;
        }

        if (current_station != "as2") {
            res.message = "[FAILURE triggering service] [" + this->dataPtr->agv_name + " has to be at station 'as2' to call this service]";
            ROS_ERROR_STREAM(res.message);
            res.success = false;
            return true;
        }

        if (this->dataPtr->current_state != "READY_TO_DELIVER") {
            res.message = "[FAILURE triggering service] [Current state is not READY_TO_DELIVER]";
            ROS_ERROR_STREAM(res.message);
            res.success = false;
            return true;
        }
    }
    else {
        ROS_ERROR_STREAM("[FAILURE triggering service][The parameter " << parameter << " does not exist]");
        return true;
    }


    res.success = true;
    res.message = "[SUCCESS] [" + this->dataPtr->agv_name + ": as2->as1 triggered]";
    ROS_INFO_STREAM(res.message);

    this->dataPtr->goto_as1_triggered = true;
    // make a request to lock the models to the tray
    gazebo::msgs::GzString lock_msg;
    lock_msg.set_data("lock");
    this->dataPtr->lock_unlock_models_gz_pub->Publish(lock_msg);

    this->dataPtr->assembly_station_name = destination;
    this->dataPtr->ros_node->setParam(parameter, "as1");

    return true;
}

/////////////////////////////////////////////////////
bool ROSAGVPlugin::ProcessAs3ToAs4ServiceCallback(
    std_srvs::Trigger::Request&,
    std_srvs::Trigger::Response& res) {

    gzdbg << "\n";
    gzdbg << "[Service Call] Move AGV to as4\n";
    gzdbg << "^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^\n";
    gzdbg << "\n";

    std::string destination = "as4";
    std::string current_station{};

    std::string parameter = "/ariac/" + this->dataPtr->agv_name + "/current_station";

    if (this->dataPtr->ros_node->getParam(parameter, current_station)) {
        if (current_station == destination) {
            res.message = "[FAILURE triggering service][" + this->dataPtr->agv_name + " is already at station 'as4']";
            ROS_ERROR_STREAM(res.message);
            res.success = false;
            return true;
        }

        if (current_station != "as3") {
            res.message = "[FAILURE triggering service] [" + this->dataPtr->agv_name + " has to be at station 'as3' to call this service]";
            ROS_ERROR_STREAM(res.message);
            res.success = false;
            return true;
        }

        if (this->dataPtr->current_state != "READY_TO_DELIVER") {
            res.message = "[FAILURE triggering service] [Current state is not READY_TO_DELIVER]";
            ROS_ERROR_STREAM(res.message);
            res.success = false;
            return true;
        }
    }
    else {
        ROS_ERROR_STREAM("[FAILURE triggering service][The parameter " << parameter << " does not exist]");
        return true;
    }


    res.success = true;
    res.message = "[SUCCESS] [" + this->dataPtr->agv_name + ": as3->as4 triggered]";
    ROS_INFO_STREAM(res.message);

    this->dataPtr->goto_as4_triggered = true;
    // make a request to lock the models to the tray
    gazebo::msgs::GzString lock_msg;
    lock_msg.set_data("lock");
    this->dataPtr->lock_unlock_models_gz_pub->Publish(lock_msg);

    this->dataPtr->assembly_station_name = destination;
    this->dataPtr->ros_node->setParam(parameter, destination);

    return true;
}

/////////////////////////////////////////////////////
bool ROSAGVPlugin::ProcessAs4ToAs3ServiceCallback(
    std_srvs::Trigger::Request&,
    std_srvs::Trigger::Response& res) {

    gzdbg << "\n";
    gzdbg << "[Service Call] Move AGV to as3\n";
    gzdbg << "^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^\n";
    gzdbg << "\n";

    std::string destination = "as3";
    std::string current_station{};

    std::string parameter = "/ariac/" + this->dataPtr->agv_name + "/current_station";

    if (this->dataPtr->ros_node->getParam(parameter, current_station)) {
        if (current_station == destination) {
            res.message = "[FAILURE triggering service][" + this->dataPtr->agv_name + " is already at station '" + destination + "']";
            ROS_ERROR_STREAM(res.message);
            res.success = false;
            return true;
        }

        if (current_station != "as4") {
            res.message = "[FAILURE triggering service] [" + this->dataPtr->agv_name + " has to be at station 'as4' to call this service]";
            ROS_ERROR_STREAM(res.message);
            res.success = false;
            return true;
        }

        if (this->dataPtr->current_state != "READY_TO_DELIVER") {
            res.message = "[FAILURE triggering service] [Current state is not READY_TO_DELIVER]";
            ROS_ERROR_STREAM(res.message);
            res.success = false;
            return true;
        }
    }
    else {
        ROS_ERROR_STREAM("[FAILURE triggering service][The parameter " << parameter << " does not exist]");
        return true;
    }

    res.success = true;
    res.message = "[SUCCESS] [" + this->dataPtr->agv_name + ": as4->" + destination + " triggered]";
    ROS_INFO_STREAM(res.message);

    this->dataPtr->goto_as3_triggered = true;
    // make a request to lock the models to the tray
    gazebo::msgs::GzString lock_msg;
    lock_msg.set_data("lock");
    this->dataPtr->lock_unlock_models_gz_pub->Publish(lock_msg);

    this->dataPtr->assembly_station_name = destination;
    this->dataPtr->ros_node->setParam(parameter, destination);

    return true;
}

bool ROSAGVPlugin::ProcessAs4ToKsServiceCallback(
    std_srvs::Trigger::Request&,
    std_srvs::Trigger::Response& res) {

    std::string destination{};

    if (this->dataPtr->agv_name == "agv1")
        destination = "ks1";
    else if (this->dataPtr->agv_name == "agv2")
        destination = "ks2";
    else if (this->dataPtr->agv_name == "agv3")
        destination = "ks3";
    else if (this->dataPtr->agv_name == "agv4")
        destination = "ks4";
    else {
        ROS_ERROR_STREAM("[ROSAGVPlugin] [Only possible agvs: agv1, agv2, agv3, or agv4]");
        return true;
    }

    gzdbg << "\n";
    gzdbg << "[Service Call] Move AGV to " << destination << "\n";
    gzdbg << "^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^\n";
    gzdbg << "\n";

    std::string current_station{};
    std::string parameter = "/ariac/" + this->dataPtr->agv_name + "/current_station";
    if (this->dataPtr->ros_node->getParam(parameter, current_station)) {
        if (current_station == destination) {
            res.message = "[FAILURE triggering service][" + this->dataPtr->agv_name + " is already at station '" + destination + "']";
            ROS_ERROR_STREAM(res.message);
            res.success = false;
            return true;
        }

        if (current_station != "as4") {
            res.message = "[FAILURE triggering service] [" + this->dataPtr->agv_name + " has to be at station 'as4' to call this service]";
            ROS_ERROR_STREAM(res.message);
            res.success = false;
            return true;
        }

        if (this->dataPtr->current_state != "READY_TO_DELIVER") {
            res.message = "[FAILURE triggering service] [Current state is not READY_TO_DELIVER]";
            ROS_ERROR_STREAM(res.message);
            res.success = false;
            return true;
        }
    }
    else {
        ROS_ERROR_STREAM("[FAILURE triggering service][The parameter " << parameter << " does not exist]");
        return true;
    }

    res.success = true;
    res.message = "[SUCCESS] [" + this->dataPtr->agv_name + ": as4->" + destination + " triggered]";
    ROS_INFO_STREAM(res.message);

    this->dataPtr->goto_ks_triggered = true;
    // make a request to lock the models to the tray
    gazebo::msgs::GzString lock_msg;
    lock_msg.set_data("lock");
    this->dataPtr->lock_unlock_models_gz_pub->Publish(lock_msg);

    this->dataPtr->kitting_station_name = destination;
    this->dataPtr->ros_node->setParam(parameter, destination);


    return true;
}


bool ROSAGVPlugin::ProcessAs3ToKsServiceCallback(
    std_srvs::Trigger::Request&,
    std_srvs::Trigger::Response& res) {

    std::string destination{};

    if (this->dataPtr->agv_name == "agv1")
        destination = "ks1";
    else if (this->dataPtr->agv_name == "agv2")
        destination = "ks2";
    else if (this->dataPtr->agv_name == "agv3")
        destination = "ks3";
    else if (this->dataPtr->agv_name == "agv4")
        destination = "ks4";
    else {
        ROS_ERROR_STREAM("[ROSAGVPlugin] [Only possible agvs: agv1, agv2, agv3, or agv4]");
        return true;
    }

    gzdbg << "\n";
    gzdbg << "[Service Call] Move AGV to " << destination << "\n";
    gzdbg << "^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^\n";
    gzdbg << "\n";

    std::string current_station{};
    std::string parameter = "/ariac/" + this->dataPtr->agv_name + "/current_station";
    if (this->dataPtr->ros_node->getParam(parameter, current_station)) {
        if (current_station == destination) {
            res.message = "[FAILURE triggering service][" + this->dataPtr->agv_name + " is already at station '" + destination + "']";
            ROS_ERROR_STREAM(res.message);
            res.success = false;
            return true;
        }

        if (current_station != "as3") {
            res.message = "[FAILURE triggering service] [" + this->dataPtr->agv_name + " has to be at station 'as3' to call this service]";
            ROS_ERROR_STREAM(res.message);
            res.success = false;
            return true;
        }

        if (this->dataPtr->current_state != "READY_TO_DELIVER") {
            res.message = "[FAILURE triggering service] [Current state is not READY_TO_DELIVER]";
            ROS_ERROR_STREAM(res.message);
            res.success = false;
            return true;
        }
    }
    else {
        ROS_ERROR_STREAM("[FAILURE triggering service][The parameter " << parameter << " does not exist]");
        return true;
    }

    res.success = true;
    res.message = "[SUCCESS] [" + this->dataPtr->agv_name + ": as3->" + destination + " triggered]";
    ROS_INFO_STREAM(res.message);

    this->dataPtr->goto_ks_triggered = true;
    // make a request to lock the models to the tray
    gazebo::msgs::GzString lock_msg;
    lock_msg.set_data("lock");
    this->dataPtr->lock_unlock_models_gz_pub->Publish(lock_msg);

    this->dataPtr->kitting_station_name = destination;
    this->dataPtr->ros_node->setParam(parameter, destination);


    return true;
}

bool ROSAGVPlugin::ProcessAs2ToKsServiceCallback(
    std_srvs::Trigger::Request&,
    std_srvs::Trigger::Response& res) {



    std::string destination{};

    if (this->dataPtr->agv_name == "agv1")
        destination = "ks1";
    else if (this->dataPtr->agv_name == "agv2")
        destination = "ks2";
    else if (this->dataPtr->agv_name == "agv3")
        destination = "ks3";
    else if (this->dataPtr->agv_name == "agv4")
        destination = "ks4";
    else {
        ROS_ERROR_STREAM("[ROSAGVPlugin] [Only possible agvs: agv1, agv2, agv3, or agv4]");
        return true;
    }

    gzdbg << "\n";
    gzdbg << "[Service Call] Move AGV to " << destination << "\n";
    gzdbg << "^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^\n";
    gzdbg << "\n";

    std::string current_station{};
    std::string parameter = "/ariac/" + this->dataPtr->agv_name + "/current_station";
    if (this->dataPtr->ros_node->getParam(parameter, current_station)) {
        if (current_station == destination) {
            res.message = "[FAILURE triggering service][" + this->dataPtr->agv_name + " is already at station '" + destination + "']";
            ROS_ERROR_STREAM(res.message);
            res.success = false;
            return true;
        }

        if (current_station != "as2") {
            res.message = "[FAILURE triggering service] [" + this->dataPtr->agv_name + " has to be at station 'as2' to call this service]";
            ROS_ERROR_STREAM(res.message);
            res.success = false;
            return true;
        }

        if (this->dataPtr->current_state != "READY_TO_DELIVER") {
            res.message = "[FAILURE triggering service] [Current state is not READY_TO_DELIVER]";
            ROS_ERROR_STREAM(res.message);
            res.success = false;
            return true;
        }
    }
    else {
        ROS_ERROR_STREAM("[FAILURE triggering service][The parameter " << parameter << " does not exist]");
        return true;
    }

    res.success = true;
    res.message = "[SUCCESS] [" + this->dataPtr->agv_name + ": as2->" + destination + " triggered]";
    ROS_INFO_STREAM(res.message);

    this->dataPtr->goto_ks_triggered = true;
    // make a request to lock the models to the tray
    gazebo::msgs::GzString lock_msg;
    lock_msg.set_data("lock");
    this->dataPtr->lock_unlock_models_gz_pub->Publish(lock_msg);

    this->dataPtr->kitting_station_name = destination;
    this->dataPtr->ros_node->setParam(parameter, destination);


    return true;
}


bool ROSAGVPlugin::ProcessAs1ToKsServiceCallback(
    std_srvs::Trigger::Request&,
    std_srvs::Trigger::Response& res) {
    std::string destination{};

    if (this->dataPtr->agv_name == "agv1")
        destination = "ks1";
    else if (this->dataPtr->agv_name == "agv2")
        destination = "ks2";
    else if (this->dataPtr->agv_name == "agv3")
        destination = "ks3";
    else if (this->dataPtr->agv_name == "agv4")
        destination = "ks4";
    else {
        ROS_ERROR_STREAM("[ROSAGVPlugin] [Only possible agvs: agv1, agv2, agv3, or agv4]");
        return true;
    }

    gzdbg << "\n";
    gzdbg << "[Service Call] Move AGV to " << destination << "\n";
    gzdbg << "^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^\n";
    gzdbg << "\n";

    std::string current_station{};
    std::string parameter = "/ariac/" + this->dataPtr->agv_name + "/current_station";
    if (this->dataPtr->ros_node->getParam(parameter, current_station)) {
        if (current_station == destination) {
            res.message = "[FAILURE triggering service][" + this->dataPtr->agv_name + " is already at station '" + destination + "']";
            ROS_ERROR_STREAM(res.message);
            res.success = false;
            return true;
        }

        if (current_station != "as1") {
            res.message = "[FAILURE triggering service] [" + this->dataPtr->agv_name + " has to be at station 'as1' to call this service]";
            ROS_ERROR_STREAM(res.message);
            res.success = false;
            return true;
        }

        if (this->dataPtr->current_state != "READY_TO_DELIVER") {
            res.message = "[FAILURE triggering service] [Current state is not READY_TO_DELIVER]";
            ROS_ERROR_STREAM(res.message);
            res.success = false;
            return true;
        }
    }
    else {
        ROS_ERROR_STREAM("[FAILURE triggering service][The parameter " << parameter << " does not exist]");
        return true;
    }

    res.success = true;
    res.message = "[SUCCESS] [" + this->dataPtr->agv_name + ": as1->" + destination + " triggered]";
    ROS_INFO_STREAM(res.message);

    this->dataPtr->goto_ks_triggered = true;
    // make a request to lock the models to the tray
    gazebo::msgs::GzString lock_msg;
    lock_msg.set_data("lock");
    this->dataPtr->lock_unlock_models_gz_pub->Publish(lock_msg);

    this->dataPtr->kitting_station_name = destination;
    this->dataPtr->ros_node->setParam(parameter, destination);


    return true;
}