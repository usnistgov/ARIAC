#ifndef _MY_CONTACT_PLUGIN_HH_
#define _MY_CONTACT_PLUGIN_HH_

#include <string>

#include <ros/ros.h>
#include <std_srvs/Trigger.h>
#include <tf2_ros/transform_broadcaster.h>

#include <gazebo/common/Plugin.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/physics/PhysicsTypes.hh>
#include <gazebo/sensors/sensors.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/util/system.hh>
#include <nist_gear/ARIAC.hh>
#include <nist_gear/DetectedKittingShipment.h>
#include <nist_gear/DetectKittingShipment.h>
#include <nist_gear/LockUnlockMovableTrayOnAgv.h>
#include "MovableTrayContactPlugin.hh"
#include <std_msgs/String.h>

namespace gazebo {
      /// \brief An example plugin for a contact sensor.

      class GAZEBO_VISIBLE MyContactPlugin : public ModelContactPlugin {
      public: MyContactPlugin();
      public: virtual ~MyContactPlugin();
      public: virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);
      protected: void OnUpdate(const common::UpdateInfo& _info);
      protected: ros::NodeHandle* node_handle;
      protected: void OnContactsReceived(ConstContactsPtr& _msg);
      protected: transport::NodePtr gz_node;

      protected: void ProcessContactingModels();
      protected: void LockUnlockServiceCallback(ConstGzStringPtr& _msg);
      protected: transport::SubscriberPtr lock_unlock_kt1_gz_sub;
      protected: transport::SubscriberPtr lock_unlock_kt2_gz_sub;
      protected: transport::SubscriberPtr lock_unlock_kt3_gz_sub;
      protected: transport::SubscriberPtr lock_unlock_kt4_gz_sub;
      protected: std::vector<physics::JointPtr> fixed_joints;
      protected: std::string movable_tray_name;
      protected: ariac::Kit kit;
      protected:
            /// \brief Parts to ignore (will be published as faulty in tray msgs)
        /// The namespace of the part (e.g. bin7) is ignored.
        /// e.g. if model_name1 is faulty, either bin7|model_name1 or bin6|model_name1 will be considered faulty
            std::vector<std::string> faulty_part_names;
      };
}
#endif