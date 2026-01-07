#ifndef ARIAC_PLUGINS__ARIAC_RECORDING_PLUGIN_HPP_
#define ARIAC_PLUGINS__ARIAC_RECORDING_PLUGIN_HPP_

#include <gz/transport/Node.hh>
#include <gz/msgs/image.pb.h>
#include <gz/sim/System.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/EventManager.hh>

#include <cstdlib>
#include <string>
#include <cstdio>
#include <filesystem>

#include <rclcpp/rclcpp.hpp>
#include <ariac_interfaces/msg/competition_states.hpp>
#include <ariac_interfaces/msg/competition_status.hpp>

using CompetitionStates = ariac_interfaces::msg::CompetitionStates;
using CompetitionStatus = ariac_interfaces::msg::CompetitionStatus;

#include <opencv2/opencv.hpp>

namespace ariac_plugins
{
  class RecordingPlugin:
    public gz::sim::System,
    public gz::sim::ISystemConfigure,
    public gz::sim::ISystemPreUpdate
  {
    public:
      ~RecordingPlugin();
      void Configure (
        const gz::sim::Entity &_entity,
        const std::shared_ptr<const sdf::Element> &_sdf,
        gz::sim::EntityComponentManager &_ecm,
        gz::sim::EventManager &_event_manager) override;
      
      void PreUpdate(const gz::sim::UpdateInfo &_info, gz::sim::EntityComponentManager &_ecm) final;

    private: 
      void gz_img_callback(const gz::msgs::Image &);
      void competition_status_cb(const CompetitionStatus::SharedPtr msg);
      void rename_recording();

      std::string recorder_name;
      std::string tmp_path;

      bool saved = false;

      int recording_width, recording_height;

      std::shared_ptr<gz::transport::Node> gz_node;
      cv::VideoWriter video_writer_;

      rclcpp::Node::SharedPtr ros_node;
      rclcpp::executors::MultiThreadedExecutor::SharedPtr executor;
      std::thread thread_executor_spin;
      rclcpp::Subscription<CompetitionStatus>::SharedPtr competition_status_sub;
    
      CompetitionStatus current_status;
  };
}

#endif // ARIAC_PLUGINS__ARIAC_RECORDING_PLUGIN_HPP