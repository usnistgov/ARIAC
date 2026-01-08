#include <gz/plugin/Register.hh>

#include <ariac_plugins/recording_plugin.hpp>

GZ_ADD_PLUGIN(
  ariac_plugins::RecordingPlugin,
  gz::sim::System,
  ariac_plugins::RecordingPlugin::ISystemPreUpdate,
  ariac_plugins::RecordingPlugin::ISystemConfigure
)

using namespace ariac_plugins;

RecordingPlugin::~RecordingPlugin(){
  if(video_writer_.isOpened()){
    video_writer_.release();
  }
}

void RecordingPlugin::Configure(const gz::sim::Entity &_entity,
                const std::shared_ptr<const sdf::Element> &_sdf,
                gz::sim::EntityComponentManager &_ecm,
                gz::sim::EventManager &_eventMgr)
{
  recorder_name = gz::sim::Model(_entity).Name(_ecm);

  gz_node = std::make_shared<gz::transport::Node>();

  if (!rclcpp::ok()){
    rclcpp::init(0, nullptr);
  }

  if(_sdf->HasElement("recording_width")){
    recording_width = _sdf->Get<int>("recording_width");
  } else {
    recording_width = 1920;
  }

  if(_sdf->HasElement("recording_height")){
    recording_height = _sdf->Get<int>("recording_height");
  } else {
    recording_height = 1080;
  }

  ros_node = rclcpp::Node::make_shared(recorder_name+"_recording_plugin");

  rclcpp::Parameter sim_time("use_sim_time", true);
  ros_node->set_parameter(sim_time);

  current_status.competition_state = CompetitionStates::PREPARING;

  executor = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
  executor->add_node(ros_node);

  auto spin = [this](){
    while(rclcpp::ok()){
      executor->spin_once();
    }
  };

  thread_executor_spin = std::thread(spin);

  tmp_path = recorder_name+".mp4";

  gz_node->Subscribe(
    "/world/ariac/model/"+recorder_name+"/link/link/sensor/rgb_camera/image", 
    &RecordingPlugin::gz_img_callback, 
    this
  );

  competition_status_sub = ros_node->create_subscription<CompetitionStatus>(
    "/competition_status", 10, std::bind(&RecordingPlugin::competition_status_cb, this, std::placeholders::_1)
  );
}

void RecordingPlugin::gz_img_callback(const gz::msgs::Image &_msg)
{
  if(current_status.competition_state != CompetitionStates::STARTED || !video_writer_.isOpened()){return;}

  cv::Mat image(_msg.height(), _msg.width(), CV_8UC3, (void*)_msg.data().data());
  cv::Mat image_bgr;
  cv::cvtColor(image, image_bgr, cv::COLOR_RGB2BGR);
  video_writer_.write(image_bgr);
}

void RecordingPlugin::PreUpdate(const gz::sim::UpdateInfo &_info,
                gz::sim::EntityComponentManager &_ecm)
{
}

void RecordingPlugin::competition_status_cb(const CompetitionStatus::SharedPtr msg){
  if (msg->competition_state == CompetitionStates::STARTED && !video_writer_.isOpened()) {
    video_writer_.open(tmp_path, cv::VideoWriter::fourcc('m', 'p', '4', 'v'), 30, cv::Size(recording_width, recording_height));
  }
  if (msg->competition_state == CompetitionStates::ENDED && video_writer_.isOpened()) {
    video_writer_.release();
    rename_recording();
    saved = true;
  }
  current_status.competition_state = msg->competition_state;
  current_status.run_id = msg->run_id;
}


void RecordingPlugin::rename_recording() {
  std::string goal_path = "/tmp/"+recorder_name+"_run_id_"+std::to_string(current_status.run_id)+".mp4";
  gzmsg << "Moving mp4 to target of " + goal_path + " from " + tmp_path;
  try {
    std::filesystem::rename(tmp_path, goal_path);
  } catch (const std::filesystem::filesystem_error& e) {
    throw std::runtime_error("Could not move file from " + tmp_path + " to " + goal_path);
  }
}