#include "nist_gear/MyContactPlugin.hh"

using namespace gazebo;
GZ_REGISTER_MODEL_PLUGIN(MyContactPlugin)

/////////////////////////////////////////////////
MyContactPlugin::MyContactPlugin() : ModelContactPlugin()
{
}

/////////////////////////////////////////////////
MyContactPlugin::~MyContactPlugin()
{
    this->updateConnection.reset();
    this->world.reset();
}

/////////////////////////////////////////////////
void MyContactPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
    ModelContactPlugin::Load(_model, _sdf);

    if (this->updateRate > 0)
        gzdbg << "MyContactPlugin running at " << this->updateRate << " Hz\n";
    else
        gzdbg << "MyContactPlugin running at the default update rate\n";

    // Make sure the ROS node for Gazebo has already been initialized
    if (!ros::isInitialized()) {
        ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized, unable to load plugin. "
            << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
        return;
    }
    this->node_handle = new ros::NodeHandle("");

    // Initialize Gazebo transport
    this->gz_node = transport::NodePtr(new transport::Node());
    this->gz_node->Init();

    ///////////////////////
    //Process ariac.world
    
    std::string lock_unlock_service_str{};
    if (_sdf->HasElement("lock_unlock_kt1_topic")) {
        lock_unlock_service_str = _sdf->Get<std::string>("lock_unlock_kt1_topic");

        this->lock_unlock_kt1_gz_sub =
            this->gz_node->Subscribe(lock_unlock_service_str,
                &MyContactPlugin::LockUnlockServiceCallback,
                this);
    }
    if (_sdf->HasElement("lock_unlock_kt2_topic")) {
        lock_unlock_service_str = _sdf->Get<std::string>("lock_unlock_kt2_topic");

        this->lock_unlock_kt2_gz_sub =
            this->gz_node->Subscribe(lock_unlock_service_str,
                &MyContactPlugin::LockUnlockServiceCallback,
                this);
    }
    if (_sdf->HasElement("lock_unlock_kt3_topic")) {
        lock_unlock_service_str = _sdf->Get<std::string>("lock_unlock_kt3_topic");

        this->lock_unlock_kt3_gz_sub =
            this->gz_node->Subscribe(lock_unlock_service_str,
                &MyContactPlugin::LockUnlockServiceCallback,
                this);
    }
    if (_sdf->HasElement("lock_unlock_kt4_topic")) {
        lock_unlock_service_str = _sdf->Get<std::string>("lock_unlock_kt4_topic");

        this->lock_unlock_kt4_gz_sub =
            this->gz_node->Subscribe(lock_unlock_service_str,
                &MyContactPlugin::LockUnlockServiceCallback,
                this);
    }

    if (_sdf->HasElement("model_name")) {
        std::string model_name_str = _sdf->Get<std::string>("model_name");

        this->movable_tray_name = model_name_str;
    }
}

/////////////////////////////////////////////////
void MyContactPlugin::LockUnlockServiceCallback(ConstGzStringPtr& _msg)
{
    if (_msg->data() == "lock") {
        // this->LockContactingModels();
    }
    else if (_msg->data() == "unlock") {
        // this->UnlockContactingModels();
    }

}


/////////////////////////////////////////////////
void MyContactPlugin::ProcessContactingModels()
{
    // Make sure that models fixed to the tray are included in the contacting models,
    // even if they aren't contacting the tray anymore.
    for (auto tray_models_joint : this->fixed_joints) {
        auto link = tray_models_joint->GetChild();
        this->contactingLinks.insert(link);
        this->contactingModels.insert(link->GetParentModel());
    }
    this->kit.products.clear();

    auto tray_world_pose = this->model->WorldPose();
    gzerr << "-----POSE------ " << "\n";
    gzerr << tray_world_pose << "\n";
    ROS_INFO_STREAM("POSE: " << tray_world_pose << std::endl);
    for (auto contacting_model : this->contactingModels) {
        contacting_model->SetAutoDisable(false);
        ariac::KitObject kit_object;


        // Determine the object type
        kit_object.type = ariac::DetermineModelType(contacting_model->GetName());
        // gzwarn << "[OBJECT TYPE] -> " << object.type << std::endl;

        // Determine if the object is faulty
        auto contacting_model_name = ariac::TrimNamespace(contacting_model->GetName());
        auto it = std::find(this->faulty_part_names.begin(), this->faulty_part_names.end(), contacting_model_name);
        kit_object.isFaulty = it != this->faulty_part_names.end();

        // Determine the pose of the object in the frame of the movable tray
        ignition::math::Pose3d kit_object_pose = contacting_model->WorldPose();
        ignition::math::Matrix4d transMat(tray_world_pose);
        ignition::math::Matrix4d objectPoseMat(kit_object_pose);
        kit_object.pose = (transMat.Inverse() * objectPoseMat).Pose();
        kit_object.pose.Rot().Normalize();

        // if this object is not a kit tray, push it to the list of products
        if (kit_object.type.compare("kit_tray") != 0) {
            this->kit.products.push_back(kit_object);
        }
        this->kit.movable_tray.type = this->movable_tray_name;
        // gzerr << "Movable tray type: " << this->currentKit.movable_tray.type << std::endl;
        this->kit.movable_tray.name = contacting_model_name;
        // gzerr << "Movable tray name: " << this->currentKit.movable_tray.name << std::endl;
        // this->currentKit.movable_tray.pose = this->model->WorldPose();

        gzerr << "-----MODEL------ " << contacting_model_name << "\n";
        gzerr << "-----TYPE------ " << kit_object.type << "\n";
        if (kit_object.type.compare("kit_tray") == 0) {
            this->kit.kit_tray_name = contacting_model_name;
            // Determine the pose of the movable tray in the frame of the kit tray
            ignition::math::Pose3d movable_tray_pose = this->model->WorldPose();
            ignition::math::Matrix4d transform_mat(contacting_model->WorldPose());
            ignition::math::Matrix4d movable_tray_pose_mat(movable_tray_pose);
            movable_tray_pose = (transform_mat.Inverse() * movable_tray_pose_mat).Pose();
            movable_tray_pose.Rot().Normalize();
            this->kit.movable_tray.pose = movable_tray_pose;
        }
    }
}

/////////////////////////////////////////////////
void MyContactPlugin::OnUpdate(const common::UpdateInfo& _info)
{
    gzdbg << "ProcessContactingModels\n";
}

void MyContactPlugin::OnContactsReceived(ConstContactsPtr& _msg)
{
    gzdbg << "ProcessContactingModels\n";
}