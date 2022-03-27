#include <memory>
#include <string>
#include <ros/ros.h>
#include <sdf/sdf.hh>
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
#include <std_msgs/String.h>
#include <memory>
#include "ROSSunLightPlugin.hh"


namespace gazebo {

    //! @brief Construct a new gz register world plugin object
    GZ_REGISTER_WORLD_PLUGIN(SunLightPlugin);

    /**
     * @brief Construct a new Sun Light Plugin:: Sun Light Plugin object
     * 
     */
    SunLightPlugin::SunLightPlugin() : WorldPlugin(),
        m_data_ptr{ new SunLightPluginPrivate },
        m_status{true}
    {
        
    }

    /**
     * @brief Destroy the Sun Light Plugin:: Sun Light Plugin object
     * 
     */
    SunLightPlugin::~SunLightPlugin() {
        this->m_data_ptr->s_nh->shutdown();
    }

    bool SunLightPlugin::getStatus() {
        return m_status;
    }

    void SunLightPlugin::setStatus(bool status) {
        m_status = status;
    }

    void SunLightPlugin::change_lighting(int color, bool flashing) {
        if (color == BLUE) {
            
        }
    }


    void SunLightPlugin::Load(physics::WorldPtr _world_ptr, sdf::ElementPtr _sdf_ptr) {
        GZ_ASSERT(_world_ptr, "SunLightPlugin world pointer is NULL");
        GZ_ASSERT(_sdf_ptr, "SunLightPlugin sdf pointer is NULL");

        if (!ros::isInitialized()) {
            ROS_FATAL("A ROS Node for Gazebo has not been initialized, unable to load plugin SunLightPlugin");
            return;
        }

        ROS_INFO("*** SunLightPlugin loaded ***");


        m_data_ptr->s_world_ptr = _world_ptr;
        m_data_ptr->s_element_ptr = _sdf_ptr;

        // get the namespace from ariac.world
        m_data_ptr->s_robot_ns = "";
        if (_sdf_ptr->HasElement("robotNamespace")) {
            this->m_data_ptr->s_robot_ns = _sdf_ptr->GetElement(
                "robotNamespace")
                ->Get<std::string>() +
                "/";
            ROS_WARN_STREAM("robotNamespace: " << m_data_ptr->s_robot_ns << "\n");
        }

        m_data_ptr->s_sun_topic = "";
        if (_sdf_ptr->HasElement("sun_change_topic")) {
            this->m_data_ptr->s_sun_topic = _sdf_ptr->GetElement("sun_change_topic")->Get<std::string>();
        }

        
        // initialize the node handle
        m_data_ptr->s_nh = std::make_unique<ros::NodeHandle>(ros::NodeHandle(m_data_ptr->s_robot_ns));



        m_node_ptr = transport::NodePtr(new transport::Node());
        m_light_pub = m_node_ptr->Advertise<msgs::Light>("~/factory/light");
        m_sun_pub = m_nh.advertise<std_msgs::String>(m_data_ptr->s_sun_topic, 1, true);

        m_node_ptr->Init(_world_ptr->Name());
        m_data_ptr->s_world_ptr->ModelByName("sun");

        // called on every sim iteration
        m_update_connection = event::Events::ConnectWorldUpdateBegin(
            std::bind(&SunLightPlugin::OnUpdate, this));

        

        
        m_blue_sun.SetFromString("<sdf version = '1.5'>\
            <light type='directional' name='sun'>\
            <cast_shadows>true</cast_shadows>\
            <pose>0 0 10 0 0 0 </pose>\
            <diffuse>0.0 0.0 0.30 1 </diffuse>\
            <specular>0.2 0.2 0.2 1 </specular>\
            <attenuation>\
            <range>1000 </range>\
            <constant>0.9 </constant>\
            <linear>0.01 </linear>\
            <quadratic>0.001 </quadratic>\
            </attenuation>\
            <direction>-0.5 0.1 - 0.9 </direction>\
            </light>\
            </sdf>");

        sdf::ElementPtr light = m_blue_sun.Root()->GetElement("light");
        m_light_msg = gazebo::msgs::LightFromSDF(light);
        // m_light_pub->Publish(m_light_msg);
    }


    void SunLightPlugin::OnUpdate() {
        if (!m_status)
            m_data_ptr->s_world_ptr->RemoveModel("sun");
    }

    void SunLightPlugin::Publish() const
    {
    }

     
}









