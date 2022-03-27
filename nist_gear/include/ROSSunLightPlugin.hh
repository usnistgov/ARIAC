/**
 * @file ROSStackLightsPlugin.hh
 * @author Zeid Kootbally (zeid.kootbally@nist.gov)
 * @brief
 * @version 0.1
 * @date 2021-12-13
 *
 * @copyright Copyright (c) 2021
 *
 */

#ifndef _ROS_STACKLIGHTS_PLUGIN_HH_
#define _ROS_STACKLIGHTS_PLUGIN_HH_

#include <ros/ros.h>
#include <gazebo/common/Plugin.hh>

namespace gazebo {
    // Forward declare private data class
    class SunLightPluginPrivate;

    typedef enum {
        NORMAL = 0, RED, YELLOW, GREEN, BLUE, ORANGE
    } SunColor;

    /**
     * @brief Struct to store private data
     *
     */
    struct SunLightPluginPrivate
    {
    public:
         //! World pointer
        physics::WorldPtr s_world_ptr;

         //! SDF pointer
        sdf::ElementPtr s_element_ptr;

         //! Mutex to avoid race conditions
        std::mutex s_mutex;
        
        //! Node handle
        std::unique_ptr<ros::NodeHandle> s_nh;

        //! Namespace used for the plugin
        std::string s_robot_ns;

        //! Topic to listen for sun color changes
        std::string s_sun_topic;
    };

    class GAZEBO_VISIBLE SunLightPlugin : public WorldPlugin {
    public:
        /**
         * @brief Construct a new Stack Lights Plugin object
         *
         */
        explicit SunLightPlugin();
        /**
         * @brief Destroy the Stack Lights Plugin object
         *
         */
        virtual ~SunLightPlugin();
        bool getStatus();
        void setStatus(bool status);
        void toggleLight();
        //overriden methods
        virtual void Load(physics::WorldPtr world_ptr, sdf::ElementPtr sdf_ptr) override;
        //! Called by the world update start event
        virtual void OnUpdate();

    private:
        void change_lighting(int color, bool flashing);
        ros::NodeHandle m_nh;
        ros::Subscriber m_sun_sub;
        ros::Publisher m_sun_pub;
        transport::NodePtr m_node_ptr;
        //! Publisher on the ~/light topic
        transport::PublisherPtr m_light_pub;
        msgs::Light m_light_msg;
        virtual void Publish() const;
        bool m_status;
        std::unique_ptr<SunLightPluginPrivate> m_data_ptr;
        sdf::SDF m_blue_sun;
        sdf::SDF m_red_sun;
        sdf::SDF m_green_sun;

        //! Pointer to the update event connection
        event::ConnectionPtr m_update_connection;

    };//end class


} //end namespace
#endif