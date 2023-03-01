/*
This software was developed by employees of the National Institute of Standards and Technology (NIST), an agency of the Federal Government. Pursuant to title 17 United States Code Section 105, works of NIST employees are not subject to copyright protection in the United States and are considered to be in the public domain. Permission to freely use, copy, modify, and distribute this software and its documentation without fee is hereby granted, provided that this notice and disclaimer of warranty appears in all copies.

The software is provided 'as is' without any warranty of any kind, either expressed, implied, or statutory, including, but not limited to, any warranty that the software will conform to specifications, any implied warranties of merchantability, fitness for a particular purpose, and freedom from infringement, and any warranty that the documentation will conform to the software, or any warranty that the software will be error free. In no event shall NIST be liable for any damages, including, but not limited to, direct, indirect, special or consequential damages, arising out of, resulting from, or in any way connected with this software, whether or not based upon warranty, contract, tort, or otherwise, whether or not injury was sustained by persons or property or otherwise, and whether or not loss was sustained from, or arose out of the results of, or use of, the software or services provided hereunder.

Distributions of NIST software should also include copyright and licensing statements of any third-party software that are legally bundled with the code in compliance with the conditions of those licenses.
*/

#ifndef ARIAC_PLUGINS__ARIAC_COMMON_HPP_
#define ARIAC_PLUGINS__ARIAC_COMMON_HPP_

// C++
#include <ostream>
#include <map>
#include <string>
#include <vector>
#include <memory>
// ROS
#include <geometry_msgs/msg/pose.hpp>
// Gazebo
#include <gazebo/gazebo.hh>
#include <ignition/math/Pose3.hh>
#include <ignition/math/Vector3.hh>
// KDL and TF2
#include <tf2_kdl/tf2_kdl.h>
#include <tf2/convert.h>
#include <kdl/frames.hpp>
// Messages
#include <ariac_msgs/msg/order.hpp>
#include <ariac_msgs/msg/condition.hpp>
#include <ariac_msgs/msg/challenge.hpp>
#include <ariac_msgs/msg/part.hpp>
#include <ariac_msgs/msg/assembly_task.hpp>
#include <ariac_msgs/msg/kitting_task.hpp>
#include <ariac_msgs/msg/combined_task.hpp>
#include <ariac_msgs/msg/kitting_part.hpp>
#include <ariac_msgs/msg/assembly_part.hpp>
#include <geometry_msgs/msg/vector3.hpp>

/**
 * @brief Namespace for common functions and classes
 *
 */
namespace ariac_common
{

    // forward declarations
    class Order;
    class Score;
    class OrderTemporal;
    class Part;
    class Quadrant;
    
    class AssemblyScore;
    class CombinedScore;
    class KittingScore;
    class ScoredAssemblyPart;

    //==============================================================================
    /**
     * @brief Helper function to convert a part type to a string
     *
     * @param part_type  Part type as an unsigned int
     * @return std::string  Part type as a string
     */
    std::string static ConvertPartTypeToString(unsigned int part_type)
    {
        if (part_type == ariac_msgs::msg::Part::BATTERY)
            return "battery";
        else if (part_type == ariac_msgs::msg::Part::PUMP)
            return "pump";
        else if (part_type == ariac_msgs::msg::Part::REGULATOR)
            return "regulator";
        else if (part_type == ariac_msgs::msg::Part::SENSOR)
            return "sensor";
        else
            return "unknown";
    }

    //==============================================================================
    /**
     * @brief Helper function to convert an assembly station to a string
     *
     * @param station_id  Assembly station as an unsigned int
     * @return std::string  Assembly station as a string
     */
    std::string static ConvertAssemblyStationToString(unsigned int station_id)
    {
        if (station_id == ariac_msgs::msg::AssemblyTask::AS1)
            return "as1";
        else if (station_id == ariac_msgs::msg::AssemblyTask::AS2)
            return "as2";
        else if (station_id == ariac_msgs::msg::AssemblyTask::AS3)
            return "as3";
        else if (station_id == ariac_msgs::msg::AssemblyTask::AS4)
            return "as4";
        else
            return "unknown";
    }

    //==============================================================================
    /**
     * @brief Helper function to convert a destination to a string
     *
     * @param destination  Destination as an unsigned int
     * @param agv_number  AGV number as an unsigned int
     * @return std::string  Destination as a string
     */
    std::string static ConvertDestinationToString(unsigned int destination, unsigned int agv_number)
    {
        if (agv_number == 1 || agv_number == 2)
        {
            if (destination == ariac_msgs::msg::KittingTask::ASSEMBLY_FRONT)
                return "as1";
            else if (destination == ariac_msgs::msg::KittingTask::ASSEMBLY_BACK)
                return "as2";
        }
        else if (agv_number == 3 || agv_number == 4)
        {
            if (destination == ariac_msgs::msg::KittingTask::ASSEMBLY_FRONT)
                return "as3";
            else if (destination == ariac_msgs::msg::KittingTask::ASSEMBLY_BACK)
                return "as4";
        }
        if (destination == ariac_msgs::msg::KittingTask::KITTING)
            return "kitting";
        else if (destination == ariac_msgs::msg::KittingTask::WAREHOUSE)
            return "warehouse";
        else
            return "unknown";
    }

    //==============================================================================
    /**
     * @brief Helper function to convert a part color to a string
     *
     * @param part_color  Part color as an unsigned int
     * @return std::string  Part color as a string
     */
    std::string static ConvertPartColorToString(unsigned int part_color)
    {
        if (part_color == ariac_msgs::msg::Part::RED)
            return "red";
        else if (part_color == ariac_msgs::msg::Part::GREEN)
            return "green";
        else if (part_color == ariac_msgs::msg::Part::BLUE)
            return "blue";
        else if (part_color == ariac_msgs::msg::Part::PURPLE)
            return "purple";
        else if (part_color == ariac_msgs::msg::Part::ORANGE)
            return "orange";
        else
            return "unknown";
    }

    //==============================================================================
    /**
     * @brief Helper function to convert an order type to a string
     *
     * @param order_type Integer representing the order type
     * @return std::string Type of order as a string
     */
    std::string static ConvertOrderTypeToString(unsigned int order_type)
    {
        if (order_type == ariac_msgs::msg::Order::ASSEMBLY)
            return "assembly";
        else if (order_type == ariac_msgs::msg::Order::KITTING)
            return "kitting";
        else if (order_type == ariac_msgs::msg::Order::COMBINED)
            return "combined";
        else
            return "unknown";
    }

    //==============================================================================
    /**
     * @brief Class to store the score and penalty for an order
     *
     */
    class Score
    {
    public:
        Score() : score_(0), penalty_(0) {}

        /**
         * @brief Increase the score for an order
         *
         * @param score  Score to add to the order
         */
        void AddScore(double score) { score_ += score; }
        /**
         * @brief Increase the penalty for an order
         *
         * @param penalty  Penalty to add to the order
         */
        void AddPenalty(double penalty) { penalty_ += penalty; }

        /**
         * @brief Get the score for an order
         *
         * @return double  Score for the order
         */
        double GetScore() const { return score_; }
        /**
         * @brief Get the penalty for an order
         *
         * @return double  Penalty for the order
         */
        double GetPenalty() const { return penalty_; }

        /**
         * @brief Overload the << operator to print the score and penalty
         *
         * @param out  Output stream
         * @param obj  Score object
         * @return std::ostream&  Output stream
         */
        friend std::ostream &operator<<(std::ostream &out,
                                        const Score &obj)
        {
            out << "Score: " << obj.score_ << " Penalty: " << obj.penalty_;

            return out;
        }

    private:
        //! Score of the order
        double score_;
        //! Penalty for having extra parts in the order
        double penalty_;
    };

    //==============================================================================
    /**
     * @brief Class to store the part information
     *
     */
    class Part
    {
    public:
        /**
         * @brief Construct a new Part object
         *
         * @param color Color of the part
         * @param type Type of the part
         */
        Part(unsigned int color, unsigned int type) : color_(color), type_(type) {}

        /**
         * @brief Overload the << operator to print the part information
         *
         * @param out  Output stream
         * @param obj  Part object
         * @return std::ostream&  Output stream
         */
        friend std::ostream &operator<<(std::ostream &out,
                                        const Part &obj)
        {
            out << "[" << ConvertPartTypeToString(obj.type_) << "," << ConvertPartColorToString(obj.color_) << "]";

            return out;
        }
        // void Print() const
        // {
        //     std::cout << "[" << ConvertPartTypeToString(type_) << "," << ConvertPartColorToString(color_) << "]" << std::endl;
        // }
        /**
         * @brief Get the color of the part
         *
         * @return unsigned int  Color of the part
         */
        unsigned int GetColor() const { return color_; }
        /**
         * @brief Get the type of the part
         *
         * @return unsigned int  Type of the part
         */
        unsigned int GetType() const { return type_; }

        

    private:
        //! Color of the part
        unsigned int color_;
        //! Type of the part
        unsigned int type_;
    };

    //==============================================================================
    /**
     * @brief Class to store the part information for a kitting task
     *
     */
    class KittingPart
    {
    public:
        /**
         * @brief Construct a new KittingPart object
         *
         * @param quadrant Quadrant of the part on the AGV
         * @param part  Part object
         */
        KittingPart(unsigned quadrant, const Part &part) : quadrant_(quadrant), part_(part) {}

        /**
         * @brief Get the Quadrant object
         *
         * @return unsigned int  Quadrant of the part on the AGV
         */
        unsigned int GetQuadrant() const { return quadrant_; }

        /**
         * @brief Get the Part object
         *
         * @return Part  Part object
         */
        Part GetPart() const { return part_; }

        /**
         * @brief Overload the << operator to print the part information
         *
         * @param out  Output stream
         * @param obj  KittingPart object
         * @return std::ostream&  Output stream
         */
        friend std::ostream &operator<<(std::ostream &out,
                                        const KittingPart &obj)
        {
            out << "   ------" << std::endl;
            out << "   Part: " << obj.part_ << std::endl;
            out << "   Quadrant: " << obj.quadrant_;
            return out;
        }

    private:
        //! Quadrant of the part on the AGV
        unsigned int quadrant_;
        //! Part object
        Part part_;
    };

    //==============================================================================
    class AssemblyPart
    {
    public:
        /**
         * @brief Construct a new Assembly Part object
         *
         * @param part  Part object
         * @param part_pose  Pose of the part in the insert
         * @param part_direction  Direction of the part in the insert
         */
        AssemblyPart(const Part &part,
                     const ignition::math::Pose3d &part_pose,
                     const ignition::math::Vector3<double> &part_direction) : part_(part),
                                                                              part_pose_(part_pose),
                                                                              part_direction_(part_direction) {}

        friend std::ostream &operator<<(std::ostream &out,
                                        const AssemblyPart &obj)
        {
            out << "   ------" << std::endl;
            out << "   Part: " << obj.part_ << std::endl;
            out << "   Assembled Pose: [" << obj.part_pose_.Pos().X() << ","
                << obj.part_pose_.Pos().Y() << "," << obj.part_pose_.Pos().Z() << "]"
                << "[" << obj.part_pose_.Rot().X() << "," << obj.part_pose_.Rot().Y() << ","
                << obj.part_pose_.Rot().Z() << "," << obj.part_pose_.Rot().W() << "]" << std::endl;
            out << "   Assembled Direction: [" << obj.part_direction_[0] << ","
                << obj.part_direction_[1] << "," << obj.part_direction_[2] << "]";

            return out;
        }

        /**
         * @brief Get the Part object
         *
         * @return Part
         */
        Part GetPart() const { return part_; }
        /**
         * @brief Get the Part Pose object
         *
         * @return ignition::math::Pose3d
         */
        ignition::math::Pose3d GetPartPose() const { return part_pose_; }
        /**
         * @brief Get the Part Direction object
         *
         * @return ignition::math::Vector3<double>
         */
        ignition::math::Vector3<double> GetPartDirection() const { return part_direction_; }

    private:
        /**
         * @brief Part object
         *
         */
        Part part_;
        /**
         * @brief Pose of the part in the insert
         *
         */
        ignition::math::Pose3d part_pose_;
        /**
         * @brief Direction of the part in the insert
         *
         */
        ignition::math::Vector3<double> part_direction_;
    };

    //==============================================================================
    /**
     * @brief Class to represent an Assemly Task
     *
     */
    class AssemblyTask
    {
    public:
        /**
         * @brief Construct a new Assembly Task object
         *
         * @param agv_numbers  AGV(s) where parts can be found to do assembly
         * @param station  Assembly where assembly will be done
         * @param products  Products to be assembled
         */
        AssemblyTask(std::vector<unsigned int> agv_numbers,
                     unsigned int station,
                     const std::vector<AssemblyPart> &products) : agv_numbers_(agv_numbers),
                                                                  station_(station),
                                                                  products_(products) {}

        /**
         * @brief Overload of the << operator to print the Assembly Task object
         *
         * @param out  Output stream
         * @param obj  Assembly Task object
         * @return std::ostream&  Output stream
         */
        friend std::ostream &operator<<(std::ostream &out,
                                        const AssemblyTask &obj)
        {
            out << "   Assembly Task" << std::endl;
            out << "   ================" << std::endl;

            if (obj.agv_numbers_.size() == 1)
                out << "   AGV: [" << obj.agv_numbers_[0] << "]" << std::endl;
            else
            {
                int counter = obj.agv_numbers_.size();

                out << "   AGV: [";
                for (auto agv_number : obj.agv_numbers_)
                {
                    counter--;
                    out << agv_number;
                    if (counter > 0)
                        out << ",";
                }

                out << "]" << std::endl;
            }

            // stations
            out << "   Station: " << ConvertAssemblyStationToString(obj.station_) << std::endl;

            // Products
            out << "   ================" << std::endl;
            out << "   Products: " << std::endl;
            for (const auto &product : obj.products_)
            {
                out << product << std::endl;
            }

            return out;
        }

        /**
         * @brief Get the Agv Numbers object
         *
         * @return const std::vector<unsigned int>&
         */
        const std::vector<unsigned int> &GetAgvNumbers() const { return agv_numbers_; }
        /**
         * @brief Get the Station object
         *
         * @return unsigned int
         */
        unsigned int GetStation() const { return station_; }
        /**
         * @brief Get the Products object
         *
         * @return const std::vector<AssemblyPart>&
         */
        const std::vector<AssemblyPart> &GetProducts() const { return products_; }

    private:
        /**
         * @brief AGV(s) where parts can be found to do assembly
         *
         */
        std::vector<unsigned int> agv_numbers_;
        /**
         * @brief Assembly where assembly will be done
         *
         */
        unsigned int station_;
        /**
         * @brief Products to be assembled
         *
         */
        std::vector<AssemblyPart> products_;
    };

    //==============================================================================
    /**
     * @brief Class to represent a Kitting task
     */
    class KittingTask
    {
    public:
        KittingTask(unsigned int agv_number,
                    unsigned int tray_id,
                    unsigned int destination,
                    const std::vector<KittingPart> &products) : agv_number_(agv_number),
                                                                tray_id_(tray_id),
                                                                destination_(destination),
                                                                products_(products) {}

        /**
         * @brief Overload of the << operator to print the Kitting Task object
         *
         * @param out  Output stream
         * @param obj  Kitting Task object
         * @return std::ostream& Output stream
         */
        friend std::ostream &operator<<(std::ostream &out,
                                        const KittingTask &obj)
        {
            out << "   Kitting Task" << std::endl;
            out << "   ================" << std::endl;

            out << "   AGV: " << obj.agv_number_ << std::endl;
            out << "   Tray ID: " << obj.tray_id_ << std::endl;

            // Destination
            out << "   Destination: " << ConvertDestinationToString(obj.destination_, obj.agv_number_) << std::endl;

            // Products
            out << "   ================" << std::endl;
            out << "   Products: " << std::endl;
            for (const auto &product : obj.products_)
            {
                out << product << std::endl;
            }

            return out;
        }

        /**
         * @brief Get the Agv Number
         *
         * @return unsigned int
         */
        unsigned int GetAgvNumber() const { return agv_number_; }
        /**
         * @brief Get the Tray Id
         *
         * @return unsigned int
         */
        unsigned int GetTrayId() const { return tray_id_; }
        /**
         * @brief Get the Destination
         *
         * @return unsigned int
         */
        unsigned int GetDestination() const { return destination_; }
        /**
         * @brief Get the Products
         *
         * @return const std::vector<KittingPart>&
         */
        const std::vector<KittingPart> &GetProducts() const { return products_; }

    private:
        /**
         * @brief AGV where parts can be found to do kitting
         *
         */
        unsigned int agv_number_;
        /**
         * @brief Tray ID where parts can be found to do kitting
         *
         */
        unsigned int tray_id_;
        /**
         * @brief Destination where kitting will be done
         *
         */
        unsigned int destination_;
        /**
         * @brief Products to be kitted
         *
         */
        std::vector<KittingPart> products_;
    };

    //==============================================================================
    /**
     * @brief Class to represent a Combined task
     */
    class CombinedTask
    {
    public:
        /**
         * @brief Construct a new Combined Task object
         *
         * @param station  Assembly station where assembly will be done
         * @param products  Products to be assembled
         */
        CombinedTask(unsigned int station,
                     const std::vector<AssemblyPart> &products) : station_(station),
                                                                  products_(products) {}

        /**
         * @brief Get the Station object
         *
         * @return unsigned int
         */
        unsigned int GetStation() const { return station_; }
        /**
         * @brief Get the Products object
         *
         * @return const std::vector<AssemblyPart>&
         */
        const std::vector<AssemblyPart> &GetProducts() const { return products_; }

        /**
         * @brief Overload of the << operator to print the Combined Task object
         *
         * @param out  Output stream
         * @param obj  Combined Task object
         * @return std::ostream& Output stream
         */
        friend std::ostream &operator<<(std::ostream &out,
                                        const CombinedTask &obj)
        {
            out << "   Combined Task" << std::endl;
            out << "   ================" << std::endl;

            // stations
            out << "   Station: " << ConvertAssemblyStationToString(obj.station_) << std::endl;

            // Products
            out << "   ================" << std::endl;
            out << "   Products: " << std::endl;
            for (const auto &product : obj.products_)
            {
                out << product << std::endl;
            }

            return out;
        }

    private:
        /**
         * @brief Assembly station where assembly will be done
         *
         */
        unsigned int station_;
        /**
         * @brief Products to be assembled
         *
         */
        std::vector<AssemblyPart> products_;
    };

    //==============================================================================
    /**
     * @brief Base class for the human challenge.
     *
     */
    class HumanChallenge
    {
    public:
        /**
         * @brief Construct a new Human challenge object
         *
         * @param behavior The behavior of the human during the trial
         */
        HumanChallenge(unsigned behavior) : behavior_(behavior) {}

        /**
         * @brief Get the type of the challenge
         *
         * @return unsigned int
         */
        unsigned int GetType() const { return type_; }

        /**
         * @brief Get the behavior of the human during the trial
         *
         * @return unsigned int
         */
        unsigned int GetBehavior() const { return behavior_; }

        /**
         * @brief Get the time penalty for the ceiling robot
         *
         * @return double
         */
        double GetRobotTimePenalty() const { return robot_time_penalty_; }
        /**
         * @brief Set the start time of this challenge for logging purposes
         *
         * @param _start_time
         */
        void SetStartTime(double start_time) { start_time_ = start_time; }

        /**
         * @brief Check if the challenge has started
         *
         * @return bool True if the challenge has started and false otherwise
         */
        bool HasStarted() const { return started_; }

        /**
         * @brief Set the started flag to true
         *
         */
        void SetStarted() { started_ = true; }

    protected:
        // Behavior of the human during the trial
        unsigned int behavior_;
        // Time penalty for the ceiling robot
        double robot_time_penalty_{15};
        const unsigned int type_ = ariac_msgs::msg::Challenge::HUMAN;
        // Whether the challenge has started
        bool started_ = false;
        // simulation time when the challenge started
        double start_time_;
    };

    //==============================================================================
    /**
     * @brief Class which derives HumanChallenge
     *
     * This challenge is triggered at a specific simulation time.
     */
    class HumanChallengeTemporal : public HumanChallenge
    {
    public:
        /**
         * @brief Construct a new Human challenge object
         *
         * @param behavior The behavior of the human during the trial
         * @param trigger_time Time at which the challenge should be triggered
         */
        HumanChallengeTemporal(unsigned behavior, double _trigger_time) : HumanChallenge(behavior),
                                                                          trigger_time_(_trigger_time) {}

        /**
         * @brief Get the time at which the challenge should be triggered
         * @return double Time at which the challenge should be triggered
         */
        double GetTriggerTime() const { return trigger_time_; }

    private:
        //! Simulation time at which the challenge should be triggered
        double trigger_time_;
    };

    //==============================================================================
    /**
     * @brief Class which derives HumanChallenge
     *
     * This challenge is triggered when a part is placed on a specific AGV.
     */
    class HumanChallengeOnPartPlacement : public HumanChallenge
    {
    public:
        /**
         * @brief Construct a new Human challenge object
         *
         * @param behavior The behavior of the human during the trial
         * @param _part Part to trigger the challenge
         * @param _agv AGV on which the part is placed
         */
        HumanChallengeOnPartPlacement(unsigned behavior,
                                      std::shared_ptr<Part> part,
                                      unsigned int agv) : HumanChallenge(behavior),
                                                          part_(part), agv_(agv) {}

        /**
         * @brief Get the part to trigger the challenge
         * @return std::shared_ptr<Part> Part to trigger the challenge
         */
        std::shared_ptr<Part> GetPart() const { return part_; }

        /**
         * @brief Get the AGV on which the part is placed
         * @return unsigned int AGV on which the part is placed
         */
        unsigned int GetAgv() const { return agv_; }

    private:
        //! Part to trigger the challenge
        std::shared_ptr<Part> part_;
        //! AGV on which the part is placed
        unsigned int agv_;
    };

    //==============================================================================
    /**
     * @brief Class which derives HumanChallenge
     *
     * This challenge is triggered when an order is submitted.
     */
    class HumanChallengeOnSubmission : public HumanChallenge
    {
    public:
        /**
         * @brief Construct a new Human challenge object
         *
         * @param behavior The behavior of the human during the trial
         * @param robot_time_penalty Time penalty for the ceiling robot
         * @param trigger_order_id ID of a submitted order
         */
        HumanChallengeOnSubmission(unsigned behavior,
                                   std::string trigger_order_id) : HumanChallenge(behavior),
                                                                   trigger_order_id_(trigger_order_id) {}

        /**
         * @brief Get the ID of a submitted order
         * @return std::string ID of a submitted order
         */
        std::string GetTriggerOrderId() const { return trigger_order_id_; }

    private:
        //! ID of a submitted order
        std::string trigger_order_id_;
    };

    //==============================================================================
    /**
     * @brief Base class for the sensor blackout challenges.
     *
     */
    class SensorBlackout
    {
    public:
        /**
         * @brief Construct a new SensorBlackout object
         *
         * @param duration The duration of the blackout in seconds
         * @param sensors_to_disable List of sensors to disable
         */
        SensorBlackout(double duration,
                       const std::vector<std::string> &sensors_to_disable) : duration_(duration),
                                                                             sensors_to_disable_(sensors_to_disable) {}

        /**
         * @brief Get the duration of the challenge
         *
         * @return double
         */
        double GetDuration() const { return duration_; }
        /**
         * @brief Get the list of sensors to disable
         *
         * @return const std::vector<std::string>&
         */
        const std::vector<std::string> &GetSensorsToDisable() const { return sensors_to_disable_; }
        /**
         * @brief Get the type of the challenge
         *
         * @return unsigned int
         */
        unsigned int GetType() const { return type_; }
        /**
         * @brief Get the Start Time
         *
         * @return double
         */
        double GetStartTime() const { return start_time_; }
        /**
         * @brief Set the Start Time
         *
         * @param _start_time
         */
        void SetStartTime(double start_time) { start_time_ = start_time; }

        /**
         * @brief Check if the challenge has started
         *
         * @return bool True if the challenge has started and false otherwise
         */
        bool HasStarted() const { return started_; }

        /**
         * @brief Set the started flag to true
         *
         */
        void SetStarted() { started_ = true; }

        /**
         * @brief Check if the challenge has been completed
         *
         * @return bool True if the challenge has been completed and false otherwise
         */
        bool HasCompleted() const { return completed_; }

        /**
         * @brief Set the completed flag to true
         *
         */
        void SetCompleted() { completed_ = true; }

        /**
         * @brief Set the Stop Time
         *
         * @param stop_time
         */
        void SetStopTime(double stop_time) { stop_time_ = stop_time; }

        /**
         * @brief Get the Stop Time
         *
         * @return double
         */
        double GetStopTime() const { return stop_time_; }

    protected:
        //! Type of the challenge
        const unsigned int type_ = ariac_msgs::msg::Challenge::SENSOR_BLACKOUT;
        //! Duration of the challenge
        double duration_;
        //! List of sensors to disable
        std::vector<std::string> sensors_to_disable_;
        //! Flag to check if the challenge has started
        bool started_ = false;
        //! Flag to check if the challenge has been completed
        bool completed_ = false;
        //! Start time of the challenge
        double start_time_;
        //! Stop time of the challenge
        double stop_time_;
    };

    //==============================================================================
    /**
     * @brief Class which derives SensorBlackout
     *
     * This challenge is triggered at a specific simulation time.
     */
    class SensorBlackoutTemporal : public SensorBlackout
    {
    public:
        /**
         * @brief Construct a new SensorBlackoutTemporal object
         *
         * @param duration Duration of the blackout in seconds
         * @param sensors_to_disable List of sensors to disable
         * @param trigger_time Time at which the challenge should be triggered
         */
        SensorBlackoutTemporal(double duration,
                               const std::vector<std::string> &sensors_to_disable,
                               double trigger_time) : SensorBlackout(duration, sensors_to_disable),
                                                      trigger_time_(trigger_time) {}

        /**
         * @brief Get the time at which the challenge should be triggered
         * @return double Time at which the challenge should be triggered
         */
        double GetTriggerTime() const { return trigger_time_; }

    private:
        //! Simulation time at which the challenge should be triggered
        double trigger_time_;
    };

    //==============================================================================
    /**
     * @brief Class to represent the fields for the sensor blackout challenge
     *
     * This challenge is triggered during kitting
     */
    class SensorBlackoutOnPartPlacement : public SensorBlackout
    {
    public:
        /**
         * @brief Construct a new SensorBlackoutKittingAction object
         *
         * @param duration Duration of the blackout
         * @param sensors_to_disable List of sensors to disable
         * @param part Part to trigger the challenge
         * @param agv AGV on which the part is placed
         */

        SensorBlackoutOnPartPlacement(double duration,
                                      const std::vector<std::string> &sensors_to_disable,
                                      std::shared_ptr<Part> part,
                                      unsigned int agv) : SensorBlackout(duration, sensors_to_disable),
                                                          part_(part),
                                                          agv_(agv) {}

        /**
         * @brief Get the Part object
         *
         * @return std::shared_ptr<Part>
         */
        std::shared_ptr<Part> GetPart() const { return part_; }
        /**
         * @brief Get the AGV on which the part is placed
         *
         * @return unsigned int
         */
        unsigned int GetAgv() const { return agv_; }

    private:
        //! Part to trigger the challenge
        std::shared_ptr<Part> part_;
        //! AGV on which the part is placed
        unsigned int agv_;
    }; // class SensorBlackoutOnPartPlacement

    //==============================================================================
    /**
     * @brief Class to represent the fields for the sensor blackout challenge
     *
     * This challenge is triggered when an order is submitted
     */
    class SensorBlackoutOnSubmission : public SensorBlackout
    {
    public:
        /**
         * @brief Construct a new SensorBlackoutKittingSubmission object
         *
         * @param duration Duration of the blackout
         * @param sensors_to_disable List of sensors to disable
         * @param trigger_order_id ID of a submitted order
         */

        SensorBlackoutOnSubmission(double duration,
                                   const std::vector<std::string> &sensors_to_disable,
                                   std::string trigger_order_id) : SensorBlackout(duration, sensors_to_disable),
                                                                   trigger_order_id_(trigger_order_id) {}

        /**
         * @brief Get the ID of a submitted order
         *
         * @return std::string
         */
        std::string GetTriggerOrderId() const { return trigger_order_id_; }

    private:
        //! ID of a submitted order
        std::string trigger_order_id_;
    }; // class SensorBlackoutKittingAction

    class Quadrant
    {
    public:
        /**
         * @brief Construct a new Quadrant object
         *
         * @param quadrant_number Quadrant number
         * @param is_correct_part_type  True if the part type is correct
         * @param is_correct_part_color True if the part color is correct
         * @param is_faulty True if the part is faulty
         * @param is_flipped True if the part is flipped
         * @param score Score for the quadrant
         */
        Quadrant(int quadrant_number,
                 bool is_correct_part_type,
                 bool is_correct_part_color,
                 bool is_faulty,
                 bool is_flipped,
                 int score) : quadrant_number_(quadrant_number),
                              is_correct_part_type_(is_correct_part_type),
                              is_correct_part_color_(is_correct_part_color),
                              is_faulty_(is_faulty),
                              is_flipped_(is_flipped),
                              score_(score)
        {
        }

        /**
         * @brief Get the Quadrant Number object
         *
         * @return int Quadrant number
         */
        friend std::ostream &operator<<(std::ostream &out,
                                        const Quadrant &obj)
        {
            int quadrant_num = 0;
            if (obj.quadrant_number_ == 1)
                quadrant_num = 1;
            else if (obj.quadrant_number_ == 2)
                quadrant_num = 2;
            else if (obj.quadrant_number_ == 3)
                quadrant_num = 3;
            else if (obj.quadrant_number_ == 4)
                quadrant_num = 4;
            out << "-----------" << std::endl;
            out << "   Quadrant" << quadrant_num << " score: " << obj.score_ << std::endl;
            out << "   -----------" << std::endl;
            out << "   Correct part type: " << std::boolalpha << obj.is_correct_part_type_ << std::endl;
            out << "   Correct part color: " << std::boolalpha << obj.is_correct_part_color_ << std::endl;
            out << "   Faulty Part: " << std::boolalpha << obj.is_faulty_ << std::endl;
            out << "   Flipped Part: " << std::boolalpha << obj.is_flipped_ << std::endl;

            return out;
        }

        /**
         * @brief Get the Quadrant Number object
         *
         * @return int Quadrant number
         */
        int GetQuadrantNumber() const { return quadrant_number_; }

        /**
         * @brief Get the Is Correct Part Type object
         *
         * @return true True if the part type is correct
         * @return false False if the part type is incorrect
         */
        bool GetIsCorrectPartType() const { return is_correct_part_type_; }

        /**
         * @brief Get the Is Correct Part Color object
         *
         * @return true True if the part color is correct
         * @return false False if the part color is incorrect
         */
        bool GetIsCorrectPartColor() const { return is_correct_part_color_; }

        /**
         * @brief Get the Is Faulty object
         *
         * @return true True if the part is faulty
         * @return false False if the part is not faulty
         */
        bool GetIsFaulty() const { return is_faulty_; }

        /**
         * @brief Get the Is Flipped object
         *
         * @return true True if the part is flipped
         * @return false False if the part is not flipped
         */
        bool GetIsFlipped() const { return is_flipped_; }

        /**
         * @brief Get the Score object
         *
         * @return int Score for the quadrant
         */
        int GetScore() const { return score_; }
    protected:
        //! Quadrant number
        int quadrant_number_;
        //! True if the part type is correct
        bool is_correct_part_type_;
        //! True if the part color is correct
        bool is_correct_part_color_;
        //! True if the part is faulty
        bool is_faulty_;
        //! True if the part is flipped
        bool is_flipped_;
        //! Score for the quadrant
        int score_;
    };

    //==============================================================================
    /**
     * @brief Class to represent the score for a kitting order
     *
     */
    class KittingScore
    {
    public:
        /**
         * @brief Construct a new Kitting Score object
         *
         * @param order_id  ID of the order
         * @param kit_score  Score for the kit
         * @param tray_score  Score for the tray
         * @param quadrant1 Quadrant 1 object
         * @param quadrant2  Quadrant 2 object
         * @param quadrant3  Quadrant 3 object
         * @param quadrant4  Quadrant 4 object
         * @param bonus  Bonus score
         * @param penalty  Penalty score
         */
        KittingScore(std::string order_id,
                     int score,
                     int tray_score,
                     std::shared_ptr<Quadrant> quadrant1,
                     std::shared_ptr<Quadrant> quadrant2,
                     std::shared_ptr<Quadrant> quadrant3,
                     std::shared_ptr<Quadrant> quadrant4,
                     int bonus,
                     int penalty,
                     int agv,
                     int destination_score) : order_id_(order_id),
                                              score_(score),
                                              tray_score_(tray_score),
                                              quadrant1_(quadrant1),
                                              quadrant2_(quadrant2),
                                              quadrant3_(quadrant3),
                                              quadrant4_(quadrant4),
                                              bonus_(bonus),
                                              penalty_(penalty),
                                              agv_(agv),
                                              destination_score_(destination_score) {}

        /**
         * @brief Get the Order Id object
         *
         * @return std::string Order ID
         */
        std::string GetOrderId() const { return order_id_; }

        /**
         * @brief Get the Order Id object
         *
         * @return std::string Order ID
         */
        int GetScore() const { return score_; }

        /**
         * @brief Get the Tray Score object
         *
         * @return int Tray score
         */
        int GetTrayScore() const { return tray_score_; }

        /**
         * @brief Get the Quadrant 1 object
         *
         * @return std::shared_ptr<Quadrant> Quadrant 1 object
         */ 
        std::shared_ptr<Quadrant> GetQuadrant1() const { return quadrant1_; }

        /**
         * @brief Get the Quadrant 2 object
         *
         * @return std::shared_ptr<Quadrant> Quadrant 2 object
         */
        std::shared_ptr<Quadrant> GetQuadrant2() const { return quadrant2_; }

        /**
         * @brief Get the Quadrant 3 object
         *
         * @return std::shared_ptr<Quadrant> Quadrant 3 object
         */
        std::shared_ptr<Quadrant> GetQuadrant3() const { return quadrant3_; }

        /**
         * @brief Get the Quadrant 4 object
         *
         * @return std::shared_ptr<Quadrant> Quadrant 4 object
         */
        std::shared_ptr<Quadrant> GetQuadrant4() const { return quadrant4_; }

        /**
         * @brief Get the Bonus object
         *
         * @return int Bonus score
         */
        int GetBonus() const { return bonus_; }

        /**
         * @brief Get the Penalty object
         *
         * @return int Penalty score
         */
        int GetPenalty() const { return penalty_; }

        /**
         * @brief Get the AGV object
         *
         * @return int AGV
         */
        int GetAGV() const { return agv_; }

        /**
         * @brief Get the Destination object
         *
         * @return int Destination
         */
        int GetDestinationScore() const { return destination_score_; }

    protected:
        //! Order ID
        std::string order_id_;
        //! Kit score
        int score_;
        //! Tray score
        int tray_score_;
        //! Quadrant 1
        std::shared_ptr<Quadrant> quadrant1_ = nullptr;
        //! Quadrant 2
        std::shared_ptr<Quadrant> quadrant2_ = nullptr;
        //! Quadrant 3
        std::shared_ptr<Quadrant> quadrant3_ = nullptr;
        //! Quadrant 4
        std::shared_ptr<Quadrant> quadrant4_ = nullptr;
        //! Bonus score
        int bonus_;
        //! Penalty score
        int penalty_;
        //! AGV
        int agv_;
        //! Destination
        int destination_score_;

    }; // class KittingScore

    class ScoredAssemblyPart
    {
    public:
        ScoredAssemblyPart(const Part &part,
                           bool is_correct_color,
                           bool is_correct_pose,
                           geometry_msgs::msg::Vector3 &position,
                           geometry_msgs::msg::Vector3 &orientation,
                           int score,
                           bool is_faulty) : part_(part),
                                              is_correct_color_(is_correct_color),
                                              is_correct_pose_(is_correct_pose),
                                              position_(position),
                                              orientation_(orientation),
                                              score_(score),
                                              is_faulty_(is_faulty)
        {
        }


        /**
         * @brief Get the Part
         *
         * @return Part Part object
         */
        Part GetPart() const { return part_; }
        /**
         * @brief Get the Score
         *
         * @return int Score
         */
        int GetScore() const { return score_; }
        /**
         * @brief Get the Correct Color
         *
         * @return bool True if the color is correct
         */
        bool GetCorrectColor() const { return is_correct_color_; }
        /**
         * @brief Get the Correct Pose
         *
         * @return bool True if the pose is correct
         */
        bool GetCorrectPose() const { return is_correct_pose_; }
        /**
         * @brief Get the Position
         *
         * @return geometry_msgs::msg::Vector3 Position
         */
        geometry_msgs::msg::Vector3 GetPosition() const { return position_; }
        /**
         * @brief Get the Orientation
         *
         * @return geometry_msgs::msg::Vector3 Orientation
         */
        geometry_msgs::msg::Vector3 GetOrientation() const { return orientation_; }

        /**
         * @brief Get the Is Faulty
         *
         * @return bool True if the part is faulty
         */
        bool GetIsFaulty() const { return is_faulty_; }

    protected:
        //! The part object
        Part part_;
        //! Position of the part in the insert
        geometry_msgs::msg::Vector3 position_;
        //! Orientation (r, p, y) of the part in the insert
        geometry_msgs::msg::Vector3 orientation_;
        //! Sore of the assembled part
        int score_;
        //! Boolean to indicate if the color is correct
        bool is_correct_color_;
        //! Boolean to indicate if the pose is correct
        bool is_correct_pose_;
        //! Boolean to indicate if the part is faulty
        bool is_faulty_;
    };

    //==============================================================================
    /**
     * @brief Class to represent the score for a kitting order
     *
     */
    class AssemblyScore
    {
    public:
        /**
         * @brief Construct a new Kitting Score object
         *
         * @param order_id  ID of the order
         * @param insert_score  Score for the insert
         * @param battery_ptr Share pointer to the battery
         * @param pump_ptr  Share pointer to the pump
         * @param regulator_ptr  Share pointer to the regulator
         * @param sensor_ptr  Share pointer to the sensor
         * @param bonus  Bonus score
         */
        AssemblyScore(std::string order_id,
                      int insert_score,
                      int station,
                      std::shared_ptr<ScoredAssemblyPart> battery_ptr,
                      std::shared_ptr<ScoredAssemblyPart> pump_ptr,
                      std::shared_ptr<ScoredAssemblyPart> regulator_ptr,
                      std::shared_ptr<ScoredAssemblyPart> sensor_ptr,
                      int bonus) : order_id_(order_id),
                      station_(station),
                                   insert_score_(insert_score),
                                   battery_ptr_(battery_ptr),
                                   pump_ptr_(pump_ptr),
                                   regulator_ptr_(regulator_ptr),
                                   sensor_ptr_(sensor_ptr),
                                   bonus_(bonus) {}

        

        /**
         * @brief Get the Order Id object
         * 
         * @return std::string  Order ID
         */
        std::string GetOrderId() const { return order_id_; }
        /**
         * @brief Get the Order Id object
         *
         * @return std::string Order ID
         */
        int GetScore() const { return insert_score_; }

        std::shared_ptr<ScoredAssemblyPart> GetBatteryPtr() const { return battery_ptr_; }
        std::shared_ptr<ScoredAssemblyPart> GetPumpPtr() const { return pump_ptr_; }
        std::shared_ptr<ScoredAssemblyPart> GetRegulatorPtr() const { return regulator_ptr_; }
        std::shared_ptr<ScoredAssemblyPart> GetSensorPtr() const { return sensor_ptr_; }
        int GetBonus() const { return bonus_; }
        int GetStation() const { return station_; }

    protected:
        //! Order ID
        std::string order_id_;
        //! Insert score
        int insert_score_;
        //! Station
        int station_;
        //! Battery
        std::shared_ptr<ScoredAssemblyPart> battery_ptr_ = nullptr;
        //! Pump
        std::shared_ptr<ScoredAssemblyPart> pump_ptr_ = nullptr;
        //! Regulator
        std::shared_ptr<ScoredAssemblyPart> regulator_ptr_ = nullptr;
        //! Sensor
        std::shared_ptr<ScoredAssemblyPart> sensor_ptr_ = nullptr;
        //! Bonus score
        int bonus_;

    }; // class AssemblyScore

    //==============================================================================
    /**
     * @brief Class to represent the score for a combined task
     *
     */
    class CombinedScore
    {
    public:
        /**
         * @brief Construct a new Kitting Score object
         *
         * @param order_id  ID of the order
         * @param insert_score  Score for the insert
         * @param battery_ptr Share pointer to the battery
         * @param pump_ptr  Share pointer to the pump
         * @param regulator_ptr  Share pointer to the regulator
         * @param sensor_ptr  Share pointer to the sensor
         * @param bonus  Bonus score
         */
        CombinedScore(std::string order_id,
                      int insert_score,
                      int station,
                      std::shared_ptr<ScoredAssemblyPart> battery_ptr,
                      std::shared_ptr<ScoredAssemblyPart> pump_ptr,
                      std::shared_ptr<ScoredAssemblyPart> regulator_ptr,
                      std::shared_ptr<ScoredAssemblyPart> sensor_ptr,
                      int bonus) : order_id_(order_id),
                                   station_(station),
                                   insert_score_(insert_score),
                                   battery_ptr_(battery_ptr),
                                   pump_ptr_(pump_ptr),
                                   regulator_ptr_(regulator_ptr),
                                   sensor_ptr_(sensor_ptr),
                                   bonus_(bonus) {}

        /**
         * @brief Get the Order Id object
         *
         * @return std::string  Order ID
         */
        std::string GetOrderId() const { return order_id_; }
        /**
         * @brief Get the Order Id object
         *
         * @return std::string Order ID
         */
        int GetScore() const { return insert_score_; }

        /**
         * @brief Get the pointer to ScoredAssemblyPart for the battery
         *
         * @return std::shared_ptr<ScoredAssemblyPart>  Share pointer to ScoredAssemblyPart for the battery
         */
        std::shared_ptr<ScoredAssemblyPart> GetBatteryPtr() const { return battery_ptr_; }
        /**
         * @brief Get the pointer to ScoredAssemblyPart for the pump
         *
         * @return std::shared_ptr<ScoredAssemblyPart>  Share pointer to ScoredAssemblyPart for the pump
         */
        std::shared_ptr<ScoredAssemblyPart> GetPumpPtr() const { return pump_ptr_; }
        /**
         * @brief Get the pointer to ScoredAssemblyPart for the regulator
         *
         * @return std::shared_ptr<ScoredAssemblyPart>  Share pointer to ScoredAssemblyPart for the regulator
         */
        std::shared_ptr<ScoredAssemblyPart> GetRegulatorPtr() const { return regulator_ptr_; }
        /**
         * @brief Get the pointer to ScoredAssemblyPart for the sensor
         *
         * @return std::shared_ptr<ScoredAssemblyPart>  Share pointer to ScoredAssemblyPart for the sensor
         */
        std::shared_ptr<ScoredAssemblyPart> GetSensorPtr() const { return sensor_ptr_; }
        /**
         * @brief Get the bonus score for the task
         *
         * @return int  Bonus score
         */
        int GetBonus() const { return bonus_; }
        /**
         * @brief Get the station number
         *
         * @return int  Station number
         */
        int GetStation() const { return station_; }

    protected:
        //! Order ID
        std::string order_id_;
        //! Insert score
        int insert_score_;
        //! Station
        int station_;
        //! Battery
        std::shared_ptr<ScoredAssemblyPart> battery_ptr_ = nullptr;
        //! Pump
        std::shared_ptr<ScoredAssemblyPart> pump_ptr_ = nullptr;
        //! Regulator
        std::shared_ptr<ScoredAssemblyPart> regulator_ptr_ = nullptr;
        //! Sensor
        std::shared_ptr<ScoredAssemblyPart> sensor_ptr_ = nullptr;
        //! Bonus score
        int bonus_;

    }; // class CombinedScore

    //==============================================================================
    class Order
    {
    public:
        /**
         * @brief Construct a new Order object
         *
         * @param order_id Unique id of the order
         * @param order_type  Type of the order.
         * @param high_priority Priority of the order (true or false)
         * @param trial_time_limit  Time limit for the trial
         */
        Order(std::string id,
              unsigned int type,
              bool priority,
              double trial_time_limit) : announced_(false),
                                         submitted_(false),
                                         pre_assembly_service_called_(false),
                                         id_(id),
                                         type_(type),
                                         priority_(priority),
                                         trial_time_limit_(trial_time_limit) {}
        ~Order() = default;

        /**
         * @brief Overload the << operator to print the order
         *
         * @param out  Output stream
         * @param obj  Order object
         * @return std::ostream& Output stream
         */
        friend std::ostream &operator<<(std::ostream &out,
                                        const Order &obj)
        {
            out << "=================" << std::endl;
            out << "Announcing Order " << obj.id_ << std::endl;
            out << "=================" << std::endl;
            if (obj.type_ == ariac_msgs::msg::Order::KITTING)
                out << "Type: Kitting" << std::endl;
            else if (obj.type_ == ariac_msgs::msg::Order::ASSEMBLY)
                out << "Type: Assembly" << std::endl;
            else if (obj.type_ == ariac_msgs::msg::Order::COMBINED)
                out << "Type: Combined" << std::endl;
            out << "Priority: " << obj.priority_ << std::endl;
            if (obj.type_ == ariac_msgs::msg::Order::KITTING)
                out << *obj.kitting_task_;
            else if (obj.type_ == ariac_msgs::msg::Order::ASSEMBLY)
                out << *obj.assembly_task_;
            else if (obj.type_ == ariac_msgs::msg::Order::COMBINED)
                out << *obj.combined_task_;
            return out;
        }

        /**
         * @brief Get the Id of the order
         * @return std::string Id of the order
         */
        std::string GetId() const { return id_; }

        /**
         * @brief Get the type of the order
         * @return unsigned int Type of the order
         */
        unsigned int GetType() const { return type_; }

        /**
         * @brief Get the priority of the order
         *
         * @return true High-priority order
         * @return false Regular-priority order
         */
        bool IsPriority() const { return priority_; }

        /**
         * @brief Get a shared pointer to a kitting task
         * @return std::shared_ptr<KittingTask> Pointer to a kitting task
         */
        std::shared_ptr<KittingTask> GetKittingTask() const { return kitting_task_; }

        /**
         * @brief Get a shared pointer to an assembly task
         * @return std::shared_ptr<AssemblyTask> Assembly task
         */
        std::shared_ptr<AssemblyTask> GetAssemblyTask() const { return assembly_task_; }

        /**
         * @brief Get a shared pointer to a combined task
         * @return std::shared_ptr<CombinedTask> Combined task
         */
        std::shared_ptr<CombinedTask> GetCombinedTask() const { return combined_task_; }

        /**
         * @brief Set the Kitting Task object for the order
         * @param _kitting_task  Shared pointer to the kitting task for the order
         */
        virtual void SetKittingTask(std::shared_ptr<KittingTask> _kitting_task) { kitting_task_ = _kitting_task; }

        /**
         * @brief Set the Assembly Task object for the order
         * @param _assembly_task  Shared pointer to the assembly task for the order
         */
        virtual void SetAssemblyTask(std::shared_ptr<AssemblyTask> _assembly_task) { assembly_task_ = _assembly_task; }

        /**
         * @brief Set the Combined Task object for the order
         * @param _combined_task  Shared pointer to the combined task for the order
         */
        virtual void SetCombinedTask(std::shared_ptr<CombinedTask> _combined_task) { combined_task_ = _combined_task; }

        /**
         * @brief Check whether or not the order has been announced
         * @return true Order has already been announced
         * @return false Order has not been announced yet
         */
        virtual bool IsAnnounced() const { return announced_; }

        /**
         * @brief Set this order as announced
         */
        virtual void SetIsAnnounced() { announced_ = true; }

        /**
         * @brief Set the time at which the order was announced
         * @param _announced_time  Time at which the order was announced
         */
        void SetAnnouncedTime(double _announced_time) { announced_time_ = _announced_time; }

        /**
         * @brief Get the time at which the order was announced
         * @return double Time at which the order was announced
         */
        double GetAnnouncedTime() const { return announced_time_; }

        /**
         * @brief Check whether or not the order has been submitted
         * @return true  Order has already been submitted
         * @return false  Order has not been submitted yet
         */
        virtual bool IsSubmitted() const { return submitted_; }

        /**
         * @brief Set this order as submitted
         */
        virtual void SetIsSubmitted() { submitted_ = true; }
        /**
         * @brief Set the time the order was submitted
         * @param _submitted_time
         */
        virtual void SetSubmittedTime(double _submitted_time) { submitted_time_ = _submitted_time; }

        /**
         * @brief Get the time the order was submitted
         * @return double Time the order was submitted
         */
        double GetSubmittedTime() const { return submitted_time_; }

        /**
         * @brief Check whether or not the service has been called for this order
         * @return true  Service has been called
         * @return false  Service has not been called
         */
        bool WasPreAssemblyServiceCalled() { return pre_assembly_service_called_; }

        /**
         * @brief Set this order as called
         */
        void SetPreAssemblyServiceCalled() { pre_assembly_service_called_ = true; }

        /**
         * @brief Set the KittingScore object for the order
         *
         * @param _kitting_score Pointer to the KittingScore object for the order
         */
        virtual void SetKittingScore(std::shared_ptr<KittingScore> _kitting_score)
        {
            kitting_score_ = _kitting_score;
        }

        /**
         * @brief Set the AssemblyScore object for the order
         *
         * @param assembly_score Pointer to the KittingScore object for the order
         */
        virtual void SetAssemblyScore(std::shared_ptr<AssemblyScore> assembly_score)
        {
            assembly_score_ = assembly_score;
        }

        /**
         * @brief Set the CombinedScore object for the order
         *
         * @param combined_score Pointer to the CombinedScore object for the order
         */
        virtual void SetCombinedScore(std::shared_ptr<CombinedScore> combined_score)
        {
            combined_score_ = combined_score;
        }

        /**
         * @brief Get the CombinedScore object for the order
         * @return std::shared_ptr<CombinedScore> Pointer to the CombinedScore object for the order
         */
        std::shared_ptr<CombinedScore> GetCombinedScore() const { return combined_score_; }

        /**
         * @brief Get the KittingScore object for the order
         * @return std::shared_ptr<KittingScore> Pointer to the KittingScore object for the order
         */
        std::shared_ptr<KittingScore> GetKittingScore() const { return kitting_score_; }

        /**
         * @brief Get the AssemblyScore object for the order
         * @return std::shared_ptr<AssemblyScore> Pointer to the AssemblyScore object for the order
         */
        std::shared_ptr<AssemblyScore> GetAssemblyScore() const { return assembly_score_; }

    protected:
        /**
         * @brief Whether or not this order has already been announced
         */
        bool announced_;

        /**
         * @brief Whether or not this order has already been submitted
         */
        bool submitted_;

        /**
         * @brief Whether or not the pre assembly pose service was called
         */
        bool pre_assembly_service_called_;

        /**
         * @brief id of the order
         */
        std::string id_;
        /**
         * @brief Type of the order. The possibilities are:
         * -0: kitting
         * -1: assembly
         * -2: combined
         */
        unsigned int type_;
        /**
         * @brief priority of the order
         * true: high-priority order
         * false: regular order
         */
        bool priority_;
        /**
         * @brief Time limit to perform the trial to which this order belongs
         */
        double trial_time_limit_;
        /**
         * @brief Time at which the order is submitted since the start of the competition
         */
        double submitted_time_;

        /**
         * @brief Time at which the order is announced since the start of the competition
         *
         */
        double announced_time_;

        /**
         * @brief A pointer to the kitting task for this order.
         *
         */
        std::shared_ptr<KittingTask> kitting_task_ = nullptr;
        /**
         * @brief A pointer to the assembly task for this order.
         *
         */
        std::shared_ptr<AssemblyTask> assembly_task_ = nullptr;
        /**
         * @brief A pointer to the combined task for this order.
         *
         */
        std::shared_ptr<CombinedTask> combined_task_ = nullptr;

        /**
         * @brief Score for the kitting task
         */
        std::shared_ptr<KittingScore> kitting_score_ = nullptr;

        /**
         * @brief Score for the assembly task
         *
         */
        std::shared_ptr<AssemblyScore> assembly_score_ = nullptr;

        /**
         * @brief Score for the assembly task
         *
         */
        std::shared_ptr<CombinedScore> combined_score_ = nullptr;
    };
    //-- end class Order

    //==============================================================================
    /**
     * @brief Class to represent the fields for a time-based order
     */
    class OrderTemporal : public Order
    {
    public:
        /**
         * @brief Construct a new OrderTemporal object
         *
         * @param id Unique id of the order
         * @param type Type of the order
         * @param priority Priority of the order (true or false)
         * @param trial_time_limit Time limit for the trial
         * @param announcement_time Time at which the order should be announced
         */
        ~OrderTemporal() = default;

        OrderTemporal(std::string id,
                      unsigned int type,
                      bool priority,
                      double trial_time_limit,
                      double announcement_time) : Order(id, type, priority, trial_time_limit),
                                                  announcement_time_(announcement_time) {}

        /**
         * @brief Get the announcement time for the order
         *
         * @return double Time at which the order should be announced
         */
        double GetAnnouncementTime() const
        {
            return announcement_time_;
        }

    private:
        //! Time at which the order should be announced
        double announcement_time_;
    }; // class OrderTemporal

    //==============================================================================
    /**
     * @brief Class to represent the fields for an order announced during kitting
     */
    class OrderOnPartPlacement : public Order
    {
    public:
        /**
         * @brief Construct a new OrderDuringKitting object
         *
         * @param id Unique id of the order
         * @param type Type of the order
         * @param priority Priority of the order (true or false)
         * @param trial_time_limit Time limit for the trial
         * @param agv Announcement: AGV on which the part is placed
         */
        OrderOnPartPlacement(std::string id,
                             unsigned int type,
                             bool priority,
                             double trial_time_limit,
                             unsigned int agv,
                             std::shared_ptr<Part> part) : Order(id, type, priority, trial_time_limit),
                                                           agv_(agv), part_(part) {}

        /**
         * @brief Get the Agv
         *
         * @return unsigned int
         */
        unsigned int GetAgv() const
        {
            return agv_;
        }

        /**
         * @brief Get the Part
         *
         * @return std::shared_ptr<Part>
         */
        std::shared_ptr<Part> GetPart() const
        {
            return part_;
        }

    private:
        //! AGV on which the part is placed
        unsigned int agv_;
        //! Part
        std::shared_ptr<Part> part_;
    }; // class OrderOnPartPlacement

    //==============================================================================
    /**
     * @brief Class to represent the fields for an order announced after kitting
     *
     */
    class OrderOnSubmission : public Order
    {
    public:
        /**
         * @brief Construct a new OrderAfterKitting object
         *
         * @param id Unique id of the order
         * @param type Type of the order
         * @param priority Priority of the order (true or false)
         * @param trial_time_limit Time limit for the trial
         * @param order_id Id of the order submitted by the competitor
         */
        OrderOnSubmission(std::string id,
                          unsigned int type,
                          bool priority,
                          double trial_time_limit,
                          std::string order_id) : Order(id, type, priority, trial_time_limit),
                                                  order_id_(order_id) {}

        /**
         * @brief Get the Order Id object
         *
         * @return std::string
         */
        std::string GetOrderId() const
        {
            return order_id_;
        }

    private:
        //! Id of the order submitted by the competitor
        std::string order_id_;
    }; // class OrderOnSubmission

    //==============================================================================
    /**
     * @brief Base class for the robot malfunction challenges.
     *
     */
    class RobotMalfunction
    {
    public:
        /**
         * @brief Construct a new RobotMalfunction object
         *
         * @param duration Duration of the challenge
         * @param robots_to_disable  List of robots to disable
         */
        RobotMalfunction(double duration,
                         const std::vector<std::string> &robots_to_disable) : duration_(duration),
                                                                              robots_to_disable_(robots_to_disable) {}

        //===================
        //--- Getters ---
        //===================

        /**
         * @brief Get the duration of the challenge
         * @return double Duration of the challenge
         */
        double GetDuration() const { return duration_; }

        /**
         * @brief Get the type of the challenge
         * @return unsigned int Type of the challenge
         */
        unsigned int GetType() const { return type_; }

        /**
         * @brief Get a list of robots to disable
         * @return const std::vector<std::string>& List of robots to disable
         */
        const std::vector<std::string> &GetRobotsToDisable() const { return robots_to_disable_; }

        /**
         * @brief Get the sim time at which the challenge was started
         * @return double Sim time at which the challenge was started
         */
        double GetStartTime() const { return start_time_; }

        /**
         * @brief Get the sim time at which the challenge was stopped
         * @return double Sim time at which the challenge was stopped
         */
        double GetStopTime() const { return stop_time_; }

        /**
         * @brief Check if the challenge has started
         * @return true If the challenge has started
         * @return false If the challenge has not started
         */
        bool HasStarted() const { return started_; }

        /**
         * @brief Check if the challenge has completed
         * @return true If the challenge has completed
         * @return false If the challenge has not completed
         */
        bool HasCompleted() const { return completed_; }

        //===================
        //--- Setters ---
        //===================

        /**
         * @brief Set the sim time at which the challenge was started
         * @param _start_time Sim time at which the challenge was started
         */
        void SetStartTime(double _start_time) { start_time_ = _start_time; }

        /**
         * @brief Set the sim time at which the challenge was stopped
         * @param _stop_time Sim time at which the challenge was stopped
         */
        void SetStopTime(double _stop_time) { stop_time_ = _stop_time; }

        /**
         * @brief Set the flag to indicate if the challenge has started
         */
        void SetStarted() { started_ = true; }

        /**
         * @brief Set the flag to indicate if the challenge has completed
         */
        void SetCompleted() { completed_ = true; }

    protected:
        //! Type of the challenge
        const unsigned int type_ = ariac_msgs::msg::Challenge::ROBOT_MALFUNCTION;
        //! Duration of the challenge
        double duration_;
        //! List of robots to disable
        std::vector<std::string> robots_to_disable_;
        //! Sim time at which the challenge was started
        double start_time_;
        //! Sim time at which the challenge was stopped
        double stop_time_;
        //! Flag to indicate if the challenge has started
        bool started_ = false;
        //! Flag to indicate if the challenge has completed
        bool completed_ = false;
    };

    //==============================================================================
    /**
     * @brief Derived class for the robot malfunction challenge.
     *
     * This challenge is triggered at a specific simulation time.
     */
    class RobotMalfunctionTemporal : public RobotMalfunction
    {
    public:
        /**
         * @brief Construct a new RobotMalfunctionTemporal object
         *
         * @param duration Duration of the challenge
         * @param robots_to_disable  List of robots to disable
         * @param trigger_time Competition time at which the challenge is triggered
         */
        RobotMalfunctionTemporal(double duration,
                                 const std::vector<std::string> &robots_to_disable,
                                 double trigger_time) : RobotMalfunction(duration, robots_to_disable),
                                                        trigger_time_(trigger_time) {}
        /**
         * @brief Get the time at which the challenge should be triggered
         * @return double Time at which the challenge should be triggered
         */
        double GetTriggerTime() const { return trigger_time_; }

        /**
         * @brief Set the time at which the challenge should be triggered
         * @param _trigger_time Time at which the challenge should be triggered
         */
        void SetTriggerTime(double _trigger_time) { trigger_time_ = _trigger_time; }

    private:
        //! Simulation time at which the challenge is triggered
        double trigger_time_;
    }; // class RobotMalfunctionTemporal

    //==============================================================================
    /**
     * @brief Class to represent the fields for the robot malfunction challenge
     *
     * This challenge is triggered during part placement
     */
    class RobotMalfunctionOnPartPlacement : public RobotMalfunction
    {
    public:
        /**
         * @brief Construct a new RobotMalfunctionOnPartPlacement object
         *
         * @param duration Duration of the blackout
         * @param robots_to_disable  List of robots to disable
         * @param part Part to trigger the challenge
         * @param agv AGV on which the part is placed
         */

        RobotMalfunctionOnPartPlacement(double duration,
                                        const std::vector<std::string> &robots_to_disable,
                                        std::shared_ptr<Part> part,
                                        unsigned int agv) : RobotMalfunction(duration, robots_to_disable),
                                                            part_(part),
                                                            agv_(agv) {}

        std::shared_ptr<Part> GetPart() const { return part_; }
        unsigned int GetAgv() const { return agv_; }

    private:
        //! Part to trigger the challenge
        std::shared_ptr<Part> part_;
        //! AGV on which the part is placed
        unsigned int agv_;
    }; // class RobotMalfunctionOnPartPlacement

    //==============================================================================
    /**
     * @brief Class to represent the fields for the sensor blackout challenge
     *
     * This challenge is triggered when an order is submitted
     */
    class RobotMalfunctionOnSubmission : public RobotMalfunction
    {
    public:
        /**
         * @brief Construct a new RobotMalfunctionOnSubmission object
         *
         * @param duration Duration of the challenge
         * @param robots_to_disable  List of robots to disable
         * @param trigger_order_id ID of the submitted order to trigger the challenge
         */

        RobotMalfunctionOnSubmission(double duration,
                                     const std::vector<std::string> &sensors_to_disable,
                                     std::string trigger_order_id) : RobotMalfunction(duration, sensors_to_disable),
                                                                     trigger_order_id_(trigger_order_id) {}

        /**
         * @brief Get the Trigger Order Id object
         *
         * @return std::string
         */
        std::string GetTriggerOrderId() const { return trigger_order_id_; }

    private:
        //! ID of a submitted order to trigger the challenge
        std::string trigger_order_id_;
    }; // class RobotMalfunctionOnSubmission

    //==============================================================================
    /**
     * @brief Class to represent the fields for the faulty part challenge
     */
    class FaultyPartChallenge
    {
    public:
        /**
         * @brief Construct a new FaultyPartChallenge object
         *
         * @param _msg FaultyPartChallenge message
         */
        FaultyPartChallenge(ariac_msgs::msg::FaultyPartChallenge _msg)
        {
            order_id_ = _msg.order_id;
            faulty_quadrants_ = {{1, _msg.quadrant1},
                                 {2, _msg.quadrant2},
                                 {3, _msg.quadrant3},
                                 {4, _msg.quadrant4}};
            quadrant_checked_ = {{1, false},
                                 {2, false},
                                 {3, false},
                                 {4, false}};
        }

        /**
         * @brief Get the ID of the order
         * @return std::string ID of the order
         */
        std::string GetOrderId() const { return order_id_; }
        /**
         * @brief Get whether a quadrant is faulty
         * @return bool True if the quadrant is faulty
         */
        bool IsQuadrantFaulty(int q) { return faulty_quadrants_[q]; }
        /**
         * @brief Get whether a quadrant has been checked
         * @return bool True if the quadrant has been checked
         */
        bool WasQuadrantChecked(int q) { return quadrant_checked_[q]; }
        /**
         * @brief Set whether a quadrant has been checked
         * @param q Quadrant to set
         */
        void SetQuadrantChecked(int q) { quadrant_checked_[q] = true; }

    protected:
        const unsigned int type_ = ariac_msgs::msg::Challenge::FAULTY_PART;
        std::string order_id_;
        std::map<int, bool> faulty_quadrants_;
        std::map<int, bool> quadrant_checked_;
    };

    
    //==============================================================================
    class InsertPart
    {
    public:
        InsertPart(const Part &part,
                   std::string model_name,
                   geometry_msgs::msg::Pose pose_in_insert) : part_(part),
                                                              model_name_(model_name),
                                                              pose_in_insert_(pose_in_insert)
        {
        }

        /**
         * @brief Get the Part
         *
         * @return Part Part object
         */
        Part GetPart() const { return part_; }
        /**
         * @brief Get the Model Name
         *
         * @return std::string Model name
         */
        std::string GetModelName() const { return model_name_; }

        /**
         * @brief Get the Pose In Insert
         *
         * @return geometry_msgs::msg::Pose Pose in the insert
         */
        geometry_msgs::msg::Pose GetPoseInInsert() const { return pose_in_insert_; }

        /**
         * @brief Whether the part is of the correct type
         *
         * @param type  Type to check
         * @return true  If the part is of the correct type
         * @return false  If the part is not of the correct type
         */
        bool isCorrectType(unsigned int type) { return type == part_.GetType(); }
        /**
         * @brief Whether the part is of the correct color
         *
         * @param color  Color to check
         * @return true  If the part is of the correct color
         * @return false  If the part is not of the correct color
         */
        bool isCorrectColor(unsigned int color) { return color == part_.GetColor(); }

    private:
        //! The part object
        Part part_;
        //! The model name
        std::string model_name_;
        //! The pose of the part in the insert
        geometry_msgs::msg::Pose pose_in_insert_;
    };

    //==============================================================================
    class KitTrayPart
    {
    public:
        KitTrayPart(const Part &part,
                    std::string model_name,
                    geometry_msgs::msg::Pose pose_on_tray) : part_(part),
                                                             model_name_(model_name),
                                                             pose_on_tray_(pose_on_tray)
        {
            // Determine which quadrant based on pose
            double x = pose_on_tray_.position.x;
            double y = pose_on_tray_.position.y;
            if (x < 0.0 && y >= 0.0)
                quadrant_ = 1;
            else if (x >= 0.0 && y >= 0.0)
                quadrant_ = 2;
            else if (x < 0.0 && y < 0.0)
                quadrant_ = 3;
            else if (x >= 0.0 && y < 0.0)
                quadrant_ = 4;
        }

        /**
         * @brief Get the Quadrant
         *
         * @return unsigned int  Quadrant number
         */
        unsigned int GetQuadrant() const { return quadrant_; }
        /**
         * @brief Get the Part
         *
         * @return Part Part object
         */
        Part GetPart() const { return part_; }
        /**
         * @brief Get the Model Name
         *
         * @return std::string Model name
         */
        std::string GetModelName() const { return model_name_; }

        /**
         * @brief Whether the part is of the correct type
         *
         * @param type  Type to check
         * @return true  If the part is of the correct type
         * @return false  If the part is not of the correct type
         */
        bool isCorrectType(unsigned int type) { return type == part_.GetType(); }
        /**
         * @brief Whether the part is of the correct color
         *
         * @param color  Color to check
         * @return true  If the part is of the correct color
         * @return false  If the part is not of the correct color
         */
        bool isCorrectColor(unsigned int color) { return color == part_.GetColor(); }
        /**
         * @brief Whether the part is faulty
         *
         * @return true  If the part is faulty
         * @return false  If the part is not faulty
         */
        bool isFaulty() { return model_name_.find("faulty") != std::string::npos; }
        /**
         * @brief Whether the part is flipped
         *
         * @return true  If the part is flipped
         * @return false  If the part is not flipped
         */
        bool isFlipped()
        {
            KDL::Frame part_on_tray;
            tf2::fromMsg(pose_on_tray_, part_on_tray);
            KDL::Vector part_z = part_on_tray * KDL::Vector(0, 0, 1);

            // Calculate the angle between the two vectors
            double angle = KDL::acos(KDL::dot(KDL::Vector(0, 0, 1), part_z) / (part_z.Norm()));

            // Return that the part is flipped if angle is greater than ~10deg
            if (angle > -0.2 && angle < 0.2)
            {
                return false;
            }
            else
            {
                return true;
            }
        }

    private:
        //! The quadrant the part is in
        unsigned int quadrant_;
        //! The part object
        Part part_;
        //! The model name
        std::string model_name_;
        //! The pose of the part on the tray
        geometry_msgs::msg::Pose pose_on_tray_;
    };

    //==============================================================================
    /**
     * @brief Class to represent a kitting shipment
     */
    class KittingShipment
    {
    public:
        /**
         * @brief Construct a new Kitting Shipment object
         *
         * @param tray_id  ID of the tray
         * @param tray_parts  Vector of KitTrayPart objects
         */
        KittingShipment(unsigned int tray_id,
                        const std::vector<KitTrayPart> &tray_parts) : tray_id_(tray_id),
                                                                      tray_parts_(tray_parts) {}

        /**
         * @brief Get the Tray Id
         *
         * @return unsigned int  Tray ID
         */
        unsigned int GetTrayId() const { return tray_id_; }
        /**
         * @brief Get the Tray Parts
         *
         * @return const std::vector<KitTrayPart>&  Vector of KitTrayPart objects
         */
        const std::vector<KitTrayPart> &GetTrayParts() const { return tray_parts_; }

        /**
         * @brief Helper function to print the contents of the shipment
         *
         * @return std::string  String representation of the shipment
         */
        std::string DebugString()
        {
            std::string s = "Kit Tray ID: " + std::to_string(tray_id_);

            for (auto part : tray_parts_)
            {
                s += "\n\tPart: (type: " + ConvertPartTypeToString(part.GetPart().GetType()) +
                     ", color: " + ConvertPartColorToString(part.GetPart().GetColor()) +
                     ", quad: " + std::to_string(part.GetQuadrant()) + ")";
            }

            return s;
        }

    private:
        //! The tray ID

        unsigned int tray_id_;
        //! The parts on the tray

        std::vector<KitTrayPart> tray_parts_;
    }; // class KittingShipment

    //==============================================================================
    /**
     * @brief Class to represent an assembly shipment
     */
    class AssemblyShipment
    {
    public:
        /**
         * @brief Construct a new Assembly Shipment object
         *
         * @param station  ID of the tray
         * @param assembly_parts  Vector of KitTrayPart objects
         */
        AssemblyShipment(unsigned int station,
                         const std::vector<InsertPart> &insert_parts) : station_(station),
                                                                        insert_parts_(insert_parts) {}

        /**
         * @brief Get the station
         *
         * @return unsigned int  Station
         */
        unsigned int GetStation() const { return station_; }
        /**
         * @brief Get the assembly parts
         *
         * @return const std::vector<AssemblyPart>&  Vector of AssemblyPart objects
         */
        const std::vector<InsertPart> &GetInsertParts() const { return insert_parts_; }

        /**
         * @brief Helper function to print the contents of the shipment
         *
         * @return std::string  String representation of the shipment
         */
        std::string DebugString()
        {
            std::string s = "Station: " + std::to_string(station_);

            for (auto part : insert_parts_)
            {
                s += "\n\tPart: (type: " + ConvertPartTypeToString(part.GetPart().GetType()) +
                     ", color: " + ConvertPartColorToString(part.GetPart().GetColor()) + ")";
            }

            return s;
        }

    private:
        //! Assembly station
        unsigned int station_;
        //! The parts in the insert
        std::vector<InsertPart> insert_parts_;
    };

} // namespace ariac_common

#endif // ARIAC_PLUGINS__ARIAC_COMMON_HPP_