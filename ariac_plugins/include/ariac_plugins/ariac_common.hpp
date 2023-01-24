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

namespace ariac_common
{

    // forward declarations
    class Order;
    class OrderTemporal;
    class Part;
    class Quadrant;

    //==============================================================================
    std::string static ConvertPartTypeToString(unsigned int _part_type)
    {
        if (_part_type == ariac_msgs::msg::Part::BATTERY)
            return "battery";
        else if (_part_type == ariac_msgs::msg::Part::PUMP)
            return "pump";
        else if (_part_type == ariac_msgs::msg::Part::REGULATOR)
            return "regulator";
        else if (_part_type == ariac_msgs::msg::Part::SENSOR)
            return "sensor";
        else
            return "unknown";
    }

    //==============================================================================
    std::string static ConvertAssemblyStationToString(unsigned int _station_id)
    {
        if (_station_id == ariac_msgs::msg::AssemblyTask::AS1)
            return "as1";
        else if (_station_id == ariac_msgs::msg::AssemblyTask::AS2)
            return "as2";
        else if (_station_id == ariac_msgs::msg::AssemblyTask::AS3)
            return "as3";
        else if (_station_id == ariac_msgs::msg::AssemblyTask::AS4)
            return "as4";
        else
            return "unknown";
    }

    //==============================================================================
    std::string static ConvertDestinationToString(unsigned int _destination, unsigned int _agv_id)
    {
        if (_agv_id == 1 || _agv_id == 2)
        {
            if (_destination == ariac_msgs::msg::KittingTask::ASSEMBLY_FRONT)
                return "as1";
            else if (_destination == ariac_msgs::msg::KittingTask::ASSEMBLY_BACK)
                return "as2";
        }
        else if (_agv_id == 3 || _agv_id == 4)
        {
            if (_destination == ariac_msgs::msg::KittingTask::ASSEMBLY_FRONT)
                return "as3";
            else if (_destination == ariac_msgs::msg::KittingTask::ASSEMBLY_BACK)
                return "as4";
        }
        if (_destination == ariac_msgs::msg::KittingTask::KITTING)
            return "kitting";
        else if (_destination == ariac_msgs::msg::KittingTask::WAREHOUSE)
            return "warehouse";
        else
            return "unknown";
    }

    //==============================================================================
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
    std::string static ConvertOrderTypeToString(unsigned int _order_type)
    {
        if (_order_type == ariac_msgs::msg::Order::ASSEMBLY)
            return "assembly";
        else if (_order_type == ariac_msgs::msg::Order::KITTING)
            return "kitting";
        else if (_order_type == ariac_msgs::msg::Order::COMBINED)
            return "combined";
        else
            return "unknown";
    }

    class Score
    {
    public:
        Score() : score_(0), penalty_(0) {}

        void AddScore(double _score) { score_ += _score; }
        void AddPenalty(double _penalty) { penalty_ += _penalty; }

        double GetScore() const { return score_; }
        double GetPenalty() const { return penalty_; }

        friend std::ostream &operator<<(std::ostream &_out,
                                        const Score &_obj)
        {
            _out << "Score: " << _obj.score_ << " Penalty: " << _obj.penalty_;

            return _out;
        }

    private:
        double score_;
        double penalty_;
    };

    //==============================================================================
    class Part
    {
    public:
        Part(unsigned int _color, unsigned int _type) : color_(_color), type_(_type) {}

        unsigned int GetColor() const { return color_; }
        unsigned int GetType() const { return type_; }

        friend std::ostream &operator<<(std::ostream &_out,
                                        const Part &_obj)
        {
            _out << "[" << ConvertPartTypeToString(_obj.type_) << "," << ConvertPartColorToString(_obj.color_) << "]";

            return _out;
        }

    private:
        unsigned int color_;
        unsigned int type_;
    };

    //==============================================================================
    class KittingPart
    {
    public:
        KittingPart(unsigned _quadrant, const Part &_part) : quadrant_(_quadrant), part_(_part) {}

        //====================
        // GETTERS & SETTERS
        //====================
        unsigned int GetQuadrant() const { return quadrant_; }
        Part GetPart() const { return part_; }

        friend std::ostream &operator<<(std::ostream &_out,
                                        const KittingPart &_obj)
        {
            _out << "   ------" << std::endl;
            _out << "   Part: " << _obj.part_ << std::endl;
            _out << "   Quadrant: " << _obj.quadrant_;
            return _out;
        }

    private:
        unsigned int quadrant_;
        Part part_;
    };

    //==============================================================================
    class AssemblyPart
    {
    public:
        AssemblyPart(const Part &_part,
                     const ignition::math::Pose3d &_part_pose,
                     const ignition::math::Vector3<double> &_part_direction) : part_(_part),
                                                                               part_pose_(_part_pose),
                                                                               part_direction_(_part_direction) {}

        friend std::ostream &operator<<(std::ostream &_out,
                                        const AssemblyPart &_obj)
        {
            _out << "   ------" << std::endl;
            _out << "   Part: " << _obj.part_ << std::endl;
            _out << "   Assembled Pose: [" << _obj.part_pose_.Pos().X() << ","
                 << _obj.part_pose_.Pos().Y() << "," << _obj.part_pose_.Pos().Z() << "]"
                 << "[" << _obj.part_pose_.Rot().X() << "," << _obj.part_pose_.Rot().Y() << ","
                 << _obj.part_pose_.Rot().Z() << "," << _obj.part_pose_.Rot().W() << "]" << std::endl;
            _out << "   Assembled Direction: [" << _obj.part_direction_[0] << ","
                 << _obj.part_direction_[1] << "," << _obj.part_direction_[2] << "]";

            return _out;
        }

        Part GetPart() const { return part_; }
        ignition::math::Pose3d GetPartPose() const { return part_pose_; }
        ignition::math::Vector3<double> GetPartDirection() const { return part_direction_; }

    private:
        Part part_;
        ignition::math::Pose3d part_pose_;
        ignition::math::Vector3<double> part_direction_;
    };

    //==============================================================================
    class AssemblyTask
    {
    public:
        AssemblyTask(std::vector<unsigned int> _agv_numbers,
                     unsigned int _station,
                     const std::vector<AssemblyPart> &_products) : agv_numbers_(_agv_numbers),
                                                                   station_(_station),
                                                                   products_(_products) {}

        friend std::ostream &operator<<(std::ostream &_out,
                                        const AssemblyTask &_obj)
        {
            _out << "   Assembly Task" << std::endl;
            _out << "   ================" << std::endl;

            if (_obj.agv_numbers_.size() == 1)
                _out << "   AGV: [" << _obj.agv_numbers_[0] << "]" << std::endl;
            else
            {
                int counter = _obj.agv_numbers_.size();

                _out << "   AGV: [";
                for (auto agv_number : _obj.agv_numbers_)
                {
                    counter--;
                    _out << agv_number;
                    if (counter > 0)
                        _out << ",";
                }

                _out << "]" << std::endl;
            }

            // stations
            _out << "   Station: " << ConvertAssemblyStationToString(_obj.station_) << std::endl;

            // Products
            _out << "   ================" << std::endl;
            _out << "   Products: " << std::endl;
            for (const auto &product : _obj.products_)
            {
                _out << product << std::endl;
            }

            return _out;
        }

        const std::vector<unsigned int> &GetAgvNumbers() const { return agv_numbers_; }
        unsigned int GetStation() const { return station_; }
        const std::vector<AssemblyPart> &GetProducts() const { return products_; }

    private:
        std::vector<unsigned int> agv_numbers_;
        unsigned int station_;
        std::vector<AssemblyPart> products_;
    };

    //==============================================================================
    /**
     * @brief Class to represent the fields for a kitting task
     */
    class KittingTask
    {
    public:
        KittingTask(unsigned int _agv_number,
                    unsigned int _tray_id,
                    unsigned int _destination,
                    const std::vector<KittingPart> &_products) : agv_number_(_agv_number),
                                                                 tray_id_(_tray_id),
                                                                 destination_(_destination),
                                                                 products_(_products) {}
        friend std::ostream &operator<<(std::ostream &_out,
                                        const KittingTask &_obj)
        {
            _out << "   Kitting Task" << std::endl;
            _out << "   ================" << std::endl;

            _out << "   AGV: " << _obj.agv_number_ << std::endl;
            _out << "   Tray ID: " << _obj.tray_id_ << std::endl;

            // Destination
            _out << "   Destination: " << ConvertDestinationToString(_obj.destination_, _obj.agv_number_) << std::endl;

            // Products
            _out << "   ================" << std::endl;
            _out << "   Products: " << std::endl;
            for (const auto &product : _obj.products_)
            {
                _out << product << std::endl;
            }

            return _out;
        }

        unsigned int GetAgvNumber() const { return agv_number_; }
        unsigned int GetTrayId() const { return tray_id_; }
        unsigned int GetDestination() const { return destination_; }
        const std::vector<KittingPart> &GetProducts() const { return products_; }

    private:
        unsigned int agv_number_;
        unsigned int tray_id_;
        unsigned int destination_;
        std::vector<KittingPart> products_;
    };

    //==============================================================================
    /**
     * @brief Class to represent the fields for a combined task
     */
    class CombinedTask
    {
    public:
        CombinedTask(unsigned int _station,
                     const std::vector<AssemblyPart> &_products) : station_(_station),
                                                                   products_(_products) {}

        unsigned int GetStation() const { return station_; }
        const std::vector<AssemblyPart> &GetProducts() const { return products_; }

        friend std::ostream &operator<<(std::ostream &_out,
                                        const CombinedTask &_obj)
        {
            _out << "   Combined Task" << std::endl;
            _out << "   ================" << std::endl;

            // stations
            _out << "   Station: " << ConvertAssemblyStationToString(_obj.station_) << std::endl;

            // Products
            _out << "   ================" << std::endl;
            _out << "   Products: " << std::endl;
            for (const auto &product : _obj.products_)
            {
                _out << product << std::endl;
            }

            return _out;
        }

    private:
        unsigned int station_;
        std::vector<AssemblyPart> products_;
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
         * @param _duration The duration of the blackout in seconds
         * @param _sensors_to_disable List of sensors to disable
         */
        SensorBlackout(double _duration,
                       const std::vector<std::string> &_sensors_to_disable) : duration_(_duration),
                                                                              sensors_to_disable_(_sensors_to_disable) {}

        double GetDuration() const { return duration_; }
        const std::vector<std::string> &GetSensorsToDisable() const { return sensors_to_disable_; }
        unsigned int GetType() const { return type_; }

        double GetStartTime() const { return start_time_; }
        void SetStartTime(double _start_time) { start_time_ = _start_time; }

        bool HasStarted() const { return started_; }
        void SetStarted() { started_ = true; }

        bool HasCompleted() const { return completed_; }
        void SetCompleted() { completed_ = true; }

        void SetStopTime(double _stop_time) { stop_time_ = _stop_time; }
        double GetStopTime() const { return stop_time_; }

    protected:
        const unsigned int type_ = ariac_msgs::msg::Challenge::SENSOR_BLACKOUT;
        //! Duration of the challenge
        double duration_;
        //! List of sensors to disable
        std::vector<std::string> sensors_to_disable_;

        bool started_ = false;
        bool completed_ = false;

        double start_time_;
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
         * @param _duration Duration of the blackout in seconds
         * @param _sensors_to_disable List of sensors to disable
         * @param _trigger_time Time at which the challenge should be triggered
         */
        SensorBlackoutTemporal(double _duration,
                               const std::vector<std::string> &_sensors_to_disable,
                               double _trigger_time) : SensorBlackout(_duration, _sensors_to_disable),
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
         * @param _duration Duration of the blackout
         * @param _sensors_to_disable List of sensors to disable
         * @param _part Part to trigger the challenge
         * @param _agv AGV on which the part is placed
         */

        SensorBlackoutOnPartPlacement(double _duration,
                                      const std::vector<std::string> &_sensors_to_disable,
                                      std::shared_ptr<Part> _part,
                                      unsigned int _agv) : SensorBlackout(_duration, _sensors_to_disable),
                                                           part_(_part),
                                                           agv_(_agv) {}

        std::shared_ptr<Part> GetPart() const { return part_; }
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
         * @param _duration Duration of the blackout
         * @param _sensors_to_disable List of sensors to disable
         * @param _trigger_order_id ID of a submitted order
         */

        SensorBlackoutOnSubmission(double _duration,
                                   const std::vector<std::string> &_sensors_to_disable,
                                   std::string _trigger_order_id) : SensorBlackout(_duration, _sensors_to_disable),
                                                                    trigger_order_id_(_trigger_order_id) {}

        std::string GetTriggerOrderId() const { return trigger_order_id_; }

    private:
        //! ID of a submitted order
        std::string trigger_order_id_;
    }; // class SensorBlackoutKittingAction

    class Quadrant
    {
    public:
        Quadrant(int _quadrant_number,
                 bool _is_correct_part_type,
                 bool _is_correct_part_color,
                 bool _is_faulty,
                 bool _is_flipped,
                 int _score) : quadrant_number_(_quadrant_number),
                               is_correct_part_type_(_is_correct_part_type),
                               is_correct_part_color_(_is_correct_part_color),
                               is_faulty_(_is_faulty),
                               is_flipped_(_is_flipped),
                               score_(_score)
        {
        }

        friend std::ostream &operator<<(std::ostream &_out,
                                        const Quadrant &_obj)
        {
            int quadrant_num = 0;
            if (_obj.quadrant_number_ == 1)
                quadrant_num = 1;
            else if (_obj.quadrant_number_ == 2)
                quadrant_num = 2;
            else if (_obj.quadrant_number_ == 3)
                quadrant_num = 3;
            else if (_obj.quadrant_number_ == 4)
                quadrant_num = 4;
            _out << "-----------" << std::endl;
            _out << "   Quadrant" << quadrant_num << " score: " << _obj.score_ << std::endl;
            _out << "   -----------" << std::endl;
            _out << "   Correct part type: " << std::boolalpha << _obj.is_correct_part_type_ << std::endl;
            _out << "   Correct part color: " << std::boolalpha << _obj.is_correct_part_color_ << std::endl;
            _out << "   Faulty Part: " << std::boolalpha << _obj.is_faulty_ << std::endl;
            _out << "   Flipped Part: " << std::boolalpha << _obj.is_flipped_ << std::endl;

            return _out;
        }

    protected:
        int quadrant_number_;
        bool is_correct_part_type_;
        bool is_correct_part_color_;
        bool is_faulty_;
        bool is_flipped_;
        int score_;
    };

    //==============================================================================
    class KittingScore
    {
    public:
        KittingScore(std::string _order_id,
                     int _kit_score,
                     int _tray_score,
                     std::shared_ptr<Quadrant> _quadrant1,
                     std::shared_ptr<Quadrant> _quadrant2,
                     std::shared_ptr<Quadrant> _quadrant3,
                     std::shared_ptr<Quadrant> _quadrant4,
                     int _bonus,
                     int _penalty) : order_id_(_order_id),
                                     kit_score_(_kit_score),
                                     tray_score_(_tray_score),
                                     quadrant1_(_quadrant1),
                                     quadrant2_(_quadrant2),
                                     quadrant3_(_quadrant3),
                                     quadrant4_(_quadrant4),
                                     bonus_(_bonus),
                                     penalty_(_penalty) {}

        /**
         * @brief Overload the << operator to print the score
         *
         * @param _out  Output stream
         * @param _obj  KittingScore object
         * @return std::ostream& Output stream
         */
        friend std::ostream &operator<<(std::ostream &_out,
                                        const KittingScore &_obj)
        {
            _out << "\n================" << std::endl;
            _out << "Score for Order " << _obj.order_id_ << ": " << _obj.kit_score_ << std::endl;
            _out << "================" << std::endl;

            auto quad1 = _obj.quadrant1_;
            auto quad2 = _obj.quadrant2_;
            auto quad3 = _obj.quadrant3_;
            auto quad4 = _obj.quadrant4_;

            _out << "   Correct tray score: " << _obj.tray_score_ << std::endl;
            if (_obj.quadrant1_)
                _out << "   " << *_obj.quadrant1_ << std::endl;
            else
            {
                _out << "   -----------" << std::endl;
                _out << "   Quadrant 1: Not used" << std::endl;
            }

            if (_obj.quadrant2_)
                _out << "   " << *_obj.quadrant2_ << std::endl;
            else
            {
                _out << "   -----------" << std::endl;
                _out << "   Quadrant 2: Not used" << std::endl;
            }
            if (_obj.quadrant3_)
                _out << "   " << *_obj.quadrant3_ << std::endl;

            else
            {
                _out << "   -----------" << std::endl;
                _out << "   Quadrant 3: Not used" << std::endl;
            }
            if (_obj.quadrant4_)
                _out << "   " << *_obj.quadrant4_ << std::endl;

            else
            {
                _out << "   -----------" << std::endl;
                _out << "   Quadrant 4: Not used" << std::endl;
            }
            _out << "   -----------" << std::endl;
            _out << "   Bonus: " << _obj.bonus_ << std::endl;
            _out << "   Extra Parts Penalty: " << _obj.penalty_ << std::endl;

            return _out;
        }

        int GetKitScore() const { return kit_score_; }

    protected:
        std::string order_id_;
        int kit_score_;
        int tray_score_;
        std::shared_ptr<Quadrant> quadrant1_ = nullptr;
        std::shared_ptr<Quadrant> quadrant2_ = nullptr;
        std::shared_ptr<Quadrant> quadrant3_ = nullptr;
        std::shared_ptr<Quadrant> quadrant4_ = nullptr;
        int bonus_;
        int penalty_;

    }; // class KittingScore

    //==============================================================================
    class Order
    {
    public:
        /**
         * @brief Construct a new Order object
         *
         * @param _order_id Unique id of the order
         * @param _order_type  Type of the order.
         * @param _high_priority Priority of the order (true or false)
         * @param _trial_time_limit  Time limit for the trial
         */
        Order(std::string _id,
              unsigned int _type,
              bool _priority,
              double _trial_time_limit) : announced_(false),
                                          submitted_(false),
                                          id_(_id),
                                          type_(_type),
                                          priority_(_priority),
                                          trial_time_limit_(_trial_time_limit) {}
        ~Order() = default;

        friend std::ostream &operator<<(std::ostream &_out,
                                        const Order &_obj)
        {
            _out << "=================" << std::endl;
            _out << "Announcing Order " << _obj.id_ << std::endl;
            _out << "=================" << std::endl;
            if (_obj.type_ == ariac_msgs::msg::Order::KITTING)
                _out << "Type: Kitting" << std::endl;
            else if (_obj.type_ == ariac_msgs::msg::Order::ASSEMBLY)
                _out << "Type: Assembly" << std::endl;
            else if (_obj.type_ == ariac_msgs::msg::Order::COMBINED)
                _out << "Type: Combined" << std::endl;
            _out << "Priority: " << _obj.priority_ << std::endl;
            if (_obj.type_ == ariac_msgs::msg::Order::KITTING)
                _out << *_obj.kitting_task_;
            else if (_obj.type_ == ariac_msgs::msg::Order::ASSEMBLY)
                _out << *_obj.assembly_task_;
            else if (_obj.type_ == ariac_msgs::msg::Order::COMBINED)
                _out << *_obj.combined_task_;
            return _out;
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
         * @brief Set the KittingScore object for the order
         *
         * @param _kitting_score Pointer to the KittingScore object for the order
         */
        virtual void SetKittingScore(std::shared_ptr<KittingScore> _kitting_score)
        {
            kitting_score_ = _kitting_score;
        }
        /**
         * @brief Get the KittingScore object for the order
         *
         * @return std::shared_ptr<KittingScore> Pointer to the KittingScore object for the order
         */
        std::shared_ptr<KittingScore> GetKittingScore() const { return kitting_score_; }

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
         * @brief Score computed for the current order
         *
         */
        std::shared_ptr<KittingScore> kitting_score_ = nullptr;
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
         * @param _id Unique id of the order
         * @param _type Type of the order
         * @param _priority Priority of the order (true or false)
         * @param _trial_time_limit Time limit for the trial
         * @param _announcement_time Time at which the order should be announced
         */
        ~OrderTemporal() = default;

        OrderTemporal(std::string _id,
                      unsigned int _type,
                      bool _priority,
                      double _trial_time_limit,
                      double _announcement_time) : Order(_id, _type, _priority, _trial_time_limit),
                                                   announcement_time_(_announcement_time) {}

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
         * @param _id Unique id of the order
         * @param _type Type of the order
         * @param _priority Priority of the order (true or false)
         * @param _trial_time_limit Time limit for the trial
         * @param _agv Announcement: AGV on which the part is placed
         */
        OrderOnPartPlacement(std::string _id,
                             unsigned int _type,
                             bool _priority,
                             double _trial_time_limit,
                             unsigned int _agv,
                             std::shared_ptr<Part> _part) : Order(_id, _type, _priority, _trial_time_limit),
                                                            agv_(_agv), part_(_part) {}

        unsigned int GetAgv() const
        {
            return agv_;
        }

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
         * @param _id Unique id of the order
         * @param _type Type of the order
         * @param _priority Priority of the order (true or false)
         * @param _trial_time_limit Time limit for the trial
         * @param _order_id Id of the order submitted by the competitor
         */
        OrderOnSubmission(std::string _id,
                          unsigned int _type,
                          bool _priority,
                          double _trial_time_limit,
                          std::string _order_id) : Order(_id, _type, _priority, _trial_time_limit),
                                                   order_id_(_order_id) {}

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
         * @param _duration Duration of the challenge
         * @param _robots_to_disable  List of robots to disable
         */
        RobotMalfunction(double _duration,
                         const std::vector<std::string> &_robots_to_disable) : duration_(_duration),
                                                                               robots_to_disable_(_robots_to_disable) {}

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
         * @param _duration Duration of the challenge
         * @param _robots_to_disable  List of robots to disable
         * @param _trigger_time Competition time at which the challenge is triggered
         */
        RobotMalfunctionTemporal(double _duration,
                                 const std::vector<std::string> &_robots_to_disable,
                                 double _trigger_time) : RobotMalfunction(_duration, _robots_to_disable),
                                                         trigger_time_(_trigger_time) {}
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
         * @param _duration Duration of the blackout
         * @param _robots_to_disable  List of robots to disable
         * @param _part Part to trigger the challenge
         * @param _agv AGV on which the part is placed
         */

        RobotMalfunctionOnPartPlacement(double _duration,
                                        const std::vector<std::string> &_robots_to_disable,
                                        std::shared_ptr<Part> _part,
                                        unsigned int _agv) : RobotMalfunction(_duration, _robots_to_disable),
                                                             part_(_part),
                                                             agv_(_agv) {}

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
         * @param _duration Duration of the challenge
         * @param _robots_to_disable  List of robots to disable
         * @param _trigger_order_id ID of the submitted order to trigger the challenge
         */

        RobotMalfunctionOnSubmission(double _duration,
                                     const std::vector<std::string> &_sensors_to_disable,
                                     std::string _trigger_order_id) : RobotMalfunction(_duration, _sensors_to_disable),
                                                                      trigger_order_id_(_trigger_order_id) {}

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

        std::string GetOrderId() const { return order_id_; }
        bool IsQuadrantFaulty(int q) { return faulty_quadrants_[q]; }
        bool WasQuadrantChecked(int q) { return quadrant_checked_[q]; }
        void SetQuadrantChecked(int q) { quadrant_checked_[q] = true; }

    protected:
        const unsigned int type_ = ariac_msgs::msg::Challenge::FAULTY_PART;
        std::string order_id_;
        std::map<int, bool> faulty_quadrants_;
        std::map<int, bool> quadrant_checked_;
    };

    class KitTrayPart
    {
    public:
        KitTrayPart(const Part &_part,
                    std::string _model_name,
                    geometry_msgs::msg::Pose _pose_on_tray) : part_(_part),
                                                              model_name_(_model_name),
                                                              pose_on_tray_(_pose_on_tray)
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

        unsigned int GetQuadrant() const { return quadrant_; }
        Part GetPart() const { return part_; }
        std::string GetModelName() const { return model_name_; }

        bool isCorrectType(unsigned int type) { return type == part_.GetType(); }
        bool isCorrectColor(unsigned int color) { return color == part_.GetColor(); }
        bool isFaulty() { return model_name_.find("faulty") != std::string::npos; }
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
        unsigned int quadrant_;
        Part part_;
        std::string model_name_;
        geometry_msgs::msg::Pose pose_on_tray_;
    };

    class KittingShipment
    {
    public:
        KittingShipment(unsigned int _tray_id,
                        const std::vector<KitTrayPart> &_tray_parts) : tray_id_(_tray_id),
                                                                       tray_parts_(_tray_parts) {}

        unsigned int GetTrayId() const { return tray_id_; }
        const std::vector<KitTrayPart> &GetTrayParts() const { return tray_parts_; }

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
        unsigned int tray_id_;
        std::vector<KitTrayPart> tray_parts_;
    };

} // namespace ariac_common

#endif // ARIAC_PLUGINS__ARIAC_COMMON_HPP_