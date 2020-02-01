/*
 * Copyright (C) 2016 Open Source Robotics Foundation
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
#ifndef _ARIAC_HH_
#define _ARIAC_HH_

#include <ostream>
#include <map>
#include <string>
#include <vector>

#include <gazebo/gazebo.hh>
#include <ignition/math/Pose3.hh>

namespace ariac
{
  using namespace gazebo;

  typedef std::string KitType_t;
  typedef std::string ShipmentType_t;
  typedef std::string OrderID_t;

  /// \brief The score of a shipment.
  class ShipmentScore
  {
    /// \brief Stream insertion operator.
    /// \param[in] _out output stream.
    /// \param[in] _obj ShipmentScore object to output.
    /// \return The output stream
    public: friend std::ostream &operator<<(std::ostream &_out,
                                            const ShipmentScore &_obj)
    {
      _out << "<shipment_score " << _obj.shipmentType << ">" << std::endl;
      _out << "\tCompletion score: [" << _obj.total() << "]" << std::endl;
      _out << "\tComplete: [" << (_obj.isComplete ? "true" : "false") << "]" << std::endl;
      _out << "\tSubmitted: [" << (_obj.isSubmitted ? "true" : "false") << "]" << std::endl;
      _out << "\tProduct presence score: [" << _obj.productPresence << "]" << std::endl;
      _out << "\tAll products bonus: [" << _obj.allProductsBonus << "]" << std::endl;
      _out << "\tProduct pose score: [" << _obj.productPose << "]" << std::endl;
      _out << "\tDelivered to correct agv: [" << (_obj.correctAGV ? "true" : "false") << "]" << std::endl;
      _out << "</shipment_score>" << std::endl;
      return _out;
    }
    public: ShipmentType_t shipmentType;
            double productPresence = 0.0;
            double allProductsBonus = 0.0;
            double productPose = 0.0;
            bool isComplete = false;  // All products present
            bool isSubmitted = false;  // the shipment has been submitted for evaluation
            bool correctAGV = false;  // The shipment was submitted to the desired AGV
            gazebo::common::Time submit_time;  // sim time when shipment submitted

            /// \brief Calculate the total score.
            double total() const
            {
              if (!correctAGV)
              {
                return 0.0;
              }
              return productPresence + allProductsBonus + productPose;
            }
  };

  /// \brief The score of an order.
  class OrderScore
  {
    /// \brief Stream insertion operator.
    /// \param[in] _out output stream.
    /// \param[in] _obj OrderScore object to output.
    /// \return The output stream
    public: friend std::ostream &operator<<(std::ostream &_out,
                                            const OrderScore &_obj)
    {
      _out << "<order_score " << _obj.orderID << ">" << std::endl;
      _out << "\tTotal order score: [" << _obj.total() << "]" << std::endl;
      _out << "\tCompletion score: [" << _obj.completion_score() << "]" << std::endl;
      _out << "\tTime taken: [" << _obj.timeTaken << "]" << std::endl;
      _out << "\tComplete: [" << (_obj.isComplete() ? "true" : "false") << "]" << std::endl;
      _out << "\tPriority: [" << (_obj.priority) << "]\n";
      for (const auto & item : _obj.shipmentScores)
      {
        _out << item.second << std::endl;
      }
      _out << "</order_score>" << std::endl;
      return _out;
    }

    public: std::string csv(bool output_header=true) const
    {
      std::stringstream out;
      // Gather data
      size_t shipments_requested = shipmentScores.size();
      size_t shipments_submitted = 0;
      size_t shipments_completed = 0;
      size_t shipments_wrong_agv = 0;
      for (const auto shipment_tuple : shipmentScores)
      {
        if (shipment_tuple.second.isSubmitted)
        {
          ++shipments_submitted;
          if (shipment_tuple.second.isComplete)
          {
            ++shipments_completed;
          }
          if (!shipment_tuple.second.correctAGV)
          {
            ++shipments_wrong_agv;
          }
        }
      }
      // Output header
      if (output_header)
      {
        out << "\"order_id\",\"completion_score\",\"priority\",\"time\",\"is_complete\","
            << "\"shipments_requested\",\"shipments_submitted\",\"shipments_completed\","
            << "\"shipments_wrong_agv\"\r\n";
      }
      // Output data
      out << "\"" << orderID << "\",";
      out << completion_score() << ",";
      out << priority << ",";
      out << timeTaken << ",";
      out << (isComplete() ? "1" : "0") << ",";
      out << shipments_requested << ",";
      out << shipments_submitted << ",";
      out << shipments_completed << ",";
      out << shipments_wrong_agv << "\r\n";
      return out.str();
    }

    /// \brief Mapping between shipment IDs and scores.
    public: std::map<ShipmentType_t, ShipmentScore> shipmentScores;

            /// \brief ID of the order being scored.
            OrderID_t orderID;

            /// \brief Time in seconds spend on the order.
            double timeTaken = 0.0;

            /// \brief Factor indicating order priority (Allowed values 1 or 3)
            int priority = 1;

            /// \brief simulation time when order was started
            gazebo::common::Time start_time;

            /// \brief Calculate if the order is complete.
            /// \return True if all shipping boxes have been submitted.
            ///   Will return false if there are no shipping boxes in the order.
            bool isComplete() const
            {
              for (const auto & item : this->shipmentScores)
              {
                if (!item.second.isSubmitted)
                {
                  return false;
                }
              }
              return true;
            };

            /// \brief Calculate the total score.
            double total() const
            {
              return completion_score() * priority;
            };

            /// \brief Get score without priority factor.
            double completion_score() const
            {
              double total = 0.0;
              for (const auto & item : this->shipmentScores)
              {
                total += item.second.total();
              }
              return total;
            };
  };

  /// \brief The score of a competition run.
  class GameScore
  {
    /// \brief Stream insertion operator.
    /// \param[in] _out output stream.
    /// \param[in] _obj GameScore object to output.
    /// \return The output stream
    public: friend std::ostream &operator<<(std::ostream &_out,
                                            const GameScore &_obj)
    {
      _out << "<game_score>" << std::endl;
      _out << "\tTotal game score: [" << _obj.total() << "]" << std::endl;
      _out << "\tTotal process time: [" << _obj.totalProcessTime << "]" << std::endl;
      _out << "\tWas arm/arm collision?: [" << _obj.was_arm_arm_collision << "]" << std::endl;
      for (const auto & item : _obj.orderScores)
      {
        _out << item.second << std::endl;
      }
      _out << "</game_score>" << std::endl;
      return _out;
    }

    public: double totalProcessTime = 0.0;
            bool was_arm_arm_collision = false;

            // The score of each of the orders during the game.
            std::map<OrderID_t, OrderScore> orderScores;

            /// \brief Calculate the total score.
            double total() const
            {
              if (was_arm_arm_collision)
              {
                return 0;
              }
              double total = 0;

              for (const auto & item : this->orderScores)
              {
                total += item.second.total();
              }
              return total;
            };
  };

  /// \brief Determine the model name without namespace
  std::string TrimNamespace(const std::string &modelName)
  {
    // Trim namespaces
    size_t index = modelName.find_last_of('|');
    return modelName.substr(index + 1);
  }

  /// \brief Determine the type of a gazebo model from its name
  std::string DetermineModelType(const std::string &modelName)
  {
    std::string modelType(TrimNamespace(modelName));

    // Trim trailing underscore and number caused by inserting multiple of the same model
    size_t index = modelType.find_last_not_of("0123456789");
    if (modelType[index] == '_' && index > 1)
    {
      modelType = modelType.substr(0, index);
    }

    // Trim "_clone" suffix if exists
    index = modelType.rfind("_clone");
    if (index != std::string::npos)
    {
      modelType.erase(index);
    }

    return modelType;
  }

  /// \brief Determine the ID of a gazebo model from its name
  std::string DetermineModelId(const std::string &modelName)
  {
    std::string modelId(TrimNamespace(modelName));

    // Trim trailing underscore and number caused by inserting multiple of the same model
    size_t index = modelId.find_last_not_of("0123456789");
    if (modelId[index] == '_' && index > 1)
    {
      modelId = modelId.substr(index + 1);
    }

    return modelId;
  }

  /// \brief Class to store information about each product contained in a shipment.
  class Product
  {
    /// \brief Stream insertion operator.
    /// \param[in] _out output stream.
    /// \param[in] _obj Shipment object to output.
    /// \return The output stream
    public: friend std::ostream &operator<<(std::ostream &_out,
                                            const Product &_obj)
    {
      _out << "<product>" << std::endl;
      _out << "Type: [" << _obj.type << "]" << std::endl;
      _out << "Faulty: [" << (_obj.isFaulty ? "true" : "false") << "]" << std::endl;
      _out << "Pose: [" << _obj.pose << "]" << std::endl;
      _out << "</product>" << std::endl;
      return _out;
    }

    /// \brief Product type.
    public: std::string type;

    /// \brief Whether or not the product is faulty.
    public: bool isFaulty;

    /// \brief Pose in which the product should be placed.
    public: ignition::math::Pose3d pose;

  };

  /// \brief Class to store information about a shipment.
  class Shipment
  {
    /// \brief Stream insertion operator.
    /// \param[in] _out output stream.
    /// \param[in] _shipment shipment to output.
    /// \return The output stream.
    public: friend std::ostream &operator<<(std::ostream &_out,
                                            const Shipment &_shipment)
    {
      _out << "<shipment type='" << _shipment.shipmentType << "' agv='" << _shipment.agv_id << "'>";
      for (const auto & obj : _shipment.products)
        _out << std::endl << obj;
      _out << std::endl << "</shipment>" << std::endl;

      return _out;
    }

    /// \brief The type of the shipment.
    public: ShipmentType_t shipmentType;

    /// \brief which AGV this shipment should be put into "agv1", "agv2", or "any"
    public: std::string agv_id;

    /// \brief A shipment is composed of multiple products.
    public: std::vector<Product> products;
  };

  /// \brief Class to store information about each object contained in a kit.
  class KitObject
  {
    /// \brief Stream insertion operator.
    /// \param[in] _out output stream.
    /// \param[in] _obj Kit object to output.
    /// \return The output stream
    public: friend std::ostream &operator<<(std::ostream &_out,
                                            const KitObject &_obj)
    {
      _out << "<object>" << std::endl;
      _out << "Type: [" << _obj.type << "]" << std::endl;
      _out << "Faulty: [" << (_obj.isFaulty ? "true" : "false") << "]" << std::endl;
      _out << "Pose: [" << _obj.pose << "]" << std::endl;
      _out << "</object>" << std::endl;
      return _out;
    }

    /// \brief Object type.
    public: std::string type;

    /// \brief Whether or not the object is faulty.
    public: bool isFaulty;

    /// \brief Pose in which the object should be placed.
    public: ignition::math::Pose3d pose;

  };

  /// \brief Class to store information about a kit.
  class Kit
  {
    /// \brief Stream insertion operator.
    /// \param[in] _out output stream.
    /// \param[in] _kit kit to output.
    /// \return The output stream.
    public: friend std::ostream &operator<<(std::ostream &_out,
                                            const Kit &_kit)
    {
      _out << "<kit type='" << _kit.kitType << "'>";
      for (const auto & obj : _kit.objects)
        _out << std::endl << obj;
      _out << std::endl << "</kit>" << std::endl;

      return _out;
    }

    /// \brief The type of the kit.
    public: KitType_t kitType;

    /// \brief A kit is composed of multiple objects.
    public: std::vector<KitObject> objects;
  };

  /// \brief Class to store information about an order.
  class Order
  {
    /// \brief Less than operator.
    /// \param[in] _order Other order to compare.
    /// \return True if this < _order.
    public: bool operator<(const Order &_order) const
    {
      return this->startTime < _order.startTime;
    }

    /// \brief Stream insertion operator.
    /// \param[in] _out output stream.
    /// \param[in] _order Order to output.
    /// \return The output stream.
    public: friend std::ostream &operator<<(std::ostream &_out,
                                            const Order &_order)
    {
      _out << "<Order>" << std::endl;
      _out << "Start time: [" << _order.startTime << "]" << std::endl;
      _out << "Shipments:" << std::endl;
      for (const auto & item : _order.shipments)
      {
        _out << item << std::endl;
      }
      _out << "</order>" << std::endl;

      return _out;
    }

    /// \brief The ID of this order.
    public: OrderID_t orderID;

    /// \brief Simulation time in which the order should be triggered.
    public: double startTime;

    /// \brief After how many unwanted products to interrupt the previous order (-1 for never).
    public: int interruptOnUnwantedProducts;

    /// \brief After how many wanted products to interrupt the previous order (-1 for never).
    public: int interruptOnWantedProducts;

    /// \brief Simulation time in seconds permitted for the order to be
    /// completed before cancelling it. Infinite by default.
    public: double allowedTime;

    /// \brief An order is composed of multiple shipments of different types.
    public: std::vector<Shipment> shipments;

    /// \brief Simulation time in seconds spent on this order.
    public: double timeTaken;
  };
}
#endif
