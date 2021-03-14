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
  typedef std::string KittingShipmentType_t;
  typedef std::string AssemblyShipmentType_t;
  typedef std::string OrderID_t;


  /////////////////////////////////////////////////////////////
  /// \brief The score of a kitting shipment.
  /////////////////////////////////////////////////////////////
  class KittingShipmentScore
  {
    /// \brief Stream insertion operator.
    /// \param[in] _out output stream.
    /// \param[in] _obj ShipmentScore object to output.
    /// \return The output stream
  public:
    friend std::ostream &operator<<(std::ostream &_out,
                                    const KittingShipmentScore &_obj)
    {
      // _out << "kitting_shipment_score" << std::endl;
      // _out << "...shipment type: [" << _obj.kittingShipmentType << "]" << std::endl;
      _out << "      ...completion score: [" << _obj.total() << "]" << std::endl;
      _out << "      ...complete: [" << (_obj.is_kitting_shipment_complete ? "true" : "false") << "]" << std::endl;
      _out << "      ...submitted: [" << (_obj.is_kitting_shipment_submitted ? "true" : "false") << "]" << std::endl;
      _out << "      ...used correct agv: [" << (_obj.is_kitting_correct_agv ? "true" : "false") << "]" << std::endl;
      _out << "      ...sent to correct station: [" << (_obj.is_kitting_correct_destination ? "true" : "false") << "]" << std::endl;
      _out << "      ...product type presence score: [" << _obj.productOnlyTypePresence << "]" << std::endl;
      _out << "      ...product color presence score: [" << _obj.productTypeAndColorPresence << "]" << std::endl;
      _out << "      ...product pose score: [" << _obj.productPose << "]" << std::endl;
      _out << "      ...all products bonus: [" << _obj.allProductsBonus << "]" << std::endl;
      // _out << "</kitting_shipment_score>" << std::endl;
      return _out;
    }

  public:
    KittingShipmentType_t kittingShipmentType;
    double productOnlyTypePresence = 0.0;
    double productTypeAndColorPresence = 0.0;
    double allProductsBonus = 0.0;
    double productPose = 0.0;
    bool is_kitting_shipment_complete = false;  // All products present
    bool is_kitting_shipment_submitted = false; // the shipment has been submitted for evaluation
    bool is_kitting_correct_agv = false;        // The shipment was submitted to the desired AGV
    bool is_kitting_correct_destination = false;
    gazebo::common::Time submit_time; // sim time when shipment submitted

    /// \brief Calculate the total score.
    double total() const
    {
      if (!is_kitting_correct_agv)
      {
        return 0.0;
      }
      if (!is_kitting_correct_destination)
      {
        return 0.0;
      }
      return productOnlyTypePresence + productTypeAndColorPresence + allProductsBonus + productPose;
    }
  };

/////////////////////////////////////////////////////////////
  /// \brief Class to store information about each object contained in a briefcase for assembly.
  /////////////////////////////////////////////////////////////
  class BriefcaseProduct
  {
    /// \brief Stream insertion operator.
    /// \param[in] _out output stream.
    /// \param[in] _obj Assembly object to output.
    /// \return The output stream
  public:
    friend std::ostream &operator<<(std::ostream &_out,
                                    const BriefcaseProduct &_obj)
    {
      _out << "<object>" << std::endl;
      _out << "...type: [" << _obj.productType << "]" << std::endl;
      _out << "...faulty: [" << (_obj.isProductFaulty ? "true" : "false") << "]" << std::endl;
      _out << "...pose: [" << _obj.productPose << "]" << std::endl;
      _out << "</object>" << std::endl;
      return _out;
    }

  public:
  bool isProductCorrectPose = false;
  bool isProductCorrectType = false;
  bool isProductCorrectColor = false;
  bool isProductFaulty = false;
  int productSuccess = 0;
  std::string productName;            /*!< Name of the product. */
  std::string productType;            /*!< Type of the product. */
  ignition::math::Pose3d productPose; /*!< Pose in which the object should be placed. */
  };

  /////////////////////////////////////////////////////////////
  /// \brief The score of a finished product from assembly.
  /////////////////////////////////////////////////////////////
  class AssemblyShipmentScore
  {
    /// \brief Stream insertion operator.
    /// \param[in] _out output stream.
    /// \param[in] _obj ShipmentScore object to output.
    /// \return The output stream
  public:

    


    friend std::ostream &operator<<(std::ostream &_out,
                                    const AssemblyShipmentScore &_obj)
    {
      _out << "      ...completion score: [" << _obj.total() << "]" << std::endl;
      _out << "         +2pts x (correct type AND correct pose): [2pts x " << _obj.computeNbCorrectPoseAndType() << "]" << std::endl;
      _out << "           +1pt x (correct color): [1pt x " << _obj.computeNbCorrectColor() << "]" << std::endl;
      _out << "             +all products bonus: [" << _obj.allProductsBonus << "]" << std::endl;
      _out << "      ...complete: [" << (_obj.isShipmentComplete ? "true" : "false") << "]" << std::endl;
      _out << "      ...shipped: [" << (_obj.isShipmentSubmitted ? "true" : "false") << "]" << std::endl;
      _out << "      ...number of products shipped: [" << (_obj.numberOfProductsInShipment) << "]" << std::endl;
      _out << "      ...has faulty products: [" << (_obj.hasFaultyProduct ? "true" : "false") << "]" << std::endl;
      _out << "      ...has missing products: [" << (_obj.hasMissingProduct ? "true" : "false") << "]" << std::endl;
      _out << "      ...has unwanted products: [" << (_obj.hasUnwantedProduct ? "true" : "false") << "]" << std::endl;
      _out << "      ...number of products shipped with correct type: [" << _obj.numberOfProductsWithCorrectType << "]" << std::endl;
      _out << "      ...number of products shipped with correct pose: [" << _obj.numberOfProductsWithCorrectPose << "]" << std::endl;
      _out << "      ...number of products shipped with correct color: [" << _obj.numberOfProductsWithCorrectColor << "]" << std::endl;
      _out << "      ...correct assembly station: [" << (_obj.isCorrectStation ? "true" : "false") << "]" << std::endl;
      return _out;
    }

  public:
    AssemblyShipmentType_t assemblyShipmentType;
    std::map<std::string, BriefcaseProduct> briefcaseProducts{};
    std::string assemblyStation;
    int allProductsBonus = 0;
    int success = 0;
    int numberOfProductsWithCorrectColor = 0;
    int numberOfProductsInShipment = 0;
    int numberOfProductsWithCorrectType = 0;
    int numberOfProductsWithCorrectPose = 0;
    bool hasFaultyProduct = false;
    bool hasMissingProduct = false;
    bool hasUnwantedProduct = false;
    bool isShipmentComplete = false;  // All products present
    bool isShipmentSubmitted = false; // The finished product has been submitted for evaluation
    bool isCorrectStation = false;                 // The finished product was built at the correct station
    gazebo::common::Time submit_time;            // Sim time when finished product was submitted
    /// \brief Calculate the total score.
    double total() const
    {
      if (!isCorrectStation)
      {
        return 0.0;
      }
      int total = 2 * computeNbCorrectPoseAndType();
      total += computeNbCorrectColor();
      return total + allProductsBonus;
    }

    int computeNbCorrectPoseAndType() const{
      int result{};
      for (const auto& x : briefcaseProducts)
      {
        auto product = x.second;
        if (product.isProductCorrectPose && product.isProductCorrectType){
          result++;
        }  
      }
      return result;
    }

    int computeNbCorrectColor() const
    {
      int result{};

      for (const auto& x : briefcaseProducts)
      {
        auto product = x.second;
        if (product.isProductCorrectPose && product.isProductCorrectType)
        {
          if (product.isProductCorrectColor)
            result++;
        }
      }
      return result;
    }
  };

  /////////////////////////////////////////////////////////////
  /// \brief The score of an order.
  /// The order score consists of kitting score + assembly score.
  /////////////////////////////////////////////////////////////
  class OrderScore
  {
    //@todo Do the same thing for assembly
    /// \brief Stream insertion operator.
    /// \param[in] _out output stream.
    /// \param[in] _obj OrderScore object to output.
    /// \return The output stream
  public:
    friend std::ostream &operator<<(std::ostream &_out,
                                    const OrderScore &_obj)
    {
      _out << "   [order score]" << std::endl;
      _out << "   ...order ID [" << _obj.order_id << "]" << std::endl;
      _out << "   ...total order score: [" << _obj.computeKittingTotal() + _obj.computeAssemblyTotal() << "]" << std::endl;
      _out << "   ...completion score: [" << _obj.computeAssemblyCompletionScore() + _obj.computeKittingCompletionScore() << "]" << std::endl;
      _out << "   ...time taken: [" << _obj.time_taken << "]" << std::endl;
      _out << "   ...priority: [" << (_obj.priority) << "]\n";
      if (_obj.kitting_shipment_scores.size() > 0)
      {
        for (const auto &item : _obj.kitting_shipment_scores)
        {
          _out << "      [kitting score]" << std::endl;
          _out << "      ...shipment type: [" << item.first << "]" << std::endl;
          // _out << item.first<< std::endl;
          _out << item.second << std::endl;
        }
      }
      if (_obj.assembly_shipment_scores.size() > 0)
      {
        for (const auto &item : _obj.assembly_shipment_scores)
        {
          _out << "      [assembly score]" << std::endl;
          _out << "      ...shipment type: [" << item.first << "]" << std::endl;
          _out << item.second << std::endl;
        }
      }
      return _out;
    }

  public:
    std::string csv_kitting(bool output_header = true) const
    {
      std::stringstream out;
      // Gather data
      size_t kitting_shipments_requested = kitting_shipment_scores.size();
      size_t kitting_shipments_submitted = 0;
      size_t kitting_shipments_completed = 0;
      size_t kitting_shipments_wrong_agv = 0;
      size_t kitting_shipments_wrong_station = 0;
      if (kitting_shipments_requested > 0)
      {
        for (const auto shipment_tuple : kitting_shipment_scores)
        {
          if (shipment_tuple.second.is_kitting_shipment_submitted)
          {
            ++kitting_shipments_submitted;
            if (shipment_tuple.second.is_kitting_shipment_complete)
            {
              ++kitting_shipments_completed;
            }
            if (!shipment_tuple.second.is_kitting_correct_agv)
            {
              ++kitting_shipments_wrong_agv;
            }
            if (!shipment_tuple.second.is_kitting_correct_destination)
            {
              ++kitting_shipments_wrong_station;
            }
          }
        }
        // Output header
        if (output_header)
        {
          out << "[kitting]:\"order_id\",\"completion_score\",\"priority\",\"time\",\"is_complete\","
              << "\"shipments_requested\",\"shipments_submitted\",\"shipments_completed\","
              << "\"shipments_wrong_agv\", \"shipments_wrong_station\"\r\n";
        }
        // Output data
        out << "\"" << order_id << "\",";
        out << computeKittingCompletionScore() << ",";
        out << priority << ",";
        out << time_taken << ",";
        out << (isKittingComplete() ? "1" : "0") << ",";
        out << kitting_shipments_requested << ",";
        out << kitting_shipments_submitted << ",";
        out << kitting_shipments_completed << ",";
        out << kitting_shipments_wrong_agv << ",";
        out << kitting_shipments_wrong_station << "\r\n";
      }
      else
      {
        out << "[kitting]: No kitting shipment was requested in this order"
            << "\r\n";
      }
      return out.str();
    }

  public:
    std::string csv_assembly(bool output_header = true) const
    {
      std::stringstream out;
      // Gather data
      size_t assembly_shipments_requested = assembly_shipment_scores.size();
      size_t assembly_shipments_submitted = 0;
      size_t assembly_shipments_completed = 0;
      size_t assembly_shipments_wrong_station = 0;
      if (assembly_shipments_requested > 0)
      {
        for (const auto shipment_tuple : assembly_shipment_scores)
        {
          if (shipment_tuple.second.isShipmentSubmitted)
          {
            ++assembly_shipments_submitted;
            if (shipment_tuple.second.isShipmentComplete)
            {
              ++assembly_shipments_completed;
            }
            if (!shipment_tuple.second.isCorrectStation)
            {
              ++assembly_shipments_wrong_station;
            }
          }
        }
        // Output header
        if (output_header)
        {
          out << "[assembly]: \"order_id\",\"completion_score\",\"priority\",\"time\",\"is_complete\","
              << "\"shipments_requested\",\"shipments_submitted\",\"shipments_completed\","
              << "\"shipments_wrong_station\"\r\n";
        }
        // Output data
        out << "\"" << order_id << "\",";
        out << computeAssemblyCompletionScore() << ",";
        out << priority << ",";
        out << time_taken << ",";
        out << (isAssemblyComplete() ? "1" : "0") << ",";
        out << assembly_shipments_requested << ",";
        out << assembly_shipments_submitted << ",";
        out << assembly_shipments_completed << ",";
        out << assembly_shipments_wrong_station << "\r\n";
      }
      else
      {
        out << "[assembly]: No assembly shipment was requested in this order"
            << "\r\n";
      }
      return out.str();
    }
    /// \brief Mapping between shipment IDs and scores.
  public:
    std::map<KittingShipmentType_t, KittingShipmentScore> kitting_shipment_scores;
    std::map<AssemblyShipmentType_t, AssemblyShipmentScore> assembly_shipment_scores;
    /// \brief ID of the order being scored.
    OrderID_t order_id;
    /// \brief Time in seconds spend on the order.
    double time_taken = 0.0;
    /// \brief Factor indicating order priority (Allowed values 1 or 3)
    int priority = 1;

    bool has_kitting_task = false;
    bool has_assembly_task = false;

    /// \brief simulation time when order was started
    gazebo::common::Time start_time;

    /// \brief Calculate if the kitting shipment is complete.
    /// \return True if all kitting shipping boxes have been submitted.
    ///   Will return false if there are no shipping boxes in the order.
    bool isKittingComplete() const
    {
      if (this->kitting_shipment_scores.size() == 0)
      {
        return false;
      }

      for (const auto &item : this->kitting_shipment_scores)
      {
        // gzdbg << "isKittingComplete: " << item.first << std::endl;
        if (!item.second.is_kitting_shipment_submitted)
        {
          return false;
        }
      }
      return true;
    };
    /// \brief Calculate if the assembly shipment is complete.
    /// \return True if all assembly shipments have been submitted.
    ///   Will return false if there are no shipping boxes in the order.
    bool isAssemblyComplete() const
    {
      if (this->assembly_shipment_scores.size() == 0)
      {
        return false;
      }
      for (const auto &item : this->assembly_shipment_scores)
      {
        if (!item.second.isShipmentSubmitted)
        {
          return false;
        }
      }
      return true;
    };

    /// \brief Calculate the total score.
    double computeKittingTotal() const
    {
      return computeKittingCompletionScore() * priority;
    };

    double computeAssemblyTotal() const
    {
      return computeAssemblyCompletionScore() * priority;
    };

    /// \brief Get score without priority factor.
    double computeKittingCompletionScore() const
    {
      double total = 0.0;
      for (const auto &item : this->kitting_shipment_scores)
      {
        total += item.second.total();
      }
      return total;
    };

    double computeAssemblyCompletionScore() const
    {
      double total = 0.0;
      for (const auto &item : this->assembly_shipment_scores)
      {
        total += item.second.total();
      }
      return total;
    };
  };
  //end class OrderScore

  /////////////////////////////////////////////////////////////
  /// \brief The score of a competition run.
  /////////////////////////////////////////////////////////////
  class GameScore
  {
    /// \brief Stream insertion operator.
    /// \param[in] _out output stream.
    /// \param[in] _obj GameScore object to output.
    /// \return The output stream
  public:
    friend std::ostream &operator<<(std::ostream &_out,
                                    const GameScore &_obj)
    {

      _out << "[trial_score]" << std::endl;
      _out << "...total trial score: [" << _obj.total()  << "]" << std::endl;
      _out << "...total process time: [" << _obj.total_process_time << "]" << std::endl;
      _out << "...arms collision?: [" << _obj.was_arm_arm_collision << "]" << std::endl;
      _out << "...dropped part penalty: [" << _obj.penalty << "]" << std::endl;
      for (const auto &item : _obj.order_scores_map)
      {
        _out << item.second << std::endl;
      }
      return _out;
    }

  public:
    double total_process_time = 0.0;
    bool was_arm_arm_collision = false;
    int penalty = 0;

    // The score of each of the orders during the game.
    std::map<OrderID_t, OrderScore> order_scores_map;

    /// \brief Calculate the total score.
    double total() const
    {
      if (was_arm_arm_collision)
      {
        return 0;
      }
      double total = 0;

      for (const auto &item : this->order_scores_map)
      {
        total += item.second.computeKittingTotal();
        total += item.second.computeAssemblyTotal();
      }

      if (total >= 0)
        total = total - penalty;

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

  /////////////////////////////////////////////////////////////
  /// \brief Class to store information about each product contained in a shipment.
  /////////////////////////////////////////////////////////////
  class Product
  {
    /// \brief Stream insertion operator.
    /// \param[in] _out output stream.
    /// \param[in] _obj Shipment object to output.
    /// \return The output stream
  public:
    friend std::ostream &operator<<(std::ostream &_out,
                                    const Product &_obj)
    {
      _out << "  <product>" << std::endl;
      _out << "    <type>" << _obj.type << ""
           << "</type>" << std::endl;
      _out << "    <faulty>" << (_obj.isFaulty ? "true" : "false") << "</faulty>" << std::endl;
      _out << "    <pose>" << _obj.pose << ""
           << "</pose>" << std::endl;
      _out << "  </product>" << std::endl;
      return _out;
    }

    /// \brief Product type.
  public:
    std::string type;

    /// \brief Whether or not the product is faulty.
  public:
    bool isFaulty;

    /// \brief Pose in which the product should be placed.
  public:
    ignition::math::Pose3d pose;
  };

  class RobotDisableCondition
  {
    ///@brief Robot to disable
    /// Options are: kitting_robot, assembly_robot
  public:
    std::string robot_type;
    ///@brief Where are parts placed to disable the kitting robot
    /// Options are: agv#, as#
  public:
    std::string location;
    ///@brief Number of parts placed in location to disable the robot
  public:
    int number_of_parts;
  };


  /////////////////////////////////////////////////////////////
  /// \brief Class to store information about a kitting shipment.
  /////////////////////////////////////////////////////////////
  class KittingShipment
  {
    /// \brief Stream insertion operator.
    /// \param[in] _out output stream.
    /// \param[in] _shipment shipment to output.
    /// \return The output stream.
  public:
    friend std::ostream &operator<<(std::ostream &_out,
                                    const KittingShipment &_shipment)
    {
      _out << "<kitting_shipment type='" << _shipment.shipment_type << "' agv='" << _shipment.agv_id << "' station='" << _shipment.assembly_station << "'>";
      for (const auto &obj : _shipment.products)
        _out << std::endl
             << obj;
      _out << "</kitting_shipment>" << std::endl;

      return _out;
    }

    /// \brief The type of the shipment.
  public:
    KittingShipmentType_t shipment_type;

    /// \brief which AGV this shipment should be put into "agv1", "agv2", "agv3", "agv4", or "any"
  public:
    std::string agv_id;

    /// \brief Station the shipment is sent to: AS1, AS2, AS3, AS4, AS5, or AS6
  public:
    std::string assembly_station;

    /// \brief A shipment is composed of multiple products.
  public:
    std::vector<Product> products;
  };

  /////////////////////////////////////////////////////////////
  /// \brief Class to store information about an assembly shipment.
  /////////////////////////////////////////////////////////////
  class AssemblyShipment
  {
    /// \brief Stream insertion operator.
    /// \param[in] _out output stream.
    /// \param[in] _shipment shipment to output.
    /// \return The output stream.
  public:
    friend std::ostream &operator<<(std::ostream &_out,
                                    const AssemblyShipment &_shipment)
    {
      _out << "<assembly_shipment type='" << _shipment.shipmentType << "' station='" << _shipment.assembly_station << "'>";
      for (const auto &obj : _shipment.products)
        _out << std::endl
             << obj;
      _out << "</assembly_shipment>" << std::endl;

      return _out;
    }

    /// \brief The type of the shipment.
  public:
    AssemblyShipmentType_t shipmentType;

    /// \brief Station the shipment is sent to: AS1, AS2, AS3, AS4, AS5, or AS6
  public:
    std::string assembly_station;

    /// \brief A shipment is composed of multiple products.
  public:
    std::vector<Product> products;
  };

  /////////////////////////////////////////////////////////////
  /// \brief Class to store information about each object contained in a kit.
  /////////////////////////////////////////////////////////////
  class KitObject
  {
    /// \brief Stream insertion operator.
    /// \param[in] _out output stream.
    /// \param[in] _obj Kit object to output.
    /// \return The output stream
  public:
    friend std::ostream &operator<<(std::ostream &_out,
                                    const KitObject &_obj)
    {
      _out << "<object>" << std::endl;
      _out << "...type: [" << _obj.type << "]" << std::endl;
      _out << "...faulty: [" << (_obj.isFaulty ? "true" : "false") << "]" << std::endl;
      _out << "...pose: [" << _obj.pose << "]" << std::endl;
      _out << "</object>" << std::endl;
      return _out;
    }

    /// \brief Object type.
  public:
    std::string type;

    /// \brief Whether or not the object is faulty.
  public:
    bool isFaulty;

    /// \brief Pose in which the object should be placed.
  public:
    ignition::math::Pose3d pose;
  };

  /////////////////////////////////////////////////////////////
  /// \brief Class to store information about a kit.
  /////////////////////////////////////////////////////////////
  class Kit
  {
    /// \brief Stream insertion operator.
    /// \param[in] _out output stream.
    /// \param[in] _kit kit to output.
    /// \return The output stream.
  public:
    friend std::ostream &operator<<(std::ostream &_out,
                                    const Kit &_kit)
    {
      _out << "<kit type='" << _kit.kitType << "'>";
      for (const auto &obj : _kit.objects)
        _out << std::endl
             << obj;
      _out << std::endl
           << "</kit>" << std::endl;

      return _out;
    }

    /// \brief The type of the kit.
  public:
    KitType_t kitType;

    /// \brief A kit is composed of multiple objects.
  public:
    std::vector<KitObject> objects;
  };

  

  /////////////////////////////////////////////////////////////
  /// \brief Class to store information about assembly.
  /////////////////////////////////////////////////////////////
  class Assembly
  {
    /// \brief Stream insertion operator.
    /// \param[in] _out output stream.
    /// \param[in] _assembly assembly to output.
    /// \return The output stream.
  public:
    friend std::ostream &operator<<(std::ostream &_out,
                                    const Assembly &_assembly)
    {
      _out << "<assembly type='" << _assembly.assemblyType << "'>";
      for (const auto &obj : _assembly.objects)
        _out << std::endl
             << obj;
      _out << std::endl
           << "</assembly>" << std::endl;

      return _out;
    }

    /// \brief The station where assembly must be performed (AS1, AS2, AS3, AS4, AS5, or AS6)
  public:
    std::string assemblyStation;

    /// \brief The type of the assembly.
  public:
    std::string assemblyType;

    /// \brief Assembly is composed of multiple objects.
  public:
    std::vector<BriefcaseProduct> objects;
  };

  /////////////////////////////////////////////////////////////
  /// \brief Class to store information about an order.
  /////////////////////////////////////////////////////////////
  class Order
  {
    /// \brief Less than operator.
    /// \param[in] _order Other order to compare.
    /// \return True if this < _order.
  public:
    bool operator<(const Order &_order) const
    {
      return this->start_time < _order.start_time;
    }

    /// \brief Stream insertion operator.
    /// \param[in] _out output stream.
    /// \param[in] _order Order to output.
    /// \return The output stream.
  public:
    friend std::ostream &operator<<(std::ostream &_out,
                                    const Order &_order)
    {
      _out << "<order start_time = " << _order.start_time << ">" << std::endl;
      // _out << "...start time: [" << _order.start_time << "]" << std::endl;

      //--if this order consists of a kitting task then display the shipments
      if (_order.has_kitting_task)
      {
        // _out << "...kitting shipments:" << std::endl;
        for (const auto &item : _order.kitting_shipments)
        {
          _out << item << std::endl;
        }
      }
      //--if this order consists of an assembly task then display the tasks information
      if (_order.has_kitting_task)
      {
        // _out << "...assembly shipments:" << std::endl;
        for (const auto &item : _order.assembly_shipments)
        {
          _out << item << std::endl;
        }
      }
      _out << "</order>" << std::endl;

      return _out;
    }

    /// \brief The ID of this order.
  public:
    OrderID_t order_id;
  public:
    int priority;
    /// \brief Simulation time in which the order should be triggered.
  public:
    double start_time;

    /// \brief After how many unwanted products to interrupt the previous order (-1 for never).
  public:
    int interrupt_on_unwanted_products;

    /// \brief After how many wanted products to interrupt the previous order (-1 for never).
  public:
    int interrupt_on_wanted_products;

    /// \brief Which station to send the AGV
    public:
      std::string interrupt_on_station_reached;

    /// \brief Simulation time in seconds permitted for the order to be
    /// completed before cancelling it. Infinite by default.
  public:
    double allowed_time;

    /// \brief An order is composed of multiple shipments of different types.
  public:
    std::vector<KittingShipment> kitting_shipments;

    /// \brief An order is composed of multiple assembly tasks of different types.
  public:
    std::vector<AssemblyShipment> assembly_shipments;

    /// \brief Simulation time in seconds spent on this order.
  public:
    double time_taken;

  public:
    bool has_kitting_task;

  public:
    bool has_assembly_task;
  //--status of the kitting robot
  public:
    ariac::RobotDisableCondition robot_disable_condition;
  //--halth status of the kitting robot
  public:
    int kitting_robot_health;
  //--health status of the assembly robot
  public:
    int assembly_robot_health;
  };
  //-- end class Order

} // namespace ariac
#endif
