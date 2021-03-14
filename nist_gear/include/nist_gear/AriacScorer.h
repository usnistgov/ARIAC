/*
 * Copyright (C) 2012-2016 Open Source Robotics Foundation
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
/*
 * Desc: ARIAC scorer.
 * Author: Deanna Hood
 * Author: Zeid Kootbally
 */
#ifndef NIST_GEAR_ARIACSCORER_H
#define NIST_GEAR_ARIACSCORER_H

#include <map>
#include <string>
#include <vector>
#include <ros/ros.h>
#include <utility>
#include <algorithm>

#include <nist_gear/ARIAC.hh>
#include <nist_gear/DetectedKittingShipment.h>
#include <nist_gear/DetectedAssemblyShipment.h>
#include <nist_gear/Order.h>
#include <nist_gear/SubmitShipment.h>
#include "nist_gear/VacuumGripperState.h"


// Ariac scorer needs to know when an order starts
// -Order
//   -kitting
//      shipments for kitting
//   -assembly
//     shipments for assembly
//   -start time of the order
//   -end time of the order
// Tell scorer when an order starts
// tell scorer when a shipment is submitted

/// \brief A scorer for the ARIAC game.
/**
 * A scorer for the ARIAC game.
 *
 * The scorer needs to know when an order starts. The following information is retrieved from the class @ref Order:
 * 
 * <ul>
 *  <li>Order</li>
 *   <ul>
 *    <li>kitting</li>
 *    <li>assembly</li>
 *   </ul>
 *  <li>start time of the order</li>
 *  <li>end time of the order</li>
 * </ul> 
 */
class AriacScorer
{
protected:
  struct OrderInfo
  {
    gazebo::common::Time start_time;
    int priority;
    nist_gear::Order::ConstPtr order;
  };

  struct OrderUpdateInfo
  {
    ariac::OrderID_t original_order_id;
    gazebo::common::Time update_time;
    nist_gear::Order::ConstPtr order;
  };

  struct KittingShipmentInfo
  {
    gazebo::common::Time submit_time;
    ariac::KittingShipmentType_t type;
    nist_gear::DetectedKittingShipment::ConstPtr shipment;
    std::string station;
  };

  struct AssemblyShipmentInfo
  {
    gazebo::common::Time submit_time;
    ariac::AssemblyShipmentType_t type;
    nist_gear::DetectedAssemblyShipment::ConstPtr shipment;
    std::string station;
  };

  /// \brief Constructor.
public:
  AriacScorer();

  /// \brief Destructor.
public:
  virtual ~AriacScorer();

  /// \brief Tell scorer a new order was published
  /// \param[in] time when in sim time the order was published
  /// \param[in] order the order that was sent to teams
public:
  void NotifyOrderStarted(gazebo::common::Time time, const nist_gear::Order &order, int order_priority);

  /// \brief Tell scorer an existing order was updated
  /// \param[in] time when in sim time the update to the order was published
  /// \param[in] order the order that was sent to teams
public:
  void NotifyOrderUpdated(gazebo::common::Time time, ariac::OrderID_t old_order, const nist_gear::Order &order);

  /// \brief Tell scorer a shipment was recieved
  /// Allows ROSAriacTaskManagerPlugin.cc to pass data to AriacScorer.cpp
public:
  void NotifyKittingShipmentReceived(gazebo::common::Time time,
                                     ariac::KittingShipmentType_t type,
                                     const nist_gear::DetectedKittingShipment &actualShipment,
                                     std::string actual_station);

public:
  void NotifyAssemblyShipmentReceived(gazebo::common::Time time,
                                      ariac::AssemblyShipmentType_t type,
                                      const nist_gear::DetectedAssemblyShipment &actualShipment,
                                      std::string actual_station);

  /// \brief Tell scorer that the two arms collided with each other
  /// \param[in] time when the collision occurred
public:
  void NotifyArmArmCollision(gazebo::common::Time time);

  /// \brief Get the current score.
  /// \return The score for the game.
public:
  ariac::GameScore GetGameScore(int penalty);

  /// \brief Score a single shipment
  /// \return The score for the game.
public:
  ariac::KittingShipmentScore GetKittingShipmentScore(
      gazebo::common::Time submit_time,
      const nist_gear::KittingShipment &desired,
      const nist_gear::DetectedKittingShipment &actual, std::string station);

public:
  ariac::AssemblyShipmentScore GetAssemblyShipmentScore(
      gazebo::common::Time submit_time,
      const nist_gear::AssemblyShipment &desired,
      const nist_gear::DetectedAssemblyShipment &actual, std::string station);

  /// \brief Mutex for protecting this class
protected:
  mutable boost::mutex mutex;

  /// \brief Collection of orders that have been announced
protected:
  std::map<ariac::OrderID_t, struct OrderInfo> orders;

  /// \brief Collection of updates to orders
protected:
  std::vector<struct OrderUpdateInfo> order_updates;

  /// \brief Collection of shipments that have been received
protected:
  std::vector<struct KittingShipmentInfo> received_kitting_shipments_vec;
  std::vector<struct AssemblyShipmentInfo> received_assembly_shipments_vec;
  /// \brief True if the arms collided with each other
  bool arm_arm_collision = false;

  
};

#endif  // NIST_GEAR_ARIACSCORER_H
