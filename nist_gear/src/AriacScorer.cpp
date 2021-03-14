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

#include <math.h>
#include <string>


#include <gazebo/common/Console.hh>
#include <ignition/math/Pose3.hh>
#include <ignition/math/Vector3.hh>
#include <ignition/math/Quaternion.hh>

#include "nist_gear/AriacScorer.h"

/////////////////////////////////////////////////
AriacScorer::AriacScorer()
{
}

/////////////////////////////////////////////////
AriacScorer::~AriacScorer()
{
}

/////////////////////////////////////////////////
void AriacScorer::NotifyOrderStarted(gazebo::common::Time time, const nist_gear::Order &order, int order_priority)
{
  AriacScorer::OrderInfo orderInfo;
  orderInfo.start_time = time;
  orderInfo.order = nist_gear::Order::ConstPtr(new nist_gear::Order(order));

  boost::mutex::scoped_lock mutexLock(this->mutex);

  orderInfo.priority = order_priority;
  // if (!this->orders.empty())
  // {
  //   // orders after the first are implicitly higher priority
  //   orderInfo.priority = 3;
  // }

  auto it = this->orders.find(order.order_id);
  if (it != this->orders.end())
  {
    gzerr << "[ARIAC ERROR] Order with duplicate ID '" << order.order_id << "'; overwriting\n";
  }

  this->orders[order.order_id] = orderInfo;
}

/////////////////////////////////////////////////
void AriacScorer::NotifyOrderUpdated(
  gazebo::common::Time time,
  ariac::OrderID_t old_order,
  const nist_gear::Order &order)
{
  AriacScorer::OrderUpdateInfo updateInfo;
  updateInfo.update_time = time;
  updateInfo.original_order_id = old_order;
  updateInfo.order = nist_gear::Order::ConstPtr(new nist_gear::Order(order));

  boost::mutex::scoped_lock mutexLock(this->mutex);
  auto it = this->orders.find(order.order_id);
  if (it != this->orders.end())
  {
    gzerr << "[ARIAC ERROR] Asked to update nonexistant order '" << order.order_id << "'; ignoring\n";
    return;
  }

  this->order_updates.push_back(updateInfo);
}


void AriacScorer::NotifyKittingShipmentReceived(gazebo::common::Time time,
                                                ariac::KittingShipmentType_t type,
                                                const nist_gear::DetectedKittingShipment &shipment,
                                                std::string actual_station)
{
  gzdbg << "NotifyKittingShipmentReceived\n";
  // information about the shipment that was actually submitted by the participant.
  AriacScorer::KittingShipmentInfo submitted_shipment_info;
  submitted_shipment_info.submit_time = time;
  submitted_shipment_info.type = type;  // type of the shipment, e.g., order_0_shipment_0
  submitted_shipment_info.station = actual_station;

  submitted_shipment_info.shipment =
  nist_gear::DetectedKittingShipment::ConstPtr(new nist_gear::DetectedKittingShipment(shipment));

  boost::mutex::scoped_lock mutexLock(this->mutex);
  this->received_kitting_shipments_vec.push_back(submitted_shipment_info);
}

void AriacScorer::NotifyAssemblyShipmentReceived(gazebo::common::Time time,
                                                 ariac::AssemblyShipmentType_t type,
                                                 const nist_gear::DetectedAssemblyShipment &shipment,
                                                 std::string actual_station)
{
  gzdbg << "NotifyAssemblyShipmentReceived\n";
  // information about the shipment that was actually submitted by the participant.
  AriacScorer::AssemblyShipmentInfo submitted_shipment_info;
  submitted_shipment_info.submit_time = time;
  submitted_shipment_info.type = type;  // type of the shipment, e.g., order_0_shipment_0
  submitted_shipment_info.station = actual_station;
  submitted_shipment_info.shipment =
  nist_gear::DetectedAssemblyShipment::ConstPtr(new nist_gear::DetectedAssemblyShipment(shipment));

  boost::mutex::scoped_lock mutexLock(this->mutex);
  this->received_assembly_shipments_vec.push_back(submitted_shipment_info);
}

/////////////////////////////////////////////////
void AriacScorer::NotifyArmArmCollision(gazebo::common::Time /*time*/)
{
  boost::mutex::scoped_lock mutexLock(this->mutex);
  this->arm_arm_collision = true;
}

/////////////////////////////////////////////////
ariac::GameScore AriacScorer::GetGameScore(int penalty)
{
  // gzdbg << "GetGameScore\n";
  boost::mutex::scoped_lock mutexLock(this->mutex);

  ariac::GameScore game_score;
  game_score.penalty = penalty;

  // arm/arm collision results in zero score, but keep going for logging
  game_score.was_arm_arm_collision = this->arm_arm_collision;

  // Calculate the current score based on received orders and shipments
  // For each order, how many shipments was it supposed to have?
  for (auto &opair : this->orders)  // this->orders has order info from ariac.world
  {
    auto order_id = opair.first;
    auto order_info = opair.second;

    gazebo::common::Time start_time = order_info.start_time;
    int priority = order_info.priority;
    nist_gear::Order::ConstPtr order = order_info.order;

    // If order was updated, score based on the lastest version of it
    for (auto &update_info : this->order_updates)
    {
      if (update_info.original_order_id == order_id)
      {
        // gzdbg << "+++++ order updated"  << std::endl;
        order = update_info.order;
        start_time = update_info.update_time;
      }
    }

    // Create score class for orderGetShipmentScore
    ariac::OrderScore order_score;
    order_score.order_id = order_id;  // e.g., order_0

    order_score.priority = priority;  // e.g., 1
    auto oit = game_score.order_scores_map.find(order_id);
    if (oit != game_score.order_scores_map.end())
    {
      gzerr << "[ARIAC ERROR] Multiple orders of duplicate ids:" << order_score.order_id << "\n";
    }

    std::vector<std::string> claimed_shipments;

    ////////////////////////////////////
    //  Take care of assembly shipments
    ////////////////////////////////////
    if (!this->received_kitting_shipments_vec.empty())
    {
      // gzdbg << "received_kitting_shipments_vec NOT EMPTY\n";
      // instantiate shipment_score for expected shipments (from ariac.world)
      for (const auto &expected_shipment : order->kitting_shipments)
      {
        ariac::KittingShipmentScore shipment_score;

        shipment_score.kittingShipmentType =
        expected_shipment.shipment_type;  // e.g., order_0_kitting_shipment_0
        // ROS_WARN_STREAM("shipment_score.kittingShipmentType " << shipment_score.kittingShipmentType);
        auto it = order_score.kitting_shipment_scores.find(expected_shipment.shipment_type);
        if (it != order_score.kitting_shipment_scores.end())
        {
          gzerr << "[ARIAC ERROR] Order contained duplicate shipment types:" << expected_shipment.shipment_type << "\n";
        }
        order_score.kitting_shipment_scores[expected_shipment.shipment_type] = shipment_score;
      }

      // Find actual shipments that belong to this order
      for (const auto &desired_shipment : order->kitting_shipments)
      {
        for (const auto &received_shipment_info : this->received_kitting_shipments_vec)
        {
          // ROS_WARN_STREAM("desired_shipment.shipment_type " << desired_shipment.shipment_type);
          // ROS_WARN_STREAM("received_shipment_info.type " << received_shipment_info.type);
          if (desired_shipment.shipment_type == received_shipment_info.type)
          {
            if (received_shipment_info.submit_time < start_time)
            {
              // Maybe order was updated, this shipment was submitted too early
              continue;
            }
            // else{
            //   ROS_WARN_STREAM("ALL GOOD");
            // }
            // If the same shipment was submitted twice, only count the first one
            bool is_claimed = false;
            for (const auto &type : claimed_shipments)
            {
              if (type == desired_shipment.shipment_type)
              {
                is_claimed = true;
                break;
              }
            }
            if (is_claimed)
            {
              continue;
            }

            claimed_shipments.push_back(desired_shipment.shipment_type);

            order_score.kitting_shipment_scores[desired_shipment.shipment_type] =
                this->GetKittingShipmentScore(received_shipment_info.submit_time,
                                              desired_shipment,
                                              *(received_shipment_info.shipment),
                                              received_shipment_info.station);
          }
        }
      }

      // Figure out the time taken to complete an order
      if (order_score.isKittingComplete())
      {
        // The latest submitted shipment time is the order completion time
        gazebo::common::Time end = start_time;
        for (auto &sspair : order_score.kitting_shipment_scores)
        {
          if (sspair.second.submit_time > end)
          {
            end = sspair.second.submit_time;
          }
        }
        order_score.time_taken = (end - start_time).Double();
      }
    }  // end if (!received_kitting_shipments_vec.empty())

    ////////////////////////////////////
    // Take care of assembly shipments
    ////////////////////////////////////
    if (!this->received_assembly_shipments_vec.empty())
    {
      // instantiate shipment_score for expected shipments (from ariac.world)
      for (const auto &expected_assembly_shipment : order->assembly_shipments)
      {
        ariac::AssemblyShipmentScore shipment_score;

        shipment_score.assemblyShipmentType =
        expected_assembly_shipment.shipment_type;  // e.g., order_0_assembly_shipment_0
        auto it = order_score.assembly_shipment_scores.find(expected_assembly_shipment.shipment_type);
        if (it != order_score.assembly_shipment_scores.end())
        {
          gzerr << "[ARIAC ERROR] Order contained duplicate shipment types:" <<
          expected_assembly_shipment.shipment_type << "\n";
        }
        order_score.assembly_shipment_scores[expected_assembly_shipment.shipment_type] = shipment_score;
      }

      // Find actual shipments that belong to this order
      for (const auto &desired_assembly_shipment : order->assembly_shipments)
      {
        for (const auto &received_shipment_info : this->received_assembly_shipments_vec)
        {
          if (desired_assembly_shipment.shipment_type == received_shipment_info.type)
          {
            if (received_shipment_info.submit_time < start_time)
            {
              // Maybe order was updated, this shipment was submitted too early
              continue;
            }
            // If the same shipment was submitted twice, only count the first one
            bool is_claimed = false;
            for (const auto &type : claimed_shipments)
            {
              if (type == desired_assembly_shipment.shipment_type)
              {
                is_claimed = true;
                break;
              }
            }
            if (is_claimed)
            {
              continue;
            }

            claimed_shipments.push_back(desired_assembly_shipment.shipment_type);

            order_score.assembly_shipment_scores[desired_assembly_shipment.shipment_type] =
                this->GetAssemblyShipmentScore(received_shipment_info.submit_time,
                                               desired_assembly_shipment,
                                               *(received_shipment_info.shipment),
                                               received_shipment_info.station);
          }
        }
      }
    }
    game_score.order_scores_map[order_id] = order_score;
  }
  return game_score;
}

/**
 * @brief Compute the score of a kitting shipment
 */
ariac::KittingShipmentScore AriacScorer::GetKittingShipmentScore(
    gazebo::common::Time submit_time,
    const nist_gear::KittingShipment &desired_shipment,
    const nist_gear::DetectedKittingShipment &actual_shipment, std::string station)
{
  ariac::KittingShipmentScore scorer;
  scorer.is_kitting_shipment_submitted = true;
  scorer.submit_time = submit_time;

  bool has_faulty_product = false;
  bool is_missing_products = false;
  bool has_unwanted_product = false;
  scorer.productOnlyTypePresence = 0;
  scorer.productTypeAndColorPresence = 0;
  scorer.allProductsBonus = 0;
  scorer.productPose = 0;
  scorer.is_kitting_correct_agv = false;
  scorer.is_kitting_correct_destination = false;

  // make sure the kit was built on the correct agv
  if ("any" == desired_shipment.agv_id)
  {
    scorer.is_kitting_correct_agv = true;
  }
  else if ("agv1" == desired_shipment.agv_id)
  {
    scorer.is_kitting_correct_agv = "agv1::kit_tray_1::kit_tray_1::tray" == actual_shipment.destination_id;
  }
  else if ("agv2" == desired_shipment.agv_id)
  {
    scorer.is_kitting_correct_agv = "agv2::kit_tray_2::kit_tray_2::tray" == actual_shipment.destination_id;
  }
  else if ("agv3" == desired_shipment.agv_id)
  {
    scorer.is_kitting_correct_agv = "agv3::kit_tray_3::kit_tray_3::tray" == actual_shipment.destination_id;
  }
  else if ("agv4" == desired_shipment.agv_id)
  {
    scorer.is_kitting_correct_agv = "agv4::kit_tray_4::kit_tray_4::tray" == actual_shipment.destination_id;
  }
  else
  {
    gzerr << "[ARIAC ERROR] desired shipment agv invalid:" << desired_shipment.agv_id << "\n";
  }

  // ROS_WARN_STREAM("desired station: " << desired_shipment.station_id);
  // ROS_WARN_STREAM("actual station: " << station);

  // make sure the AGV was sent to the correct station
  if ("as1" == desired_shipment.station_id)
  {
    scorer.is_kitting_correct_destination = "as1" == station;
  }
  else if ("as2" == desired_shipment.station_id)
  {
    scorer.is_kitting_correct_destination = "as2" == station;
  }
  else if ("as3" == desired_shipment.station_id)
  {
    scorer.is_kitting_correct_destination = "as3" == station;
  }
  else if ("as4" == desired_shipment.station_id)
  {
    scorer.is_kitting_correct_destination = "as4" == station;
  }
  else
  {
    gzerr << "[ARIAC ERROR] desired shipment station invalid:" << desired_shipment.station_id << "\n";
  }
  // Separate faulty and non-faulty products
  std::vector<nist_gear::DetectedProduct> detected_non_faulty_products;
  for (const auto &actual_product : actual_shipment.products)
  {
    if (actual_product.is_faulty)
    {
      has_faulty_product = true;
    }
    else
    {
      detected_non_faulty_products.push_back(actual_product);
    }
  }

  // +1 point for each product of correct type
  // make a copy of non faulty products
  // we will work with this vector for actual products in the trays
  std::vector<nist_gear::DetectedProduct> tmp_non_faulty_products;
  tmp_non_faulty_products = detected_non_faulty_products;
  // check product type is correct even if color is wrong
  for (size_t d = 0; d < desired_shipment.products.size(); ++d)
  {
    // bool found_exact_product_type = false;

    auto desired_product = desired_shipment.products[d].type;
    auto desired_product_name = desired_product;
    // ROS_WARN_STREAM("desired_product_name: " << desired_product_name);

    // keep only the product type and remove the product colot
    //-- e.g., assembly_battery_blue becomes assembly_battery
    auto desired_product_type = desired_product.erase(desired_product.rfind('_'));
    // ROS_WARN_STREAM("desired_product_type: " << desired_product_type);

    for (size_t a = 0; a < tmp_non_faulty_products.size(); ++a)
    {
      auto actual_product_name = tmp_non_faulty_products[a].type;

      // In the case the actual_product_name has the following format: agv2::tray_2::assembly_battery_blue
      auto pos = actual_product_name.rfind(':');
      if (pos != std::string::npos)
      {
        actual_product_name.erase(0, pos + 1);
      }
      // now, let's grab only the type of the product and discard the color
      auto actual_product_type = actual_product_name.erase(actual_product_name.rfind('_'));

      if (desired_product_type.compare(actual_product_type) == 0)
      {
        // found_exact_product_type = true;

        // give 1pt for correct type
        scorer.productOnlyTypePresence++;
        // we are done with this part
        // @todo: Do not remove this part yet
        tmp_non_faulty_products.erase(tmp_non_faulty_products.begin() + a);
        break;
      }
    }
  }

  // +1 pt if color is correct (part type has to be correct)
  // e.g., if 'assembly_battery_red' is required but 'assembly_pump_red' is provided
  // then no point is awarded even though the wrong product is red

  // Map of product type to indexes in desired products (first) and indexes in non faulty actual products (second)
  std::map<std::string, std::pair<std::vector<size_t>, std::vector<size_t>>> product_type_map;

  for (size_t d = 0; d < desired_shipment.products.size(); ++d)
  {
    const auto &desired_product = desired_shipment.products[d];
    auto &mapping = product_type_map[desired_product.type];
    mapping.first.push_back(d);
  }

  for (size_t a = 0; a < detected_non_faulty_products.size(); ++a)
  {
    auto &actual_product = detected_non_faulty_products[a];
    // ROS_WARN_STREAM("actual_product " << actual_product.type);
    // if the name contains :: then clean the name

    auto pos = actual_product.type.rfind(':');
    if (pos != std::string::npos)
    {
      actual_product.type.erase(0, pos + 1);
    }
    if (0u == product_type_map.count(actual_product.type))
    {
      // since desired products were put into the type map first, this product must be unwanted
      has_unwanted_product = true;
      continue;
    }
    auto &mapping = product_type_map.at(actual_product.type);
    mapping.second.push_back(a);
  }

  for (const auto &type_pair : product_type_map)
  {
    const std::vector<size_t> &desired_indexes = type_pair.second.first;
    const std::vector<size_t> &actual_indexes = type_pair.second.second;
    auto product_name = type_pair.first;
    // ROS_WARN_STREAM("desired_indexes: "<< desired_indexes.size());
    // ROS_WARN_STREAM("actual_indexes: "<< actual_indexes.size());
    // ROS_WARN_STREAM("product_name: "<< product_name);

    if (desired_indexes.size() > actual_indexes.size())
    {
      is_missing_products = true;
    }
    else if (desired_indexes.size() < actual_indexes.size())
    {
      has_unwanted_product = true;
    }

    // no point in trying to score this type if there are none delivered
    if (actual_indexes.empty())
    {
      continue;
    }

    scorer.productTypeAndColorPresence += std::min(desired_indexes.size(), actual_indexes.size());

    double contributing_pose_score = 0;
    size_t num_indices = std::max(desired_indexes.size(), actual_indexes.size());
    // ROS_WARN_STREAM("num_indices: "<< num_indices);
    std::vector<size_t> permutation(num_indices);
    for (size_t i = 0; i < num_indices; ++i)
    {
      permutation[i] = i;
    }
    // Now iterate through all permutations of actual matched with desired to find the highest pose score
    do
    {
      double permutation_pose_score = 0;
      for (size_t d = 0; d < desired_indexes.size(); ++d)
      {
        const size_t actual_index_index = permutation[d];
        if (actual_index_index >= actual_indexes.size())
        {
          // There were fewer actual products than the order called for
          continue;
        }
        const auto &desired_product = desired_shipment.products[desired_indexes[d]];
        const auto &actual_product = detected_non_faulty_products[actual_indexes[actual_index_index]];

        // Add points for each product in the correct pose
        const double translation_target = 0.03;  // 3 cm
        const double orientation_target = 0.1;  // 0.1 rad
        // get translation distance
        ignition::math::Vector3d posnDiff(
            desired_product.pose.position.x - actual_product.pose.position.x,
            desired_product.pose.position.y - actual_product.pose.position.y,
            0);
        // ROS_WARN_STREAM("desired_product.pose: " <<
        // desired_product.pose.position.x << ", " << desired_product.pose.position.y);
        // ROS_WARN_STREAM("actual_product.pose: " <<
        // actual_product.pose.position.x << ", " << actual_product.pose.position.y);

        const double distance = posnDiff.Length();

        if (distance > translation_target)
        {
          // Skipping product because translation error is too big
          continue;
        }

        ignition::math::Quaterniond orderOrientation(
            desired_product.pose.orientation.w,
            desired_product.pose.orientation.x,
            desired_product.pose.orientation.y,
            desired_product.pose.orientation.z);

        ignition::math::Quaterniond objOrientation(
            actual_product.pose.orientation.w,
            actual_product.pose.orientation.x,
            actual_product.pose.orientation.y,
            actual_product.pose.orientation.z);

        // Filter products that aren't in the appropriate orientation (loosely).
        // If the quaternions represent the same orientation, q1 = +-q2 => q1.dot(q2) = +-1
        const double orientationDiff = objOrientation.Dot(orderOrientation);
        // TODO(zeid): this value can probably be derived using relationships between
        // euler angles and quaternions.
        const double quaternionDiffThresh = 0.05;
        if (std::abs(orientationDiff) < (1.0 - quaternionDiffThresh))
        {
          // Skipping product because it is not in the correct orientation (roughly)
          continue;
        }

        // Filter the yaw based on a threshold set in radians (more user-friendly).
        // Account for wrapping in angles. E.g. -pi compared with pi should "pass".
        double angleDiff = objOrientation.Yaw() - orderOrientation.Yaw();
        if ((std::abs(angleDiff) < orientation_target) ||
        (std::abs(std::abs(angleDiff) - 2 * M_PI) <= orientation_target))
        {
          permutation_pose_score += 1.0;
        }
      }
      if (permutation_pose_score > contributing_pose_score)
      {
        contributing_pose_score = permutation_pose_score;
      }
    }
    while (std::next_permutation(permutation.begin(), permutation.end()));

    // Add the pose score contributed by the highest scoring permutation
    scorer.productPose += contributing_pose_score;
  }

  if (!is_missing_products)
  {
    scorer.is_kitting_shipment_complete = true;
  }
  if (!has_faulty_product && !has_unwanted_product && !is_missing_products)
  {
    // allProductsBonus is applied if all products have:
    // -the correct pose
    // -the correct color
    // -the correct type
    // -the product is not faulty
    // correct type is true AND correct color is true AND correct pose is true
    if (scorer.productPose == desired_shipment.products.size())
    {
      if (scorer.productTypeAndColorPresence == desired_shipment.products.size())
      {
        if (scorer.productOnlyTypePresence == desired_shipment.products.size())
        {
          scorer.allProductsBonus = desired_shipment.products.size();
        }
      }
    }
  }

  return scorer;
}

/////////////////////////////////////////////////////////////////////////
ariac::AssemblyShipmentScore AriacScorer::GetAssemblyShipmentScore(
    gazebo::common::Time submit_time,
    const nist_gear::AssemblyShipment &desired_shipment,
    const nist_gear::DetectedAssemblyShipment &actual_shipment, std::string station)
{
  ariac::AssemblyShipmentScore scorer;

  scorer.assemblyStation = station;   // assembly station this shipment was sent from
  scorer.submit_time = submit_time;   // time the shipment was submitted
  scorer.isShipmentSubmitted = true;  // true since we are in this function
  scorer.allProductsBonus = 0;
  scorer.isCorrectStation = false;
  scorer.numberOfProductsInShipment = actual_shipment.products.size();

  std::map<std::string, ariac::BriefcaseProduct> mapOfBriefcaseProducts;

  // was the shipment submitted from the desired assembly station?
  if ("as1" == desired_shipment.station_id)
  {
    scorer.isCorrectStation = "as1" == station;
  }
  else if ("as2" == desired_shipment.station_id)
  {
    scorer.isCorrectStation = "as2" == station;
  }
  else if ("as3" == desired_shipment.station_id)
  {
    scorer.isCorrectStation = "as3" == station;
  }
  else if ("as4" == desired_shipment.station_id)
  {
    scorer.isCorrectStation = "as4" == station;
  }
  else
  {
    gzerr << "[ARIAC ERROR] desired shipment station invalid:" << desired_shipment.station_id << "\n";
  }

  // check for faulty products and set hasFaultyProduct=true if faulty products exist
  // store non-faulty products in a vector
  std::vector<nist_gear::DetectedProduct> detected_non_faulty_products;

  for (const auto &actual_product : actual_shipment.products)
  {
    if (actual_product.is_faulty)
    {
      scorer.hasFaultyProduct = true;
    }
    else
    {
      detected_non_faulty_products.push_back(actual_product);
    }
  }

  // make a copy of non-faulty products
  std::vector<nist_gear::DetectedProduct> tmp_non_faulty_products;
  tmp_non_faulty_products = detected_non_faulty_products;

  // compare each desired product with actual product
  for (size_t d = 0; d < desired_shipment.products.size(); ++d)
  {
    auto desired_product = desired_shipment.products[d].type;
    auto desired_product_name = desired_product;

    // filter out the product color from the part name to get the product type
    // e.g., assembly_battery_blue becomes assembly_battery
    auto desired_product_type = desired_product.erase(desired_product.rfind('_'));
    // get the product color
    //-- 1. Find the last occurrence of _
    std::size_t found = desired_product_name.find_last_of("_");
    //-- 2. Grab the substring after the last occurrence of _
    auto desired_product_color = desired_product_name.substr(found + 1);

    for (size_t a = 0; a < tmp_non_faulty_products.size(); ++a)
    {
      ariac::BriefcaseProduct briefcaseProduct;
      auto actual_product = tmp_non_faulty_products[a].type;
      auto actual_product_name = actual_product;
      // in the case the actual_product_name has the following format: agv2::tray_2::assembly_battery_blue
      auto pos = actual_product.rfind(':');
      if (pos != std::string::npos)
      {
        actual_product_name.erase(0, pos + 1);
      }

      auto actual_product_color = actual_product_name.substr(found + 1);
      // filter out the product color from the part name
      // e.g., assembly_battery_blue becomes assembly_battery
      auto actual_product_type = actual_product_name.erase(actual_product_name.rfind('_'));

      briefcaseProduct.productName = actual_product_name;
      briefcaseProduct.productType = actual_product_type;
      // compare the colors
      if (desired_product_color.compare(actual_product_color) == 0)
      {
        briefcaseProduct.isProductCorrectColor = true;
        scorer.numberOfProductsWithCorrectColor++;
      }
      // compare the types
      if (desired_product_type.compare(actual_product_type) == 0)
      {
        briefcaseProduct.isProductCorrectType = true;
        scorer.numberOfProductsWithCorrectType++;
        mapOfBriefcaseProducts.insert(
          std::pair<std::string, ariac::BriefcaseProduct>(actual_product, briefcaseProduct));
        tmp_non_faulty_products.erase(tmp_non_faulty_products.begin() + a);
        break;
      }
    }
  }

  // Map of product type to indexes in desired products (first) and indexes in non faulty actual products (second)
  std::map<std::string, std::pair<std::vector<size_t>, std::vector<size_t>>> product_type_map;

  for (size_t d = 0; d < desired_shipment.products.size(); ++d)
  {
    const auto &desired_product = desired_shipment.products[d];
    auto &mapping = product_type_map[desired_product.type];
    mapping.first.push_back(d);
  }

  for (size_t a = 0; a < detected_non_faulty_products.size(); ++a)
  {
    auto &actual_product = detected_non_faulty_products[a];

    if (0u == product_type_map.count(actual_product.type))
    {
      // since desired products were put into the type map first, this product must be unwanted
      scorer.hasUnwantedProduct = true;
      continue;
    }
    auto &mapping = product_type_map.at(actual_product.type);
    mapping.second.push_back(a);
  }

  for (const auto &type_pair : product_type_map)
  {
    const std::vector<size_t> &desired_indexes = type_pair.second.first;
    const std::vector<size_t> &actual_indexes = type_pair.second.second;
    auto desired_product_name = type_pair.first;
    // ROS_WARN_STREAM("desired_indexes: "<< desired_indexes.size());
    // ROS_WARN_STREAM("actual_indexes: "<< actual_indexes.size());

    // products are missing
    if (desired_indexes.size() > actual_indexes.size())
    {
      scorer.hasMissingProduct = true;
    }
    // more products than needed
    else if (desired_indexes.size() < actual_indexes.size())
    {
      scorer.hasUnwantedProduct = true;
    }

    // no point in trying to score this type if there are none delivered
    if (actual_indexes.empty())
    {
      continue;
    }

    double contributing_pose_score = 0;
    size_t num_indices = std::max(desired_indexes.size(), actual_indexes.size());

    std::vector<size_t> permutation(num_indices);
    for (size_t i = 0; i < num_indices; ++i)
    {
      permutation[i] = i;
    }

    // Now iterate through all permutations of actual matched with desired to find the highest pose score
    ////////////////////////////////////
    // Check pose of desired vs. actual
    ////////////////////////////////////
    std::string briefcase_actual_product{};
    do
    {
      double permutation_pose_score = 0;
      for (size_t d = 0; d < desired_indexes.size(); ++d)
      {
        const size_t actual_index_index = permutation[d];
        if (actual_index_index >= actual_indexes.size())
        {
          // There were fewer actual products than the order called for
          continue;
        }
        const auto &desired_product = desired_shipment.products[desired_indexes[d]];
        const auto &actual_product = detected_non_faulty_products[actual_indexes[actual_index_index]];
        briefcase_actual_product = actual_product.type;
        // ROS_WARN_STREAM("desired_product: "<< desired_product.type);
        // ROS_WARN_STREAM("actual_product: "<< actual_product.type);

        // Add points for each product in the correct pose
        const double translation_target = 0.02;  // 2 cm
        const double orientation_target = 0.1;   // 0.1 rad
        // get translation distance
        ignition::math::Vector3d posnDiff(
            desired_product.pose.position.x - actual_product.pose.position.x,
            desired_product.pose.position.y - actual_product.pose.position.y,
            0);
        // ROS_WARN_STREAM("desired_product.pose: " <<
        // desired_product.pose.position.x << ", " << desired_product.pose.position.y);
        // ROS_WARN_STREAM("actual_product.pose: " <<
        // actual_product.pose.position.x << ", " << actual_product.pose.position.y);

        const double distance = posnDiff.Length();

        if (distance > translation_target)
        {
          // Skipping product because translation error is too big
          continue;
        }

        ignition::math::Quaterniond orderOrientation(
            desired_product.pose.orientation.w,
            desired_product.pose.orientation.x,
            desired_product.pose.orientation.y,
            desired_product.pose.orientation.z);

        ignition::math::Quaterniond objOrientation(
            actual_product.pose.orientation.w,
            actual_product.pose.orientation.x,
            actual_product.pose.orientation.y,
            actual_product.pose.orientation.z);

        // Filter products that aren't in the appropriate orientation (loosely).
        // If the quaternions represent the same orientation, q1 = +-q2 => q1.dot(q2) = +-1
        const double orientationDiff = objOrientation.Dot(orderOrientation);
        // TODO(zeid): this value can probably be derived using relationships between
        // euler angles and quaternions.
        const double quaternionDiffThresh = 0.05;
        if (std::abs(orientationDiff) < (1.0 - quaternionDiffThresh))
        {
          // Skipping product because it is not in the correct orientation (roughly)
          continue;
        }

        // Filter the yaw based on a threshold set in radians (more user-friendly).
        // Account for wrapping in angles. E.g. -pi compared with pi should "pass".
        double angleDiff = objOrientation.Yaw() - orderOrientation.Yaw();
        if ((std::abs(angleDiff) < orientation_target) ||
        (std::abs(std::abs(angleDiff) - 2 * M_PI) <= orientation_target))
        {
          permutation_pose_score += 1.0;
        }
      }

      if (permutation_pose_score > contributing_pose_score)
      {
        contributing_pose_score = permutation_pose_score;

        if (contributing_pose_score > 0)
        {
          // ROS_WARN_STREAM("Product type: " <<  briefcase_actual_product);
          std::map<std::string, ariac::BriefcaseProduct>::iterator it = 
          mapOfBriefcaseProducts.find(briefcase_actual_product);
          if (it != mapOfBriefcaseProducts.end())
          {
            it->second.isProductCorrectPose = true;
            scorer.numberOfProductsWithCorrectPose++;
          }
        }
      }
    }
    while (std::next_permutation(permutation.begin(), permutation.end()));

  size_t nbOfProductsInBriefcase = mapOfBriefcaseProducts.size();
  // scoring for each product
  for (auto &product : mapOfBriefcaseProducts)
  {
    // +2pts if correct pose AND correct type
    if (product.second.isProductCorrectPose && product.second.isProductCorrectPose)
    {
      product.second.productSuccess = 2;
      // +1pt if correct pose AND correct type AND correct color
      if (product.second.isProductCorrectColor)
      {
        product.second.productSuccess++;
      }
    }
  }

  int nbOfSuccessfulProducts{0};
  // scoring for all products bonus
  for (auto &product : mapOfBriefcaseProducts)
  {
    // +2pts if correct pose AND correct type
    if (product.second.productSuccess == 3)
    {
      nbOfSuccessfulProducts++;
    }
  }

  if (nbOfSuccessfulProducts == nbOfProductsInBriefcase)
    scorer.allProductsBonus = 4 * nbOfSuccessfulProducts;
  }  // end processing each part

  if (!scorer.hasMissingProduct)
  {
    scorer.isShipmentComplete = true;
  }

  scorer.briefcaseProducts = mapOfBriefcaseProducts;

  return scorer;
}
