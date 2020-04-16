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
void AriacScorer::NotifyOrderStarted(gazebo::common::Time time, const nist_gear::Order & order)
{
  AriacScorer::OrderInfo orderInfo;
  orderInfo.start_time = time;
  orderInfo.order = nist_gear::Order::ConstPtr(new nist_gear::Order(order));

  boost::mutex::scoped_lock mutexLock(this->mutex);

  orderInfo.priority = 1;
  if (!this->orders.empty())
  {
    // orders after the first are implicitly higher priority
    orderInfo.priority = 3;
  }

  auto it = this->orders.find(order.order_id);
  if (it != this->orders.end())
  {
    gzerr << "[ARIAC ERROR] Order with duplicate ID '" << order.order_id << "'; overwriting\n";
  }

  this->orders[order.order_id] = orderInfo;
}

/////////////////////////////////////////////////
void AriacScorer::NotifyOrderUpdated(gazebo::common::Time time, ariac::OrderID_t old_order, const nist_gear::Order & order)
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

/////////////////////////////////////////////////
void AriacScorer::NotifyShipmentReceived(gazebo::common::Time time, ariac::ShipmentType_t type, const nist_gear::DetectedShipment & shipment)
{
  AriacScorer::ShipmentInfo shipmentInfo;
  shipmentInfo.submit_time = time;
  shipmentInfo.type = type;
  shipmentInfo.shipment = nist_gear::DetectedShipment::ConstPtr(new nist_gear::DetectedShipment(shipment));

  boost::mutex::scoped_lock mutexLock(this->mutex);
  this->shipments.push_back(shipmentInfo);
}

/////////////////////////////////////////////////
void AriacScorer::NotifyArmArmCollision(gazebo::common::Time /*time*/)
{
  boost::mutex::scoped_lock mutexLock(this->mutex);
  this->arm_arm_collision = true;
}

/////////////////////////////////////////////////
ariac::GameScore AriacScorer::GetGameScore()
{
  boost::mutex::scoped_lock mutexLock(this->mutex);

  ariac::GameScore game_score;

  // arm/arm collision results in zero score, but keep going for logging
  game_score.was_arm_arm_collision = this->arm_arm_collision;

  // Calculate the current score based on received orders and shipments
  // For each order, how many shipments was it supposed to have?
  // Set up shipment sc
  for (auto & opair : this->orders)
  {
    auto order_id = opair.first;
    auto order_info = opair.second;

    gazebo::common::Time start_time = order_info.start_time;
    int priority = order_info.priority;
    nist_gear::Order::ConstPtr order = order_info.order;

    // If order was updated, score based on the lastest version of it
    for (auto & update_info : this->order_updates)
    {
      if (update_info.original_order_id == order_id)
      {
        order = update_info.order;
        start_time = update_info.update_time;
      }
    }

    // Create score class for order
    ariac::OrderScore order_score;
    order_score.orderID = order_id;
    order_score.priority = priority;
    auto oit = game_score.orderScores.find(order_id);
    if (oit != game_score.orderScores.end())
    {
      gzerr << "[ARIAC ERROR] Multiple orders of duplicate ids:" << order_score.orderID << "\n";
    }

    // Create score classes for shipments
    for (const auto & expected_shipment : order->shipments)
    {
      ariac::ShipmentScore shipment_score;
      shipment_score.shipmentType = expected_shipment.shipment_type;
      auto it = order_score.shipmentScores.find(expected_shipment.shipment_type);
      if (it != order_score.shipmentScores.end())
      {
        gzerr << "[ARIAC ERROR] Order contained duplicate shipment types:" << expected_shipment.shipment_type << "\n";
      }
      order_score.shipmentScores[expected_shipment.shipment_type] = shipment_score;
    }

    std::vector<std::string> claimed_shipments;

    // Find actual shipments that belong to this order
    for (const auto & desired_shipment : order->shipments)
    {
      for (const auto & shipment_info : this->shipments)
      {
        if (desired_shipment.shipment_type == shipment_info.type)
        {
          if (shipment_info.submit_time < start_time)
          {
            // Maybe order was updated, this shipment was submitted too early
            continue;
          }
          // If the same shipment was submitted twice, only count the first one
          bool is_claimed = false;
          for (const auto & type : claimed_shipments)
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
          order_score.shipmentScores[desired_shipment.shipment_type] =
            this->GetShipmentScore(shipment_info.submit_time, desired_shipment, *(shipment_info.shipment));
        }
      }
    }

    // Figure out the time taken to complete an order
    if (order_score.isComplete())
    {
      // The latest submitted shipment time is the order completion time
      gazebo::common::Time end = start_time;
      for (auto & sspair : order_score.shipmentScores)
      {
        if (sspair.second.submit_time > end)
        {
          end = sspair.second.submit_time;
        }
      }
      order_score.timeTaken = (end - start_time).Double();
    }

    game_score.orderScores[order_id] = order_score;
  }

  return game_score;
}

ariac::ShipmentScore AriacScorer::GetShipmentScore(
  gazebo::common::Time submit_time,
  const nist_gear::Shipment & desired_shipment,
  const nist_gear::DetectedShipment & actual_shipment)
{
  ariac::ShipmentScore scorer;
  scorer.isSubmitted = true;
  scorer.submit_time = submit_time;

  bool has_faulty_product = false;
  bool is_missing_products = false;
  bool has_unwanted_product = false;
  scorer.productOnlyTypePresence = 0;
  scorer.productTypeAndColorPresence = 0;
  scorer.allProductsBonus = 0;
  scorer.productPose = 0;
  scorer.correctAGV = false;

  if ("any" == desired_shipment.agv_id)
  {
    scorer.correctAGV = true;
  }
  else if ("agv1" == desired_shipment.agv_id)
  {
    scorer.correctAGV = "agv1::kit_tray_1::kit_tray_1::tray" == actual_shipment.destination_id;
  }
  else if ("agv2" == desired_shipment.agv_id)
  {
    scorer.correctAGV = "agv2::kit_tray_2::kit_tray_2::tray" == actual_shipment.destination_id;
  }
  else
  {
    gzerr << "[ARIAC ERROR] desired shipment destination invalid:" << desired_shipment.agv_id << "\n";
  }

  // Separate faulty and non-faulty products
  std::vector<nist_gear::DetectedProduct> non_faulty_products;
  for (const auto & actual_product : actual_shipment.products)
  {
    if (actual_product.is_faulty)
    {
      has_faulty_product = true;
    }
    else
    {
      non_faulty_products.push_back(actual_product);
    }
  }


  //--award 1 point if type is correct and color is incorrect

  //make a copy of non faulty products
  //we will work with this vector for actual products in the trays
  std::vector<nist_gear::DetectedProduct> tmp_non_faulty_products;
  tmp_non_faulty_products = non_faulty_products;
  //--Check product type is correct even if color is wrong
  for (size_t d = 0; d < desired_shipment.products.size(); ++d)
  {
    bool found_exact_product = false;

    auto desired_product = desired_shipment.products[d].type;
    auto desired_product_name = desired_product;
    auto desired_product_type = desired_product.erase(desired_product.rfind('_'));

    //desired_product.erase(desired_product.rfind('_'));
  //  if (!found_exact_product){
    //std::cout << "---Desired product: " << desired_product_name << std::endl;
    for (size_t a = 0; a < tmp_non_faulty_products.size(); ++a){
      auto actual_product_name = tmp_non_faulty_products[a].type;
      //std::cout << "Checking for exact part with " << actual_product_name << std::endl;
      //ex: desired=blue gear, actual=blue gear
      if (desired_product_name.compare(actual_product_name) == 0){
        //std::cout << "Found exact part for desired part: " << desired_product_name << std::endl;
        found_exact_product = true;
        //--give 1pt for correct type
        scorer.productOnlyTypePresence ++;
        //--we are done with this part
        tmp_non_faulty_products.erase(tmp_non_faulty_products.begin()+a);
        break;
      }
    }
  //}

    if (!found_exact_product){
      //std::cout << "Did not find exact part for desired part: " << desired_product << std::endl;
      for (size_t a = 0; a < tmp_non_faulty_products.size(); ++a){
        auto actual_product = tmp_non_faulty_products[a].type;
        //std::cout << "Checking with actual part: " << actual_product << std::endl;
        //--get only part type ex: gear_part from gear_part_blue
        auto actual_product_type = actual_product.erase(actual_product.rfind('_'));
        //std::cout << "Actual type: " << actual_product_type << std::endl;

        //std::cout << "Desired type: " << desired_product_type << std::endl;

        if (desired_product_type.compare(actual_product_type) == 0){
          //--give 1pt for correct type
          scorer.productOnlyTypePresence ++;
          //std::cout << "+1 pt for matching type" << std::endl;
          //--we are done with this part
          tmp_non_faulty_products.erase(tmp_non_faulty_products.begin()+a);
          // found_exact_product=true;
          break;
        }
      }
    }
  }

  //--Award 1 pt if color is correct for correct part types
  // Map of product type to indexes in desired products (first) and indexes in non faulty actual products (second)
  std::map<std::string, std::pair<std::vector<size_t>, std::vector<size_t>>> product_type_map;
  for (size_t d = 0; d < desired_shipment.products.size(); ++d)
  {
    const auto & desired_product = desired_shipment.products[d];
    auto & mapping = product_type_map[desired_product.type];
    //std::cout << "desired product type: " << desired_product.type << std::endl;
    mapping.first.push_back(d);
  }
  for (size_t a = 0; a < non_faulty_products.size(); ++a)
  {
    const auto & actual_product = non_faulty_products[a];
    //std::cout << "actual product type: " << actual_product.type << std::endl;
    if (0u == product_type_map.count(actual_product.type))
    {
      // since desired products were put into the type map first, this product must be unwanted
      has_unwanted_product = true;
      continue;
    }
    auto & mapping = product_type_map.at(actual_product.type);
    mapping.second.push_back(a);
  }

  for (const auto & type_pair : product_type_map)
  {
    const std::vector<size_t> & desired_indexes = type_pair.second.first;
    const std::vector<size_t> & actual_indexes = type_pair.second.second;
    auto product_name = type_pair.first;

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
        const auto & desired_product = desired_shipment.products[desired_indexes[d]];
        const auto & actual_product = non_faulty_products[actual_indexes[actual_index_index]];
        // Add points for each product in the correct pose
        const double translation_target = 0.03;  // 3 cm
        const double orientation_target = 0.1;  // 0.1 rad
        // get translation distance
        ignition::math::Vector3d posnDiff(
          desired_product.pose.position.x - actual_product.pose.position.x,
          desired_product.pose.position.y - actual_product.pose.position.y,
          0);
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
        // TODO: this value can probably be derived using relationships between
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
        if ( (std::abs(angleDiff) < orientation_target)
          || (std::abs(std::abs(angleDiff) - 2 * M_PI) <= orientation_target))
        {
           permutation_pose_score += 1.0;
        }
      }
      if (permutation_pose_score > contributing_pose_score)
      {
        contributing_pose_score = permutation_pose_score;
      }
    } while (std::next_permutation(permutation.begin(), permutation.end()));

    // Add the pose score contributed by the highest scoring permutation
    scorer.productPose += contributing_pose_score;
  }

  if (!is_missing_products)
  {
    scorer.isComplete = true;
  }
  if (!has_faulty_product && !has_unwanted_product && !is_missing_products)
  {
    scorer.allProductsBonus = scorer.productTypeAndColorPresence;
  }

  return scorer;
}
