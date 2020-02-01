/*
 * Copyright (C) 2019 Open Source Robotics Foundation
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

#include <algorithm>

#include <gtest/gtest.h>

#include <geometry_msgs/Pose.h>
#include <nist_gear/AriacScorer.h>
#include <nist_gear/DetectedShipment.h>
#include <nist_gear/Order.h>
#include <nist_gear/Shipment.h>

#include <ignition/math/Quaternion.hh>

using gazebo::common::Time;
using nist_gear::DetectedShipment;
using nist_gear::Order;
using nist_gear::Shipment;


geometry_msgs::Pose
make_pose(
  double x, double y, double z,
  double roll, double pitch,double yaw)
{
  geometry_msgs::Pose pose;
  pose.position.x = x;
  pose.position.y = y;
  pose.position.z = z;

  ignition::math::Quaterniond rot(roll, pitch, yaw);
  pose.orientation.w = rot.W();
  pose.orientation.x = rot.X();
  pose.orientation.y = rot.Y();
  pose.orientation.z = rot.Z();

  return pose;
}

const size_t ALL_PRODUCTS = 0x01;

double make_shipment_score(
  int correct_products_present,
  int products_with_correct_pose,
  size_t flags = 0)
{
  const double all_products_bonus = (flags & ALL_PRODUCTS) ? correct_products_present : 0;
  return correct_products_present + products_with_correct_pose + all_products_bonus;
}

const double HIGH_PRIORITY_FACTOR = 3.0;


TEST(TestAriacScorer, nothing_happened)
{
  AriacScorer scorer;
  EXPECT_DOUBLE_EQ(0.0, scorer.GetGameScore().total());
}

TEST(TestAriacScorer, order_with_no_shipments)
{
  AriacScorer scorer;

  Order order;
  order.order_id = "order_0";
  order.shipments.emplace_back();
  order.shipments.back().shipment_type = "order_0_shipment_0";
  order.shipments.back().agv_id = "any";
  order.shipments.back().products.emplace_back();
  order.shipments.back().products.back().type = "gear_part";
  order.shipments.back().products.back().pose = make_pose(0, 1, 2, 0, 0, 5);
  Time start_time(123, 456);

  scorer.NotifyOrderStarted(start_time, order);
  EXPECT_DOUBLE_EQ(0.0, scorer.GetGameScore().total());
}

TEST(TestAriacScorer, updated_order_with_no_shipments)
{
  AriacScorer scorer;

  Order order;
  order.order_id = "order_0";
  order.shipments.emplace_back();
  order.shipments.back().shipment_type = "order_0_shipment_0";
  order.shipments.back().agv_id = "any";
  order.shipments.back().products.emplace_back();
  order.shipments.back().products.back().type = "gear_part";
  order.shipments.back().products.back().pose = make_pose(0, 1, 2, 0, 0, 5);
  Time start_time(123, 456);

  scorer.NotifyOrderStarted(start_time, order);

  Time update_time(789, 123);
  scorer.NotifyOrderUpdated(update_time, order.order_id, order);

  EXPECT_DOUBLE_EQ(0.0, scorer.GetGameScore().total());
}

TEST(TestAriacScorer, order_with_one_empty_shipment)
{
  AriacScorer scorer;

  Order order;
  order.order_id = "order_0";
  order.shipments.emplace_back();
  order.shipments.back().shipment_type = "order_0_shipment_0";
  order.shipments.back().agv_id = "any";
  order.shipments.back().products.emplace_back();
  order.shipments.back().products.back().type = "gear_part";
  order.shipments.back().products.back().pose = make_pose(0, 1, 2, 0, 0, 5);
  Time start_time(123, 456);

  scorer.NotifyOrderStarted(start_time, order);

  Time shipment_time(789, 123);
  DetectedShipment shipment;
  scorer.NotifyShipmentReceived(shipment_time, "order_0_shipment_0", shipment);

  auto score = scorer.GetGameScore();
  EXPECT_DOUBLE_EQ(0.0, score.total());
  EXPECT_TRUE(score.orderScores["order_0"].isComplete());
  EXPECT_EQ(1u, score.orderScores["order_0"].shipmentScores.size());
  EXPECT_DOUBLE_EQ((shipment_time - start_time).Double(), score.orderScores["order_0"].timeTaken);
}

TEST(TestAriacScorer, order_with_one_unrelated_shipment)
{
  AriacScorer scorer;

  Order order;
  order.order_id = "order_0";
  order.shipments.emplace_back();
  order.shipments.back().shipment_type = "order_0_shipment_0";
  order.shipments.back().agv_id = "any";
  order.shipments.back().products.emplace_back();
  order.shipments.back().products.back().type = "gear_part";
  order.shipments.back().products.back().pose = make_pose(0, 1, 2, 0, 0, 5);
  Time start_time(123, 456);

  scorer.NotifyOrderStarted(start_time, order);

  Time shipment_time(789, 123);
  DetectedShipment shipment;
  shipment.products.emplace_back();
  shipment.products.back().is_faulty = false;
  shipment.products.back().type = order.shipments.back().products.back().type;
  shipment.products.back().pose = order.shipments.back().products.back().pose;
  scorer.NotifyShipmentReceived(shipment_time, "order_1234_shipment_0", shipment);

  auto score = scorer.GetGameScore();
  EXPECT_DOUBLE_EQ(0.0, score.total());
  EXPECT_FALSE(score.orderScores["order_0"].isComplete());
}

TEST(TestAriacScorer, order_with_one_shipment_fulfilled_perfectly)
{
  AriacScorer scorer;

  Order order;
  order.order_id = "order_0";
  order.shipments.emplace_back();
  order.shipments.back().shipment_type = "order_0_shipment_0";
  order.shipments.back().agv_id = "any";
  order.shipments.back().products.emplace_back();
  order.shipments.back().products.back().type = "gear_part";
  order.shipments.back().products.back().pose = make_pose(0, 1, 2, 0, 0, 5);
  Time start_time(123, 456);

  scorer.NotifyOrderStarted(start_time, order);

  Time shipment_time(789, 123);
  DetectedShipment shipment;
  shipment.products.emplace_back();
  shipment.products.back().is_faulty = false;
  shipment.products.back().type = order.shipments.back().products.back().type;
  shipment.products.back().pose = order.shipments.back().products.back().pose;
  scorer.NotifyShipmentReceived(shipment_time, order.shipments.back().shipment_type, shipment);

  auto score = scorer.GetGameScore();
  EXPECT_DOUBLE_EQ(make_shipment_score(1, 1, ALL_PRODUCTS), score.total()) << score;
  EXPECT_TRUE(score.orderScores["order_0"].isComplete());
  EXPECT_DOUBLE_EQ((shipment_time - start_time).Double(), score.orderScores["order_0"].timeTaken);
}

TEST(TestAriacScorer, order_with_one_shipment_fulfilled_perfectly_wrong_agv)
{
  AriacScorer scorer;

  Order order;
  order.order_id = "order_0";
  order.shipments.emplace_back();
  order.shipments.back().shipment_type = "order_0_shipment_0";
  order.shipments.back().agv_id = "agv2";
  order.shipments.back().products.emplace_back();
  order.shipments.back().products.back().type = "gear_part";
  order.shipments.back().products.back().pose = make_pose(0, 1, 2, 0, 0, 5);
  order.shipments.emplace_back();
  order.shipments.back().shipment_type = "order_0_shipment_1";
  order.shipments.back().agv_id = "agv1";
  order.shipments.back().products.emplace_back();
  order.shipments.back().products.back().type = "gear_part";
  order.shipments.back().products.back().pose = make_pose(0, 1, 2, 0, 0, 5);
  Time start_time(123, 456);

  scorer.NotifyOrderStarted(start_time, order);

  Time shipment_time(789, 123);
  DetectedShipment shipment;
  shipment.destination_id = "agv1::kit_tray_1::kit_tray_1::tray";
  shipment.products.emplace_back();
  shipment.products.back().is_faulty = false;
  shipment.products.back().type = order.shipments.back().products.back().type;
  shipment.products.back().pose = order.shipments.back().products.back().pose;
  scorer.NotifyShipmentReceived(shipment_time, "order_0_shipment_0", shipment);
  auto score = scorer.GetGameScore();
  EXPECT_DOUBLE_EQ(0.0, score.total()) << score;

  scorer.NotifyShipmentReceived(shipment_time, "order_0_shipment_1", shipment);
  score = scorer.GetGameScore();
  EXPECT_DOUBLE_EQ(make_shipment_score(1, 1, ALL_PRODUCTS), score.total()) << score;
  EXPECT_TRUE(score.orderScores["order_0"].isComplete());
}

TEST(TestAriacScorer, order_with_one_shipment_faulty_part)
{
  AriacScorer scorer;

  Order order;
  order.order_id = "order_0";
  order.shipments.emplace_back();
  order.shipments.back().shipment_type = "order_0_shipment_0";
  order.shipments.back().agv_id = "any";
  order.shipments.back().products.emplace_back();
  order.shipments.back().products.back().type = "gear_part";
  order.shipments.back().products.back().pose = make_pose(0, 1, 2, 0, 0, 5);
  Time start_time(123, 456);

  scorer.NotifyOrderStarted(start_time, order);

  Time shipment_time(789, 123);
  DetectedShipment shipment;
  shipment.products.emplace_back();
  shipment.products.back().is_faulty = true;
  shipment.products.back().type = order.shipments.back().products.back().type;
  shipment.products.back().pose = order.shipments.back().products.back().pose;
  scorer.NotifyShipmentReceived(shipment_time, order.shipments.back().shipment_type, shipment);

  auto score = scorer.GetGameScore();
  EXPECT_DOUBLE_EQ(make_shipment_score(0, 0), score.total()) << score;
}

TEST(TestAriacScorer, order_with_one_shipment_fulfilled_poor_translation)
{
  AriacScorer scorer;

  Order order;
  order.order_id = "order_0";
  order.shipments.emplace_back();
  order.shipments.back().shipment_type = "order_0_shipment_0";
  order.shipments.back().agv_id = "any";
  order.shipments.back().products.emplace_back();
  order.shipments.back().products.back().type = "gear_part";
  order.shipments.back().products.back().pose = make_pose(0, 1, 2, 0, 0, 5);
  Time start_time(123, 456);

  scorer.NotifyOrderStarted(start_time, order);

  Time shipment_time(789, 123);
  DetectedShipment shipment;
  shipment.products.emplace_back();
  shipment.products.back().is_faulty = false;
  shipment.products.back().type = order.shipments.back().products.back().type;
  shipment.products.back().pose = order.shipments.back().products.back().pose;
  shipment.products.back().pose.position.x += 0.04;
  scorer.NotifyShipmentReceived(shipment_time, order.shipments.back().shipment_type, shipment);

  auto score = scorer.GetGameScore();
  EXPECT_DOUBLE_EQ(make_shipment_score(1, 0, ALL_PRODUCTS), score.total()) << score;
  EXPECT_DOUBLE_EQ(0.0, score.orderScores.begin()->second.shipmentScores.begin()->second.productPose);
}

TEST(TestAriacScorer, order_with_one_shipment_fulfilled_poor_orientation)
{
  AriacScorer scorer;

  Order order;
  order.order_id = "order_0";
  order.shipments.emplace_back();
  order.shipments.back().shipment_type = "order_0_shipment_0";
  order.shipments.back().agv_id = "any";
  order.shipments.back().products.emplace_back();
  order.shipments.back().products.back().type = "gear_part";
  order.shipments.back().products.back().pose = make_pose(0, 1, 2, 0, 0, 5);
  Time start_time(123, 456);

  scorer.NotifyOrderStarted(start_time, order);

  Time shipment_time(789, 123);
  DetectedShipment shipment;
  shipment.products.emplace_back();
  shipment.products.back().is_faulty = false;
  shipment.products.back().type = order.shipments.back().products.back().type;
  shipment.products.back().pose = make_pose(0, 1, 2, 0, 0, 5.2);
  scorer.NotifyShipmentReceived(shipment_time, order.shipments.back().shipment_type, shipment);

  auto score = scorer.GetGameScore();
  EXPECT_DOUBLE_EQ(make_shipment_score(1, 0, ALL_PRODUCTS), score.total()) << score;
}

TEST(TestAriacScorer, order_with_one_shipment_fulfilled_missing_product)
{
  AriacScorer scorer;

  Order order;
  order.order_id = "order_0";
  order.shipments.emplace_back();
  order.shipments.back().shipment_type = "order_0_shipment_0";
  order.shipments.back().agv_id = "any";
  order.shipments.back().products.emplace_back();
  order.shipments.back().products.back().type = "gear_part";
  order.shipments.back().products.back().pose = make_pose(0, 1, 2, 0, 0, 5);
  order.shipments.back().products.emplace_back();
  order.shipments.back().products.back().type = "pulley_part";
  order.shipments.back().products.back().pose = make_pose(3, 4, 5, 0, 0, 5);
  Time start_time(123, 456);

  scorer.NotifyOrderStarted(start_time, order);

  Time shipment_time(789, 123);
  DetectedShipment shipment;
  shipment.products.emplace_back();
  shipment.products.back().is_faulty = false;
  shipment.products.back().type = order.shipments.back().products.back().type;
  shipment.products.back().pose = order.shipments.back().products.back().pose;
  scorer.NotifyShipmentReceived(shipment_time, order.shipments.back().shipment_type, shipment);

  auto score = scorer.GetGameScore();
  EXPECT_DOUBLE_EQ(make_shipment_score(1, 1), score.total()) << score;
}

TEST(TestAriacScorer, order_with_one_shipment_fulfilled_extra_product_different_type)
{
  AriacScorer scorer;

  Order order;
  order.order_id = "order_0";
  order.shipments.emplace_back();
  order.shipments.back().shipment_type = "order_0_shipment_0";
  order.shipments.back().agv_id = "any";
  order.shipments.back().products.emplace_back();
  order.shipments.back().products.back().type = "gear_part";
  order.shipments.back().products.back().pose = make_pose(0, 1, 2, 0, 0, 5);
  Time start_time(123, 456);

  scorer.NotifyOrderStarted(start_time, order);

  Time shipment_time(789, 123);
  DetectedShipment shipment;
  shipment.products.emplace_back();
  shipment.products.back().is_faulty = false;
  shipment.products.back().type = order.shipments.back().products.back().type;
  shipment.products.back().pose = order.shipments.back().products.back().pose;
  shipment.products.emplace_back();
  shipment.products.back().type = "pulley_part";
  shipment.products.back().pose = make_pose(3, 4, 5, 0, 0, 5);
  scorer.NotifyShipmentReceived(shipment_time, order.shipments.back().shipment_type, shipment);

  auto score = scorer.GetGameScore();
  EXPECT_DOUBLE_EQ(make_shipment_score(1, 1), score.total()) << score;
}

TEST(TestAriacScorer, order_with_one_shipment_fulfilled_extra_product_same_type)
{
  AriacScorer scorer;

  Order order;
  order.order_id = "order_0";
  order.shipments.emplace_back();
  order.shipments.back().shipment_type = "order_0_shipment_0";
  order.shipments.back().agv_id = "any";
  order.shipments.back().products.emplace_back();
  order.shipments.back().products.back().type = "gear_part";
  order.shipments.back().products.back().pose = make_pose(0, 1, 2, 0, 0, 5);
  Time start_time(123, 456);

  scorer.NotifyOrderStarted(start_time, order);

  Time shipment_time(789, 123);
  DetectedShipment shipment;
  shipment.products.emplace_back();
  shipment.products.back().is_faulty = false;
  shipment.products.back().type = order.shipments.back().products.back().type;
  shipment.products.back().pose = order.shipments.back().products.back().pose;
  shipment.products.emplace_back();
  shipment.products.back().type = "gear_part";
  shipment.products.back().pose = make_pose(3, 4, 5, 0, 0, 5);
  scorer.NotifyShipmentReceived(shipment_time, order.shipments.back().shipment_type, shipment);

  auto score = scorer.GetGameScore();
  EXPECT_DOUBLE_EQ(make_shipment_score(1, 1), score.total()) << score;
}

TEST(TestAriacScorer, order_with_one_shipment_fulfilled_mismatched_product)
{
  AriacScorer scorer;

  Order order;
  order.order_id = "order_0";
  order.shipments.emplace_back();
  order.shipments.back().shipment_type = "order_0_shipment_0";
  order.shipments.back().agv_id = "any";
  order.shipments.back().products.emplace_back();
  order.shipments.back().products.back().type = "gear_part";
  order.shipments.back().products.back().pose = make_pose(0, 1, 2, 0, 0, 5);
  Time start_time(123, 456);

  scorer.NotifyOrderStarted(start_time, order);

  Time shipment_time(789, 123);
  DetectedShipment shipment;
  shipment.products.emplace_back();
  shipment.products.back().is_faulty = false;
  shipment.products.back().type = "pulley_part";
  shipment.products.back().pose = order.shipments.back().products.back().pose;
  scorer.NotifyShipmentReceived(shipment_time, order.shipments.back().shipment_type, shipment);

  auto score = scorer.GetGameScore();
  EXPECT_DOUBLE_EQ(make_shipment_score(0, 0), score.total()) << score;
}

TEST(TestAriacScorer, order_with_one_shipment_fulfilled_flipped_product)
{
  AriacScorer scorer;

  Order order;
  order.order_id = "order_0";
  order.shipments.emplace_back();
  order.shipments.back().shipment_type = "order_0_shipment_0";
  order.shipments.back().agv_id = "any";
  order.shipments.back().products.emplace_back();
  order.shipments.back().products.back().type = "gear_part";
  order.shipments.back().products.back().pose = make_pose(0, 1, 2, 0, 0, 5);
  Time start_time(123, 456);

  scorer.NotifyOrderStarted(start_time, order);

  Time shipment_time(789, 123);
  DetectedShipment shipment;
  shipment.products.emplace_back();
  shipment.products.back().is_faulty = false;
  shipment.products.back().type = order.shipments.back().products.back().type;
  shipment.products.back().pose = make_pose(0, 1, 2, 0, 3.14159, 5);
  scorer.NotifyShipmentReceived(shipment_time, order.shipments.back().shipment_type, shipment);

  auto score = scorer.GetGameScore();
  EXPECT_DOUBLE_EQ(make_shipment_score(1, 0, ALL_PRODUCTS), score.total()) << score;
}

TEST(TestAriacScorer, order_with_two_shipments_perfect)
{
  AriacScorer scorer;

  Order order;
  order.order_id = "order_0";
  order.shipments.emplace_back();
  order.shipments.back().shipment_type = "order_0_shipment_0";
  order.shipments.back().agv_id = "any";
  order.shipments.back().products.emplace_back();
  order.shipments.back().products.back().type = "gear_part";
  order.shipments.back().products.back().pose = make_pose(0, 1, 2, 0, 0, 5);
  order.shipments.emplace_back();
  order.shipments.back().shipment_type = "order_0_shipment_1";
  order.shipments.back().agv_id = "any";
  order.shipments.back().products.emplace_back();
  order.shipments.back().products.back().type = "pulley_part";
  order.shipments.back().products.back().pose = make_pose(0, 1, 2, 0, 0, 5);
  Time start_time(123, 456);

  scorer.NotifyOrderStarted(start_time, order);

  {
    Time shipment_time(789, 123);
    DetectedShipment shipment;
    shipment.products.emplace_back();
    shipment.products.back().is_faulty = false;
    shipment.products.back().type = order.shipments.front().products.back().type;
    shipment.products.back().pose = order.shipments.front().products.back().pose;
    scorer.NotifyShipmentReceived(shipment_time, "order_0_shipment_0", shipment);
  }
  {
    Time shipment_time(101112, 123);
    DetectedShipment shipment;
    shipment.products.emplace_back();
    shipment.products.back().is_faulty = false;
    shipment.products.back().type = order.shipments.back().products.back().type;
    shipment.products.back().pose = order.shipments.back().products.back().pose;
    scorer.NotifyShipmentReceived(shipment_time, "order_0_shipment_1", shipment);
  }

  auto score = scorer.GetGameScore();
  EXPECT_DOUBLE_EQ(
    make_shipment_score(1, 1, ALL_PRODUCTS) + make_shipment_score(1, 1, ALL_PRODUCTS),
    score.total()) << score;
}

TEST(TestAriacScorer, order_with_two_shipments_first_perfect)
{
  AriacScorer scorer;

  Order order;
  order.order_id = "order_0";
  order.shipments.emplace_back();
  order.shipments.back().shipment_type = "order_0_shipment_0";
  order.shipments.back().agv_id = "any";
  order.shipments.back().products.emplace_back();
  order.shipments.back().products.back().type = "gear_part";
  order.shipments.back().products.back().pose = make_pose(0, 1, 2, 0, 0, 5);
  order.shipments.emplace_back();
  order.shipments.back().shipment_type = "order_0_shipment_1";
  order.shipments.back().agv_id = "any";
  order.shipments.back().products.emplace_back();
  order.shipments.back().products.back().type = "pulley_part";
  order.shipments.back().products.back().pose = make_pose(0, 1, 2, 0, 0, 5);
  Time start_time(123, 456);

  scorer.NotifyOrderStarted(start_time, order);

  {
    Time shipment_time(789, 123);
    DetectedShipment shipment;
    shipment.products.emplace_back();
    shipment.products.back().is_faulty = false;
    shipment.products.back().type = order.shipments.front().products.back().type;
    shipment.products.back().pose = order.shipments.front().products.back().pose;
    scorer.NotifyShipmentReceived(shipment_time, "order_0_shipment_0", shipment);
  }
  {
    Time shipment_time(101112, 123);
    DetectedShipment shipment;
    shipment.products.emplace_back();
    shipment.products.back().is_faulty = false;
    shipment.products.back().type = order.shipments.back().products.back().type;
    shipment.products.back().pose = make_pose(0, 0, 0, 0, 0, 0);
    scorer.NotifyShipmentReceived(shipment_time, "order_0_shipment_1", shipment);
  }

  auto score = scorer.GetGameScore();
  EXPECT_DOUBLE_EQ(
    make_shipment_score(1, 1, ALL_PRODUCTS) + make_shipment_score(1, 0, ALL_PRODUCTS),
    score.total()) << score;
}

TEST(TestAriacScorer, order_with_two_shipments_first_before_second)
{
  AriacScorer scorer;

  Order order;
  order.order_id = "order_0";
  order.shipments.emplace_back();
  order.shipments.back().shipment_type = "order_0_shipment_0";
  order.shipments.back().agv_id = "any";
  order.shipments.back().products.emplace_back();
  order.shipments.back().products.back().type = "gear_part";
  order.shipments.back().products.back().pose = make_pose(0, 1, 2, 0, 0, 5);
  order.shipments.emplace_back();
  order.shipments.back().shipment_type = "order_0_shipment_1";
  order.shipments.back().agv_id = "any";
  order.shipments.back().products.emplace_back();
  order.shipments.back().products.back().type = "pulley_part";
  order.shipments.back().products.back().pose = make_pose(0, 1, 2, 0, 0, 5);
  Time start_time(123, 456);

  scorer.NotifyOrderStarted(start_time, order);

  Time latest_shipment_time(101112, 123);
  {
    DetectedShipment shipment;
    scorer.NotifyShipmentReceived(latest_shipment_time, "order_0_shipment_0", shipment);
  }
  {
    Time shipment_time(789, 123);
    DetectedShipment shipment;
    scorer.NotifyShipmentReceived(shipment_time, "order_0_shipment_1", shipment);
  }

  auto score = scorer.GetGameScore();
  EXPECT_DOUBLE_EQ(
    make_shipment_score(0, 0) + make_shipment_score(0, 0),
    score.total()) << score;
  EXPECT_DOUBLE_EQ(
    (latest_shipment_time - start_time).Double(),
    score.orderScores["order_0"].timeTaken);
}

TEST(TestAriacScorer, order_with_two_shipments_second_before_first)
{
  AriacScorer scorer;

  Order order;
  order.order_id = "order_0";
  order.shipments.emplace_back();
  order.shipments.back().shipment_type = "order_0_shipment_0";
  order.shipments.back().agv_id = "any";
  order.shipments.back().products.emplace_back();
  order.shipments.back().products.back().type = "gear_part";
  order.shipments.back().products.back().pose = make_pose(0, 1, 2, 0, 0, 5);
  order.shipments.emplace_back();
  order.shipments.back().shipment_type = "order_0_shipment_1";
  order.shipments.back().agv_id = "any";
  order.shipments.back().products.emplace_back();
  order.shipments.back().products.back().type = "pulley_part";
  order.shipments.back().products.back().pose = make_pose(0, 1, 2, 0, 0, 5);
  Time start_time(123, 456);

  scorer.NotifyOrderStarted(start_time, order);

  {
    Time shipment_time(789, 123);
    DetectedShipment shipment;
    scorer.NotifyShipmentReceived(shipment_time, "order_0_shipment_0", shipment);
  }
  Time latest_shipment_time(101112, 123);
  {
    DetectedShipment shipment;
    scorer.NotifyShipmentReceived(latest_shipment_time, "order_0_shipment_1", shipment);
  }

  auto score = scorer.GetGameScore();
  EXPECT_DOUBLE_EQ(
    make_shipment_score(0, 0) + make_shipment_score(0, 0),
    score.total()) << score;
  EXPECT_DOUBLE_EQ(
    (latest_shipment_time - start_time).Double(),
    score.orderScores["order_0"].timeTaken);
}

TEST(TestAriacScorer, order_with_one_shipment_multiple_products_perfect)
{
  AriacScorer scorer;

  Order order;
  order.order_id = "order_0";
  order.shipments.emplace_back();
  order.shipments.back().shipment_type = "order_0_shipment_0";
  order.shipments.back().agv_id = "any";
  order.shipments.back().products.emplace_back();
  order.shipments.back().products.back().type = "gear_part";
  order.shipments.back().products.back().pose = make_pose(0, 1, 2, 0, 0, 5);
  order.shipments.back().products.emplace_back();
  order.shipments.back().products.back().type = "pulley_part";
  order.shipments.back().products.back().pose = make_pose(3, 4, 5, 0, 0, 0);
  order.shipments.back().products.emplace_back();
  order.shipments.back().products.back().type = "pulley_part";
  order.shipments.back().products.back().pose = make_pose(2, 0, 1, 0, 0, 0);
  Time start_time(123, 456);

  scorer.NotifyOrderStarted(start_time, order);

  Time shipment_time(789, 123);
  DetectedShipment shipment;
  for (auto & desired_product : order.shipments.back().products)
  {
    shipment.products.emplace_back();
    shipment.products.back().is_faulty = false;
    shipment.products.back().type = desired_product.type;
    shipment.products.back().pose = desired_product.pose;
  }
  scorer.NotifyShipmentReceived(shipment_time, order.shipments.back().shipment_type, shipment);

  auto score = scorer.GetGameScore();
  EXPECT_DOUBLE_EQ(make_shipment_score(3, 3, ALL_PRODUCTS), score.total()) << score;
}

TEST(TestAriacScorer, order_with_one_shipment_multiple_products_one_wrong_translation)
{
  AriacScorer scorer;

  Order order;
  order.order_id = "order_0";
  order.shipments.emplace_back();
  order.shipments.back().shipment_type = "order_0_shipment_0";
  order.shipments.back().agv_id = "any";
  order.shipments.back().products.emplace_back();
  order.shipments.back().products.back().type = "gear_part";
  order.shipments.back().products.back().pose = make_pose(0, 1, 2, 0, 0, 5);
  order.shipments.back().products.emplace_back();
  order.shipments.back().products.back().type = "pulley_part";
  order.shipments.back().products.back().pose = make_pose(3, 4, 5, 0, 0, 0);
  order.shipments.back().products.emplace_back();
  order.shipments.back().products.back().type = "pulley_part";
  order.shipments.back().products.back().pose = make_pose(2, 0, 1, 0, 0, 0);
  Time start_time(123, 456);

  scorer.NotifyOrderStarted(start_time, order);

  Time shipment_time(789, 123);
  DetectedShipment shipment;
  for (auto & desired_product : order.shipments.back().products)
  {
    shipment.products.emplace_back();
    shipment.products.back().is_faulty = false;
    shipment.products.back().type = desired_product.type;
    shipment.products.back().pose = desired_product.pose;
  }
  shipment.products.back().pose.position.x -= 0.04;
  scorer.NotifyShipmentReceived(shipment_time, order.shipments.back().shipment_type, shipment);

  auto score = scorer.GetGameScore();
  EXPECT_DOUBLE_EQ(make_shipment_score(3, 2, ALL_PRODUCTS), score.total()) << score;
}

TEST(TestAriacScorer, order_with_one_shipment_multiple_products_one_wrong_orientation)
{
  AriacScorer scorer;

  Order order;
  order.order_id = "order_0";
  order.shipments.emplace_back();
  order.shipments.back().shipment_type = "order_0_shipment_0";
  order.shipments.back().agv_id = "any";
  order.shipments.back().products.emplace_back();
  order.shipments.back().products.back().type = "gear_part";
  order.shipments.back().products.back().pose = make_pose(0, 1, 2, 0, 0, 5);
  order.shipments.back().products.emplace_back();
  order.shipments.back().products.back().type = "pulley_part";
  order.shipments.back().products.back().pose = make_pose(3, 4, 5, 0, 0, 0);
  order.shipments.back().products.emplace_back();
  order.shipments.back().products.back().type = "pulley_part";
  order.shipments.back().products.back().pose = make_pose(2, 0, 1, 0, 0, 0);
  Time start_time(123, 456);

  scorer.NotifyOrderStarted(start_time, order);

  Time shipment_time(789, 123);
  DetectedShipment shipment;
  for (auto & desired_product : order.shipments.back().products)
  {
    shipment.products.emplace_back();
    shipment.products.back().is_faulty = false;
    shipment.products.back().type = desired_product.type;
    shipment.products.back().pose = desired_product.pose;
  }
  shipment.products.back().pose.orientation.w = 0;
  shipment.products.back().pose.orientation.x = 1;
  shipment.products.back().pose.orientation.y = 0;
  shipment.products.back().pose.orientation.z = 0;
  scorer.NotifyShipmentReceived(shipment_time, order.shipments.back().shipment_type, shipment);

  auto score = scorer.GetGameScore();
  EXPECT_DOUBLE_EQ(make_shipment_score(3, 2, ALL_PRODUCTS), score.total()) << score;
}

TEST(TestAriacScorer, order_with_many_shipments_many_parts_close_enough)
{
  AriacScorer scorer;

  Order order;
  order.order_id = "order_0";
  order.shipments.emplace_back();
  order.shipments.back().shipment_type = "order_0_shipment_0";
  order.shipments.back().agv_id = "any";
  order.shipments.back().products.emplace_back();
  order.shipments.back().products.back().type = "gear_part";
  order.shipments.back().products.back().pose = make_pose(0, 1, 2, 0, 0, 5);
  order.shipments.back().products.emplace_back();
  order.shipments.back().products.back().type = "pulley_part";
  order.shipments.back().products.back().pose = make_pose(3, 4, 5, 0, 0, 0);
  order.shipments.back().products.emplace_back();
  order.shipments.back().products.back().type = "pulley_part";
  order.shipments.back().products.back().pose = make_pose(2, 0, 1, 0, 0, 0);
  order.shipments.emplace_back();
  order.shipments.back().shipment_type = "order_0_shipment_1";
  order.shipments.back().agv_id = "any";
  order.shipments.back().products.emplace_back();
  order.shipments.back().products.back().type = "piston_part";
  order.shipments.back().products.back().pose = make_pose(1, 2, 3, 1, 0, 5);
  order.shipments.back().products.emplace_back();
  order.shipments.back().products.back().type = "gear_part";
  order.shipments.back().products.back().pose = make_pose(3, 4, 6, 1, 0, 0);
  order.shipments.back().products.emplace_back();
  order.shipments.back().products.back().type = "pulley_part";
  order.shipments.back().products.back().pose = make_pose(0, 0, 1, 1, 0, 0);
  Time start_time(123, 456);

  scorer.NotifyOrderStarted(start_time, order);


  std::default_random_engine generator;
  generator.seed(std::chrono::system_clock::now().time_since_epoch().count());
  std::uniform_real_distribution<double> distribution(-0.02, 0.02);
  for (auto & shipment : order.shipments)
  {
    DetectedShipment actual_shipment;
    Time shipment_time(789, 123);
    for (auto & desired_product : shipment.products)
    {
      actual_shipment.products.emplace_back();
      actual_shipment.products.back().is_faulty = false;
      actual_shipment.products.back().type = desired_product.type;
      actual_shipment.products.back().pose = desired_product.pose;
      actual_shipment.products.back().pose.position.x += distribution(generator);
      actual_shipment.products.back().pose.position.y += distribution(generator);
      actual_shipment.products.back().pose.position.z += distribution(generator);
    }
    scorer.NotifyShipmentReceived(shipment_time, shipment.shipment_type, actual_shipment);
  }

  auto score = scorer.GetGameScore();
  EXPECT_DOUBLE_EQ(
    make_shipment_score(3, 3, ALL_PRODUCTS) + make_shipment_score(3, 3, ALL_PRODUCTS),
    score.total()) << score;
}

TEST(TestAriacScorer, order_with_many_shipments_mixed_success)
{
  AriacScorer scorer;

  Order order;
  order.order_id = "order_0";
  order.shipments.emplace_back();
  order.shipments.back().shipment_type = "order_0_shipment_0";
  order.shipments.back().agv_id = "any";
  order.shipments.back().products.emplace_back();
  order.shipments.back().products.back().type = "gear_part";
  order.shipments.back().products.back().pose = make_pose(0, 1, 2, 0, 0, 5);
  order.shipments.back().products.emplace_back();
  order.shipments.back().products.back().type = "pulley_part";
  order.shipments.back().products.back().pose = make_pose(3, 4, 5, 0, 0, 0);
  order.shipments.back().products.emplace_back();
  order.shipments.back().products.back().type = "pulley_part";
  order.shipments.back().products.back().pose = make_pose(2, 0, 1, 0, 0, 0);
  order.shipments.emplace_back();
  order.shipments.back().shipment_type = "order_0_shipment_1";
  order.shipments.back().agv_id = "any";
  order.shipments.back().products.emplace_back();
  order.shipments.back().products.back().type = "piston_part";
  order.shipments.back().products.back().pose = make_pose(1, 2, 3, 1, 0, 5);
  order.shipments.back().products.emplace_back();
  order.shipments.back().products.back().type = "gear_part";
  order.shipments.back().products.back().pose = make_pose(3, 4, 6, 1, 0, 0);
  order.shipments.back().products.emplace_back();
  order.shipments.back().products.back().type = "pulley_part";
  order.shipments.back().products.back().pose = make_pose(0, 0, 1, 1, 0, 0);
  Time start_time(123, 456);

  scorer.NotifyOrderStarted(start_time, order);

  {
    Time shipment_time(789, 123);
    DetectedShipment actual_shipment;
    actual_shipment.products.emplace_back();
    actual_shipment.products.back().is_faulty = false;
    actual_shipment.products.back().type = "pulley_part";
    actual_shipment.products.back().pose = make_pose(2.05, 0, 1, 0, 0, 0);
    actual_shipment.products.emplace_back();
    actual_shipment.products.back().is_faulty = true;
    actual_shipment.products.back().type = "gear_part";
    actual_shipment.products.back().pose = make_pose(0, 1, 2, 0, 0, 5);

    scorer.NotifyShipmentReceived(shipment_time, "order_0_shipment_0", actual_shipment);
  }
  {
    Time shipment_time(456789, 123);
    DetectedShipment actual_shipment;
    actual_shipment.products.emplace_back();
    actual_shipment.products.back().is_faulty = false;
    actual_shipment.products.back().type = "gear_part";
    actual_shipment.products.back().pose = make_pose(0, 1, 2, 0, 0, 5);
    actual_shipment.products.emplace_back();
    actual_shipment.products.back().is_faulty = false;
    actual_shipment.products.back().type = "piston_part";
    actual_shipment.products.back().pose = make_pose(1, 2.05, 3, 1, 0, 5);
    actual_shipment.products.emplace_back();
    actual_shipment.products.back().is_faulty = false;
    actual_shipment.products.back().type = "gear_part";
    actual_shipment.products.back().pose = make_pose(3, 4, 6, 1, 0, 0);
    actual_shipment.products.emplace_back();
    actual_shipment.products.back().is_faulty = false;
    actual_shipment.products.back().type = "pulley_part";
    actual_shipment.products.back().pose = make_pose(0, 0, 1, 1, 0, 0);

    scorer.NotifyShipmentReceived(shipment_time, "order_0_shipment_1", actual_shipment);
  }

  auto score = scorer.GetGameScore();
  EXPECT_DOUBLE_EQ(
    make_shipment_score(1, 0) + make_shipment_score(3, 2),
    score.total()) << score;
}

TEST(TestAriacScorer, two_orders_perfect)
{
  AriacScorer scorer;

  {
    Order order;
    order.order_id = "order_0";
    order.shipments.emplace_back();
    order.shipments.back().shipment_type = "order_0_shipment_0";
    order.shipments.back().agv_id = "any";
    order.shipments.back().products.emplace_back();
    order.shipments.back().products.back().type = "gear_part";
    order.shipments.back().products.back().pose = make_pose(0, 1, 2, 0, 0, 5);
    Time start_time(123, 456);
    scorer.NotifyOrderStarted(start_time, order);
  }
  {
    Order order;
    order.order_id = "order_1";
    order.shipments.emplace_back();
    order.shipments.back().shipment_type = "order_1_shipment_0";
    order.shipments.back().agv_id = "any";
    order.shipments.back().products.emplace_back();
    order.shipments.back().products.back().type = "pulley_part";
    order.shipments.back().products.back().pose = make_pose(0, 1, 0, 0, 0, 0);
    Time start_time(123456, 456);
    scorer.NotifyOrderStarted(start_time, order);
  }

  {
    Time shipment_time(789, 0);
    DetectedShipment shipment;
    shipment.products.emplace_back();
    shipment.products.back().is_faulty = false;
    shipment.products.back().type = "gear_part";
    shipment.products.back().pose = make_pose(0, 1, 2, 0, 0, 5);
    scorer.NotifyShipmentReceived(shipment_time, "order_0_shipment_0", shipment);
  }
  {
    Time shipment_time(1234567, 0);
    DetectedShipment shipment;
    shipment.products.emplace_back();
    shipment.products.back().is_faulty = false;
    shipment.products.back().type = "pulley_part";
    shipment.products.back().pose = make_pose(0, 1, 0, 0, 0, 0);
    scorer.NotifyShipmentReceived(shipment_time, "order_1_shipment_0", shipment);
  }

  auto score = scorer.GetGameScore();
  EXPECT_DOUBLE_EQ(
    make_shipment_score(1, 1, ALL_PRODUCTS) + HIGH_PRIORITY_FACTOR * make_shipment_score(1, 1, ALL_PRODUCTS),
    score.total()) << score;
  EXPECT_TRUE(score.orderScores["order_0"].isComplete());
  EXPECT_TRUE(score.orderScores["order_1"].isComplete());
  EXPECT_DOUBLE_EQ(
    make_shipment_score(1, 1, ALL_PRODUCTS),
    score.orderScores["order_1"].completion_score());
}

TEST(TestAriacScorer, two_orders_first_ignored)
{
  AriacScorer scorer;

  {
    Order order;
    order.order_id = "order_0";
    order.shipments.emplace_back();
    order.shipments.back().shipment_type = "order_0_shipment_0";
    order.shipments.back().agv_id = "any";
    order.shipments.back().products.emplace_back();
    order.shipments.back().products.back().type = "gear_part";
    order.shipments.back().products.back().pose = make_pose(0, 1, 2, 0, 0, 5);
    Time start_time(123, 456);
    scorer.NotifyOrderStarted(start_time, order);
  }
  {
    Order order;
    order.order_id = "order_1";
    order.shipments.emplace_back();
    order.shipments.back().shipment_type = "order_1_shipment_0";
    order.shipments.back().agv_id = "any";
    order.shipments.back().products.emplace_back();
    order.shipments.back().products.back().type = "pulley_part";
    order.shipments.back().products.back().pose = make_pose(0, 1, 0, 0, 0, 0);
    Time start_time(123456, 456);
    scorer.NotifyOrderStarted(start_time, order);
  }
  {
    Time shipment_time(1234567, 0);
    DetectedShipment shipment;
    shipment.products.emplace_back();
    shipment.products.back().is_faulty = false;
    shipment.products.back().type = "pulley_part";
    shipment.products.back().pose = make_pose(0, 1, 0, 0, 0, 0);
    scorer.NotifyShipmentReceived(shipment_time, "order_1_shipment_0", shipment);
  }

  auto score = scorer.GetGameScore();
  EXPECT_DOUBLE_EQ(
    HIGH_PRIORITY_FACTOR * make_shipment_score(1, 1, ALL_PRODUCTS),
    score.total()) << score;
  EXPECT_FALSE(score.orderScores["order_0"].isComplete());
  EXPECT_FALSE(score.orderScores["order_0"].shipmentScores.begin()->second.isComplete);
  EXPECT_TRUE(score.orderScores["order_1"].isComplete());
  EXPECT_TRUE(score.orderScores["order_1"].shipmentScores.begin()->second.isComplete);
}

TEST(TestAriacScorer, two_orders_second_ignored)
{
  AriacScorer scorer;

  {
    Order order;
    order.order_id = "order_0";
    order.shipments.emplace_back();
    order.shipments.back().shipment_type = "order_0_shipment_0";
    order.shipments.back().agv_id = "any";
    order.shipments.back().products.emplace_back();
    order.shipments.back().products.back().type = "gear_part";
    order.shipments.back().products.back().pose = make_pose(0, 1, 2, 0, 0, 5);
    Time start_time(123, 456);
    scorer.NotifyOrderStarted(start_time, order);
  }
  {
    Order order;
    order.order_id = "order_1";
    order.shipments.emplace_back();
    order.shipments.back().shipment_type = "order_1_shipment_0";
    order.shipments.back().agv_id = "any";
    order.shipments.back().products.emplace_back();
    order.shipments.back().products.back().type = "pulley_part";
    order.shipments.back().products.back().pose = make_pose(0, 1, 0, 0, 0, 0);
    Time start_time(123456, 456);
    scorer.NotifyOrderStarted(start_time, order);
  }

  {
    Time shipment_time(789, 0);
    DetectedShipment shipment;
    shipment.products.emplace_back();
    shipment.products.back().is_faulty = false;
    shipment.products.back().type = "gear_part";
    shipment.products.back().pose = make_pose(0, 1, 2, 0, 0, 5);
    scorer.NotifyShipmentReceived(shipment_time, "order_0_shipment_0", shipment);
  }

  auto score = scorer.GetGameScore();
  EXPECT_DOUBLE_EQ(
    make_shipment_score(1, 1, ALL_PRODUCTS),
    score.total()) << score;
  EXPECT_TRUE(score.orderScores["order_0"].isComplete());
  EXPECT_FALSE(score.orderScores["order_1"].isComplete());
}

TEST(TestAriacScorer, order_fulfilled_but_arm_collision)
{
  AriacScorer scorer;

  Order order;
  order.order_id = "order_0";
  order.shipments.emplace_back();
  order.shipments.back().shipment_type = "order_0_shipment_0";
  order.shipments.back().agv_id = "any";
  order.shipments.back().products.emplace_back();
  order.shipments.back().products.back().type = "gear_part";
  order.shipments.back().products.back().pose = make_pose(0, 1, 2, 0, 0, 5);
  Time start_time(123, 456);

  scorer.NotifyOrderStarted(start_time, order);

  Time shipment_time(789, 123);
  DetectedShipment shipment;
  shipment.products.emplace_back();
  shipment.products.back().is_faulty = false;
  shipment.products.back().type = order.shipments.back().products.back().type;
  shipment.products.back().pose = order.shipments.back().products.back().pose;
  scorer.NotifyShipmentReceived(shipment_time, order.shipments.back().shipment_type, shipment);

  scorer.NotifyArmArmCollision(Time(1234,0));

  auto score = scorer.GetGameScore();
  EXPECT_DOUBLE_EQ(0.0, score.total()) << score;
  EXPECT_DOUBLE_EQ(
    make_shipment_score(1, 1, ALL_PRODUCTS),
    score.orderScores["order_0"].total()) << score;
}

TEST(TestAriacScorer, order_update_old_shipments_ignored_got_some_points)
{
  AriacScorer scorer;

  {
    Order order;
    order.order_id = "order_0";
    order.shipments.emplace_back();
    order.shipments.back().shipment_type = "order_0_shipment_0";
    order.shipments.back().agv_id = "any";
    order.shipments.back().products.emplace_back();
    order.shipments.back().products.back().type = "gear_part";
    order.shipments.back().products.back().pose = make_pose(0, 1, 2, 0, 0, 5);
    Time start_time(123, 456);
    scorer.NotifyOrderStarted(start_time, order);
  }

  // This would have fulfiled the original order perfectly
  {
    Time shipment_time(456, 0);
    DetectedShipment shipment;
    shipment.products.emplace_back();
    shipment.products.back().is_faulty = false;
    shipment.products.back().type = "gear_part";
    shipment.products.back().pose = make_pose(0, 1, 2, 0, 0, 5);
    scorer.NotifyShipmentReceived(shipment_time, "order_0_shipment_0", shipment);
  }
  // This would update the new order perfectly, but it's too soon
  {
    Time shipment_time(789, 0);
    DetectedShipment shipment;
    shipment.products.emplace_back();
    shipment.products.back().is_faulty = false;
    shipment.products.back().type = "pulley_part";
    shipment.products.back().pose = make_pose(0, 1, 0, 0, 0, 0);
    scorer.NotifyShipmentReceived(shipment_time, "order_0_shipment_0", shipment);
  }
  {
    Order order;
    order.order_id = "order_0_update_0";
    order.shipments.emplace_back();
    order.shipments.back().shipment_type = "order_0_shipment_0";
    order.shipments.back().agv_id = "any";
    order.shipments.back().products.emplace_back();
    order.shipments.back().products.back().type = "pulley_part";
    order.shipments.back().products.back().pose = make_pose(0, 1, 0, 0, 0, 0);
    Time start_time(123456, 456);
    scorer.NotifyOrderUpdated(start_time, "order_0", order);
  }
  // THis fulfils the updated order without the product pose point
  {
    Time shipment_time(1234567, 0);
    DetectedShipment shipment;
    shipment.products.emplace_back();
    shipment.products.back().is_faulty = false;
    shipment.products.back().type = "pulley_part";
    shipment.products.back().pose = make_pose(0, 1, 0, 0, 1, 0);
    scorer.NotifyShipmentReceived(shipment_time, "order_0_shipment_0", shipment);
  }

  auto score = scorer.GetGameScore();
  EXPECT_DOUBLE_EQ(
    make_shipment_score(1, 0, ALL_PRODUCTS),
    score.total()) << score;
}

TEST(TestAriacScorer, order_update_old_shipments_ignored_got_no_points)
{
  AriacScorer scorer;

  {
    Order order;
    order.order_id = "order_0";
    order.shipments.emplace_back();
    order.shipments.back().shipment_type = "order_0_shipment_0";
    order.shipments.back().agv_id = "any";
    order.shipments.back().products.emplace_back();
    order.shipments.back().products.back().type = "gear_part";
    order.shipments.back().products.back().pose = make_pose(0, 1, 2, 0, 0, 5);
    Time start_time(123, 456);
    scorer.NotifyOrderStarted(start_time, order);
  }

  // This would have fulfiled the original order perfectly
  {
    Time shipment_time(456, 0);
    DetectedShipment shipment;
    shipment.products.emplace_back();
    shipment.products.back().is_faulty = false;
    shipment.products.back().type = "gear_part";
    shipment.products.back().pose = make_pose(0, 1, 2, 0, 0, 5);
    scorer.NotifyShipmentReceived(shipment_time, "order_0_shipment_0", shipment);
  }
  // This would update the new order perfectly, but it's too soon
  {
    Time shipment_time(789, 0);
    DetectedShipment shipment;
    shipment.products.emplace_back();
    shipment.products.back().is_faulty = false;
    shipment.products.back().type = "pulley_part";
    shipment.products.back().pose = make_pose(0, 1, 0, 0, 0, 0);
    scorer.NotifyShipmentReceived(shipment_time, "order_0_shipment_0", shipment);
  }
  {
    Order order;
    order.order_id = "order_0_update_0";
    order.shipments.emplace_back();
    order.shipments.back().shipment_type = "order_0_shipment_0";
    order.shipments.back().products.emplace_back();
    order.shipments.back().products.back().type = "pulley_part";
    order.shipments.back().products.back().pose = make_pose(0, 1, 0, 0, 0, 0);
    Time start_time(123456, 456);
    scorer.NotifyOrderUpdated(start_time, "order_0", order);
  }
  // THis fulfils the updated order without any points
  {
    Time shipment_time(1234567, 0);
    DetectedShipment shipment;
    scorer.NotifyShipmentReceived(shipment_time, "order_0_shipment_0", shipment);
  }

  auto score = scorer.GetGameScore();
  EXPECT_DOUBLE_EQ(0, score.total()) << score;
}

TEST(TestAriacScorer, order_with_one_shipment_fulfilled_perfectly_then_again_with_nothing)
{
  AriacScorer scorer;

  Order order;
  order.order_id = "order_0";
  order.shipments.emplace_back();
  order.shipments.back().shipment_type = "order_0_shipment_0";
  order.shipments.back().agv_id = "any";
  order.shipments.back().products.emplace_back();
  order.shipments.back().products.back().type = "gear_part";
  order.shipments.back().products.back().pose = make_pose(0, 1, 2, 0, 0, 5);
  Time start_time(123, 456);

  scorer.NotifyOrderStarted(start_time, order);

  // submit perfect shipment
  {
  Time shipment_time(789, 123);
  DetectedShipment shipment;
  shipment.products.emplace_back();
  shipment.products.back().is_faulty = false;
  shipment.products.back().type = order.shipments.back().products.back().type;
  shipment.products.back().pose = order.shipments.back().products.back().pose;
  scorer.NotifyShipmentReceived(shipment_time, order.shipments.back().shipment_type, shipment);
  }
  // Incorrectly submit same shipment again, but with no content
  {
  Time shipment_time(7890, 123);
  DetectedShipment shipment;
  scorer.NotifyShipmentReceived(shipment_time, order.shipments.back().shipment_type, shipment);
  }

  // Only the first shipment score should count
  auto score = scorer.GetGameScore();
  EXPECT_DOUBLE_EQ(make_shipment_score(1, 1, ALL_PRODUCTS), score.total()) << score;
  EXPECT_TRUE(score.orderScores["order_0"].isComplete());
}

TEST(TestAriacScorer, order_with_one_shipment_fulfilled_with_nothing_then_again_perfectly)
{
  AriacScorer scorer;

  Order order;
  order.order_id = "order_0";
  order.shipments.emplace_back();
  order.shipments.back().shipment_type = "order_0_shipment_0";
  order.shipments.back().agv_id = "any";
  order.shipments.back().products.emplace_back();
  order.shipments.back().products.back().type = "gear_part";
  order.shipments.back().products.back().pose = make_pose(0, 1, 2, 0, 0, 5);
  Time start_time(123, 456);

  scorer.NotifyOrderStarted(start_time, order);

  // submit shipment with no content
  {
  Time shipment_time(789, 123);
  DetectedShipment shipment;
  scorer.NotifyShipmentReceived(shipment_time, order.shipments.back().shipment_type, shipment);
  }
  // incorrectly ship same shipment again, but having perfect content
  {
  Time shipment_time(7890, 123);
  DetectedShipment shipment;
  shipment.products.emplace_back();
  shipment.products.back().is_faulty = false;
  shipment.products.back().type = order.shipments.back().products.back().type;
  shipment.products.back().pose = order.shipments.back().products.back().pose;
  scorer.NotifyShipmentReceived(shipment_time, order.shipments.back().shipment_type, shipment);
  }

  // Only the first shipment score should count
  auto score = scorer.GetGameScore();
  EXPECT_DOUBLE_EQ(0, score.total()) << score;
  EXPECT_TRUE(score.orderScores["order_0"].isComplete());
}

std::ostream &operator<<(std::ostream &_out, const std::vector<size_t> &_obj)
{
  _out << "{";
  bool is_first = true;
  for (size_t e : _obj)
  {
    if (is_first)
    {
      is_first = false;
    }
    else
    {
      _out << ", ";
    }
    _out << e;
  }
  _out << "}";
  return _out;
}

TEST(TestAriacScorer, order_with_5_parts_permutated)
{
  // Modeled after ariac_scoring_updated_order.test, without the update
  Order order;
  order.order_id = "order_0";
  order.shipments.emplace_back();
  order.shipments.back().shipment_type = "order_0_shipment_0";
  order.shipments.back().agv_id = "any";
  order.shipments.back().products.emplace_back();
  order.shipments.back().products.back().type = "piston_rod_part";
  order.shipments.back().products.back().pose = make_pose(0.1, -0.2, 0, 0, 0, 0);
  order.shipments.back().products.emplace_back();
  order.shipments.back().products.back().type = "gear_part";
  order.shipments.back().products.back().pose = make_pose(-0.1, -0.2, 0, 0, 0, 0);
  order.shipments.back().products.emplace_back();
  order.shipments.back().products.back().type = "piston_rod_part";
  order.shipments.back().products.back().pose = make_pose(0.15, 0.15, 0, 0, 0, 0);
  order.shipments.back().products.emplace_back();
  order.shipments.back().products.back().type = "gear_part";
  order.shipments.back().products.back().pose = make_pose(-0.15, 0.15, 0, 0, 0, 0);
  order.shipments.back().products.emplace_back();
  order.shipments.back().products.back().type = "gear_part";
  order.shipments.back().products.back().pose = make_pose(0, 0.15, 0, 0, 0, 0);
  Time start_time(123, 456);

  DetectedShipment base_shipment;
  base_shipment.products.emplace_back();
  base_shipment.products.back().is_faulty = false;
  base_shipment.products.back().type = "piston_rod_part";
  base_shipment.products.back().pose = make_pose(
    0.10000017526, -0.19999958198, 0.00435986090436,
    -0.0000713, 0.0027489, 0);
  base_shipment.products.emplace_back();
  base_shipment.products.back().is_faulty = false;
  base_shipment.products.back().type = "piston_rod_part";
  base_shipment.products.back().pose = make_pose(
    -0.0999998347742, -0.200000116097, 0.00438380424118,
    -0.0001645, 0.0008087, 0);
  base_shipment.products.emplace_back();
  base_shipment.products.back().is_faulty = false;
  base_shipment.products.back().type = "gear_part";
  base_shipment.products.back().pose = make_pose(
    0.150000095384, 0.150000596946, 0.00437033950751,
    0.0000845, 0.0002234, 0);
  base_shipment.products.emplace_back();
  base_shipment.products.back().is_faulty = false;
  base_shipment.products.back().type = "gear_part";
  base_shipment.products.back().pose = make_pose(
    -0.149999494027, 0.149999979668, 0.00434921469508,
    0.0003539, -0.0004118, 1e-7);
  base_shipment.products.emplace_back();
  base_shipment.products.back().is_faulty = false;
  base_shipment.products.back().type = "gear_part";
  base_shipment.products.back().pose = make_pose(
    -2.47204166137e-07, 0.15000077682, 0.00433509279306,
    0, 0.00075, -1e-7);

  // For all possible detected part orderings, the score should always be the same
  std::vector<size_t> permutation{0, 1, 2, 3, 4};
  do
  {
    AriacScorer scorer;
    scorer.NotifyOrderStarted(start_time, order);

    Time shipment_time(789, 123);
    DetectedShipment shipment;
    shipment.products.resize(base_shipment.products.size());
    for (size_t i = 0; i < permutation.size(); ++i)
    {
      shipment.products.at(i) = base_shipment.products.at(permutation[i]);
    }
    scorer.NotifyShipmentReceived(shipment_time, order.shipments.back().shipment_type, shipment);

    auto score = scorer.GetGameScore();
    EXPECT_DOUBLE_EQ(make_shipment_score(5, 3, ALL_PRODUCTS), score.total()) << score << permutation;
  } while (std::next_permutation(permutation.begin(), permutation.end()));
}

// Run all the tests that were declared with TEST()
int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
