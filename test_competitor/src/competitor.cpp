#include <test_competitor/test_competitor.hpp>

#include <rclcpp/rclcpp.hpp>

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);

  auto test_competitor = std::make_shared<TestCompetitor>();

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(test_competitor);
  std::thread([&executor]() { executor.spin(); }).detach();

  // Start Competition
  test_competitor->StartCompetition();

  // Move Robots to Home Poses
  test_competitor->FloorRobotSendHome();
  test_competitor->CeilingRobotSendHome();

  // Complete Orders
  test_competitor->CompleteOrders();

  test_competitor->EndCompetition();

  rclcpp::shutdown();
}