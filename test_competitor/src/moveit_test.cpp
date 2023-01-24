#include <test_competitor/test_competitor.hpp>

#include <rclcpp/rclcpp.hpp>

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto test_competitor = std::make_shared<TestCompetitor>();

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(test_competitor);
  std::thread([&executor]() { executor.spin(); }).detach();

  // Start Competition
  test_competitor->StartCompetition();

  // Send Robots to home position
  test_competitor->FloorRobotSendHome();
  test_competitor->CeilingRobotSendHome();

  // Shutdown ROS
  rclcpp::shutdown();
  return 0;
}