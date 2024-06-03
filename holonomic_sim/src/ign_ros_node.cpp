/**
 * @file ign_ros_node.cpp
 * @brief ROS 2 node for holonomic simulation
 * @author kousei
 * @date 2024-05-29
*/
#include <rclcpp/rclcpp.hpp>

#include <holonomic_sim/robot_vel.hpp>

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::executors::MultiThreadedExecutor exec;
  const auto robot_vel =
      std::make_shared<holonomic_sim::RobotVel>(rclcpp::NodeOptions());
  exec.add_node(robot_vel);
  exec.spin();
  rclcpp::shutdown();
}