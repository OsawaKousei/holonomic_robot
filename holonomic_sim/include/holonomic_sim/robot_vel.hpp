/**
 * @file robot_vel.hpp
 * @brief Robot velocity publisher for holonomic simulation
 * @author kousei
 * @date 2024-05-29
*/
#ifndef HOLONOMIC_SIM__ROBOT_VEL_HPP_
#define HOLONOMIC_SIM__ROBOT_VEL_HPP_

#include "holonomic_sim/visibility_control.h"
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>

namespace holonomic_sim {

class RobotVel : public rclcpp::Node {
public:
  TUTORIAL_PUBLIC
  explicit RobotVel(const rclcpp::NodeOptions &options);

  virtual ~RobotVel();

private:
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;

  nav_msgs::msg::Odometry prev_odom;
  double prev_yaw;
  geometry_msgs::msg::Twist robot_vel;
};

} // namespace holonomic_sim

#endif // HOLONOMIC_SIM__ROBOT_VEL_HPP_