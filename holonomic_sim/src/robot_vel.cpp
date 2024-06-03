/**
 * @file robot_vel.cpp
 * @brief Robot velocity publisher for holonomic simulation
 * @author kousei
 * @date 2024-05-29
*/
#include "holonomic_sim/robot_vel.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp" //avoid the error "undefined reference to 'tf2::fromMsg(geometry_msgs::msg::Quaternion_<std::allocator<void> > const&, tf2::Quaternion&)'"
#include <functional>
#include <memory>
#include <rclcpp_components/register_node_macro.hpp>
#include <tf2/utils.h> //getEulerYPR

using namespace std::chrono_literals;

namespace holonomic_sim {

RobotVel::RobotVel(const rclcpp::NodeOptions &options)
    : rclcpp::Node("robot_vel", options) {
  publisher_ =
      this->create_publisher<geometry_msgs::msg::Twist>("robot_vel", 10);

  auto publish_msg_callback = [this]() -> void {
    auto message = geometry_msgs::msg::Twist();
    message = robot_vel;
    this->publisher_->publish(message);
  };

  timer_ = this->create_wall_timer(500ms, publish_msg_callback);

  auto topic_callback = [this](const nav_msgs::msg::Odometry &msg) -> void {
    // calculate the difference in robot coordinates
    tf2::Vector3 diff_v;
    diff_v.setX(msg.pose.pose.position.x - prev_odom.pose.pose.position.x);
    diff_v.setY(msg.pose.pose.position.y - prev_odom.pose.pose.position.y);
    double yaw, pitch, roll;
    tf2::getEulerYPR(msg.pose.pose.orientation, yaw, pitch, roll);
    tf2::Matrix3x3 rotation_matrix;
    rotation_matrix.setEulerYPR(-(yaw + prev_yaw) / 2.0, 0,
                                0); // use the average to reduce error
    diff_v = rotation_matrix * diff_v;

    // calculate the velocity
    double time_diff = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9 -
                       prev_odom.header.stamp.sec -
                       prev_odom.header.stamp.nanosec * 1e-9;
    robot_vel.linear.x = diff_v.x() / (time_diff);
    robot_vel.linear.y = diff_v.y() / (time_diff);
    robot_vel.angular.z = (yaw - prev_yaw) / (time_diff);

    // show the velocity if the robot is moving
    if (robot_vel.linear.x != 0 || robot_vel.linear.y != 0 ||
        robot_vel.angular.z != 0) {
      RCLCPP_INFO(this->get_logger(), "Robot velocity: x=%f, y=%f, z=%f",
                  robot_vel.linear.x, robot_vel.linear.y, robot_vel.angular.z);
    }

    // set the previous position, orientation and time
    prev_odom = msg;
    prev_yaw = yaw;
  };

  subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "odom", 10, topic_callback);
}

// デストラクタ
RobotVel::~RobotVel() {}

} // namespace holonomic_sim

RCLCPP_COMPONENTS_REGISTER_NODE(holonomic_sim::RobotVel)