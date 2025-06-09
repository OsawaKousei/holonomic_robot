/**
 * @file teleop.hpp
 * @brief Teleoperation node for holonomic robot
 * @author kousei
 * @date 2025-06-02
 */
#ifndef HOLONOMIC_SIM__TELEOP_HPP_
#define HOLONOMIC_SIM__TELEOP_HPP_

#include "holonomic_sim/visibility_control.h"
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <rclcpp/rclcpp.hpp>

namespace holonomic_sim
{

    class Teleop : public rclcpp::Node
    {
    public:
        TUTORIAL_PUBLIC
        explicit Teleop(const rclcpp::NodeOptions &options);

        virtual ~Teleop();

    private:
        rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_subscription_;
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher_;

        // Scale factors for joystick axes
        double linear_x_scale_;
        double linear_y_scale_;
        double angular_scale_;

        // Joystick axis mapping
        int axis_linear_x_;
        int axis_linear_y_;
        int axis_angular_;

        // Deadzone threshold
        double deadzone_;
    };

} // namespace holonomic_sim

#endif // HOLONOMIC_SIM__TELEOP_HPP_
