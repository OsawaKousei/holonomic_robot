/**
 * @file teleop.cpp
 * @brief Teleoperation node for holonomic robot
 * @author kousei
 * @date 2025-06-02
 */
#include "holonomic_sim/teleop.hpp"
#include <functional>
#include <memory>
#include <rclcpp_components/register_node_macro.hpp>
#include <cmath>

namespace holonomic_sim
{

    Teleop::Teleop(const rclcpp::NodeOptions &options)
        : rclcpp::Node("teleop", options)
    {
        // パラメータの宣言とデフォルト値の設定
        this->declare_parameter("linear_x_scale", 1.0);
        this->declare_parameter("linear_y_scale", 1.0);
        this->declare_parameter("angular_scale", 1.0);
        this->declare_parameter("axis_linear_x", 1);
        this->declare_parameter("axis_linear_y", 0);
        this->declare_parameter("axis_angular", 2);
        this->declare_parameter("deadzone", 0.1);

        // パラメータの取得
        linear_x_scale_ = this->get_parameter("linear_x_scale").as_double();
        linear_y_scale_ = this->get_parameter("linear_y_scale").as_double();
        angular_scale_ = this->get_parameter("angular_scale").as_double();
        axis_linear_x_ = this->get_parameter("axis_linear_x").as_int();
        axis_linear_y_ = this->get_parameter("axis_linear_y").as_int();
        axis_angular_ = this->get_parameter("axis_angular").as_int();
        deadzone_ = this->get_parameter("deadzone").as_double();

        // cmd_vel パブリッシャーの作成
        cmd_vel_publisher_ =
            this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

        // ジョイスティック入力のコールバック関数
        auto joy_callback = [this](const sensor_msgs::msg::Joy::SharedPtr joy_msg) -> void
        {
            geometry_msgs::msg::Twist cmd_vel;

            // 各軸の値を取得し、デッドゾーンを適用
            double linear_x = std::abs(joy_msg->axes[axis_linear_x_]) > deadzone_ ? joy_msg->axes[axis_linear_x_] : 0.0;
            double linear_y = std::abs(joy_msg->axes[axis_linear_y_]) > deadzone_ ? joy_msg->axes[axis_linear_y_] : 0.0;
            double angular = std::abs(joy_msg->axes[axis_angular_]) > deadzone_ ? joy_msg->axes[axis_angular_] : 0.0;

            // スケーリングを適用
            cmd_vel.linear.x = linear_x * linear_x_scale_;
            cmd_vel.linear.y = linear_y * linear_y_scale_;
            cmd_vel.angular.z = angular * angular_scale_;

            // 速度コマンドをパブリッシュ
            cmd_vel_publisher_->publish(cmd_vel);

            // 入力がある場合のみログ出力
            if (cmd_vel.linear.x != 0.0 || cmd_vel.linear.y != 0.0 || cmd_vel.angular.z != 0.0)
            {
                RCLCPP_INFO(this->get_logger(), "Publishing cmd_vel: x=%f, y=%f, angular=%f",
                            cmd_vel.linear.x, cmd_vel.linear.y, cmd_vel.angular.z);
            }
        };

        // ジョイスティック入力のサブスクリプション
        joy_subscription_ = this->create_subscription<sensor_msgs::msg::Joy>(
            "joy", 10, joy_callback);

        RCLCPP_INFO(this->get_logger(), "Teleop node initialized");
    }

    Teleop::~Teleop() {}

} // namespace holonomic_sim

RCLCPP_COMPONENTS_REGISTER_NODE(holonomic_sim::Teleop)
