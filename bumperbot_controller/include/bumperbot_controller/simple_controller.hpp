#ifndef SIMPLE_CONTROLLER_HPP
#define SIMPLE_CONTROLLER_HPP

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <Eigen/Core>

class SimpleControllerNode : public rclcpp::Node {
    public:
        SimpleControllerNode(const std::string & name);

    private:
        void velCallback(const geometry_msgs::msg::TwistStamped & msg);

        rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr velocity_sub_;
        rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr wheel_command_pub_;

        double wheel_radius_;
        double wheel_separation_;
        Eigen::Matrix2d speed_conversion_;
};

#endif