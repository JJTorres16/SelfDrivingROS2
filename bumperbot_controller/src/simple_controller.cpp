#include "bumperbot_controller/simple_controller.hpp"

SimpleControllerNode::SimpleControllerNode(const std::string & name) : Node(name){

    declare_parameter<double>("wheel_radius", 0.33);
    declare_parameter<double>("wheel_separation", 0.17);

    wheel_radius_ = get_parameter("wheel_radius").as_double();
    wheel_separation_ = get_parameter("wheel_separation").as_double();

    RCLCPP_INFO_STREAM(get_logger(), "Using wheel radius: " <<wheel_radius_);
    RCLCPP_INFO_STREAM(get_logger(), "Using wheel separation: " <<wheel_separation_);

    wheel_command_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>("/simple_velocity_controller/commands", 10);
    velocity_sub_ = create_subscription<geometry_msgs::msg::TwistStamped>("")
    

}