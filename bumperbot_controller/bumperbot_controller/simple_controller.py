#!/usr/bin/env python3

import rclpy
import numpy as np
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
from geometry_msgs.msg import TwistStamped, TransformStamped
from rclpy.time import Time
from rclpy.constants import S_TO_NS
from nav_msgs.msg import Odometry
import math
from tf_transformations import quaternion_from_euler
from tf2_ros import TransformBroadcaster

class SimpleControllerNode(Node):
    def __init__(self):
        super().__init__("simple_controller")

        self.declare_parameter("wheel_radius", 0.033)
        self.declare_parameter("wheel_separation", 0.17)

        self.wheel_radius_ = self.get_parameter("wheel_radius").get_parameter_value().double_value
        self.wheel_separation_ = self.get_parameter("wheel_separation").get_parameter_value().double_value

        self.get_logger().info("Wheel raiduis: %f" % self.wheel_radius_)
        self.get_logger().info("Wheel separation: %f" % self.wheel_separation_)

        self.left_wheel_prev_pos_ = 0.0
        self.right_wheel_prev_pos_ = 0.0
        self.prev_time_ = self.get_clock().now()

        self.x_ = 0.0 # The current value of the x coordinates of the robot. This variable will update any time that recibes a mesegge from joint_states
        self.y_ = 0.0 # The current value of the y coordinates of the robot. This variable will update any time that recibes a mesegge from joint_states
        self.theta_ = 0.0

        self.wheel_command_pub_ = self.create_publisher(Float64MultiArray, "/simple_velocity_controller/commands", 10)
        self.velocity_sub_ = self.create_subscription(TwistStamped, "bumperbot_controller/cmd_vel", self.velCallback, 10)
        self.joint_sub_ = self.create_subscription(JointState, "joint_states", self.jointCallback, 10) # What this topic does?
        self.odom_pub_ = self.create_publisher(Odometry, "bumperbot_controller/odom", 10)

        self.speed_conversion_ = np.array([[self.wheel_radius_/2, self.wheel_radius_/2], 
                                           [self.wheel_radius_/self.wheel_separation_, -self.wheel_radius_/self.wheel_separation_]])
        
        self.odom_msg_ = Odometry()
        self.odom_msg_.header.frame_id = "odom" # The name of the frame that robot use to express it movement (reference)
        self.odom_msg_.child_frame_id = "base_footprint"
        self.odom_msg_.pose.pose.orientation.x = 0.0
        self.odom_msg_.pose.pose.orientation.y = 0.0
        self.odom_msg_.pose.pose.orientation.z = 0.0
        self.odom_msg_.pose.pose.orientation.w = 1.0

        self.broadcaster_ = TransformBroadcaster(self)
        self.transform_stamped_ = TransformStamped()
        self.transform_stamped_.header.frame_id = "odom"
        self.transform_stamped_.child_frame_id = "base_footprint"
        
        self.get_logger().info("The convertion matrix is: %s" % self.speed_conversion_)

    def velCallback(self, msg):
        # Calculate the velocity of the wheels from the linear velocity
        robot_speed = np.array([[msg.twist.linear.x],
                                [msg.twist.angular.z]]) # Now, we have in invert the speed_conversion_ matrix in order to get the angular velocity
                                                       # of the wheels

        wheel_speed = np.matmul(np.linalg.inv(self.speed_conversion_), robot_speed) # In this array are the angular velocities of the two wheels
                                                                                    # np.matmul to multiply arrays

        wheel_speed_msg = Float64MultiArray()
        wheel_speed_msg.data = [wheel_speed[1, 0], wheel_speed[0, 0]]
        self.wheel_command_pub_.publish(wheel_speed_msg)

    def jointCallback(self, msg):
        dp_left =  msg.position[1] - self.left_wheel_prev_pos_ # msg.position[1] -> Postition of the left wheel and the current moment
        dp_right =  msg.position[0] - self.right_wheel_prev_pos_ # msg.position[0] -> Postition of the right wheel and the current moment
        dt = Time.from_msg(msg.header.stamp) - self.prev_time_ # msg.header.stamp -> current time. Time() to convert in a time object 

        self.left_wheel_prev_pos_ = msg.position[1]
        self.right_wheel_prev_pos_ = msg.position[0]
        self.prev_time_ = Time.from_msg(msg.header.stamp)

        fi_left = dp_left / (dt.nanoseconds / S_TO_NS) # Angular velocity of the left wheel
        fi_right = dp_right / (dt.nanoseconds / S_TO_NS) # Angular velocity of the right wheel

        # Calculate the linear and angular velocity of the robot
        linear = (self.wheel_radius_ * fi_right + self.wheel_radius_ * fi_left)/2
        angular = (self.wheel_radius_ * fi_right - self.wheel_radius_ * fi_left)/self.wheel_separation_

        # Position incremente of the robot:
        d_pos = (self.wheel_radius_ * dp_right + self.wheel_radius_ * dp_left) / 2
        d_theta = (self.wheel_radius_ * dp_right - self.wheel_radius_ * dp_left) / self.wheel_separation_
        self.theta_ += d_theta
        self.x_ += d_pos * math.cos(self.theta_)
        self.y_ += d_pos * math.sin(self.theta_)

        # Transform from euler angles to quaternion angles
        quaternion = quaternion_from_euler(0, 0, self.theta_)
        # Orientation
        self.odom_msg_.pose.pose.orientation.x = quaternion[0]
        self.odom_msg_.pose.pose.orientation.y = quaternion[1]
        self.odom_msg_.pose.pose.orientation.z = quaternion[2]
        self.odom_msg_.pose.pose.orientation.w = quaternion[3]
        self.odom_msg_.header.stamp = self.get_clock().now().to_msg()
        # Position
        self.odom_msg_.pose.pose.position.x = self.x_
        self.odom_msg_.pose.pose.position.y = self.y_
        self.odom_msg_.twist.twist.linear.x = linear
        self.odom_msg_.twist.twist.angular.z = angular

        # Translation vector
        self.transform_stamped_.transform.translation.x = self.x_
        self.transform_stamped_.transform.translation.y = self.y_
        # Orientation vector
        self.transform_stamped_.transform.rotation.x = quaternion[0]
        self.transform_stamped_.transform.rotation.y = quaternion[1]
        self.transform_stamped_.transform.rotation.z = quaternion[2]
        self.transform_stamped_.transform.rotation.w = quaternion[3]
        # TimeStamped in order to know when this message was generated
        self.transform_stamped_.header.stamp = self.get_clock().now().to_msg()

        self.odom_pub_.publish(self.odom_msg_)
        self.broadcaster_.sendTransform(self.transform_stamped_)

def main(args=None):
    rclpy.init()
    node = SimpleControllerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()

