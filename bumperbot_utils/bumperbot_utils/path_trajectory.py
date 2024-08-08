#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped

class DrawTrajectoryNode(Node):
    def __init__(self):
        super().__init__("draw_trajectory")

        # Suscriber to bumperbot/odom
        self.bumperbot_odom_sub_ = self.create_subscription(Odometry, "bumperbot_controller/odom", self.pathTrajectoryCallback, 10)
        self.bumperbot_trajectory_pub_ = self.create_publisher(Path, "bumperbot_controller/trajectory", 10)
        self.trajectory_ = Path()

    def pathTrajectoryCallback(self, msg: Odometry):
        curent_position = PoseStamped()

        self.trajectory_.header.frame_id = msg.header.frame_id
        curent_position.header.frame_id = msg.header.frame_id # Para este nuevo objeto, usamos el frame del robo 
        curent_position.header.stamp = msg.header.stamp # indicates a specific point in time, relative to a clock's 0 point.
        curent_position.pose = msg.pose.pose # A representation of pose in free space, composed of position and orientation (quaternions)
        self.trajectory_.poses.append(curent_position)
        
        self.bumperbot_trajectory_pub_.publish(self.trajectory_)


def main(args=None):
    rclpy.init()
    node = DrawTrajectoryNode()
    rclpy.spin(node)
    node.destroy_node
    rclpy.shutdown

if __name__ == "__main__":
    main()