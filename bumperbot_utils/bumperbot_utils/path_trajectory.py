#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry, Path

class PathTrajectoryNode(Node):
    def __init__(self):
        super().__init__("path_trajectory")

        # Suscriber to bumperbot/odom
        self.bumperbot_odom_sub_ = self.create_subscription(Odometry, "bumperbot_controller/odom", self.pathTrajectoryCallback, 10)
        self.bumperbot_trajectory_pub_ = self.create_publisher(Path, "bumperbot_controller/trajectory ", 10)

        

def main(args=None):
    rclpy.init()
    node = PathTrajectoryNode()
    rclpy.spin(node)
    node.destroy_node
    rclpy.shutdown

if __name__ == "__main__":
    main()