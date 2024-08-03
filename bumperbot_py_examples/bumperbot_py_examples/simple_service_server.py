#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from bumperbot_msgs.srv import AddTwoInts

class SimpleServiceServerNode(Node):
    def __init__(self):
        super().__init__("simple_server")

        self.service_ = self.create_service(AddTwoInts, "add_two_ints", self.serviceCallback)

        self.get_logger().info("Service add_two_ints ready")

    def serviceCallback(self, request, response):
        self.get_logger().info("New requeste recive a: %d, b: %d" %(request.a, request.b))
        response.sum = request.a + request.b
        self.get_logger().info("Returning sum: %d" % response.sum)

        return response
    
def main(args=None):
    rclpy.init()
    node = SimpleServiceServerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()