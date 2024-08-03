#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import SetParametersResult
from rclpy.parameter import Parameter

class SimpleParameterNode(Node):
    def __init__(self):
        super().__init__("simple_parameter")

        self.declare_parameter("simple_int_param", 16)
        self.declare_parameter("simple_string_param", 'Julian')

        self.add_on_set_parameters_callback(self.param_change_callback)

    def param_change_callback(self, params):
        result = SetParametersResult()

        for param in params:
            if param.name == "simple_int_param" and param.type_ == Parameter.Type.INTEGER:
                self.get_logger().info("Parameter simple_int_param changed! New value: %d" % param.value)
                result.successful = True
                
            if param.name == "simple_string_param" and param.type_ == Parameter.Type.STRING:
                self.get_logger().info("Parameter simple_string_param cnaged! New Value: %s" % param.value)
                result.successful = True

        return result
    
def main(args=None):
     rclpy.init()
     node = SimpleParameterNode()
     rclpy.spin(node)
     node.destroy_node()
     rclpy.shutdown()

if __name__ == "__main__":
    main()
