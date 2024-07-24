import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose  
import math

class SimpleTurtlesimKinematics(Node):
    def __init__(self):
        super().__init__("simple_turtlesim_kinematics")

        self.turtle1_pos_sub_ = self.create_subscription(Pose, "/turtle1/pose", self.turtle1PoseCallback, 10)
        self.turtle2_pos_sub_ = self.create_subscription(Pose, "/turtle2/pose", self.turtle2PoseCallback, 10)

        self.last_turtle1_pose_ = Pose()
        self.last_turtle2_pose_ = Pose()

    def turtle1PoseCallback(self, msg):
        self.last_turtle1_pose_ = msg

    def turtle2PoseCallback(self, msg):
        self.last_turtle2_pose_ = msg

        Tx = self.last_turtle2_pose_.x - self.last_turtle1_pose_.x
        Ty = self.last_turtle2_pose_.y - self.last_turtle1_pose_.y

        # Calculation of the rotation matrix
        # 1.- Calcultate the relative orientation
        theta_rad = self.last_turtle2_pose_.theta - self.last_turtle1_pose_.theta
        theta_deg = 180 * theta_rad / 3.1416

        self.get_logger().info("""\n 
                               Translation Vector turtle1 -> turtle2 \n
                               Tx: %f \n
                               Ty: %f \n 
                               Rotation Matrix turtle1 -> turtle2 \n
                               theta(rad): %f
                               theta(deg): %f \n
                               |R11      R12| : |%f       %f|\n
                               |R21      R22| : |%f       %f|\n""" % (Tx, Ty, theta_rad, theta_deg, math.cos(theta_rad), -math.sin(theta_rad), 
                                                                      math.sin(theta_rad), math.cos(theta_rad)))
        
def main(args=None):
    rclpy.init()
    node = SimpleTurtlesimKinematics()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    

if __name__ == "__main__":
    main()