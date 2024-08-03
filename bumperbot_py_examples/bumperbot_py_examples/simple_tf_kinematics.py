#!/usr/bin/env python

import rclpy
from rclpy.node import Node
import rclpy.time
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from tf2_ros import TransformBroadcaster, TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from geometry_msgs.msg import TransformStamped
from bumperbot_msgs.srv import GetTransform
from tf_transformations import quaternion_from_euler, quaternion_multiply, quaternion_inverse

class SimpleTfKInematicsNode(Node):
    def __init__(self):
        super().__init__("simple_tf_kinematics")

        self.static_tf_broadcaster_ = StaticTransformBroadcaster(self)
        self.dynamic_tf_broadcaster_ = TransformBroadcaster(self)

        # Contain the id's, and the vectors of translation and rotation between the two selected frames
        self.static_transform_stamped_ = TransformStamped() 
        self.dynamic_transform_stamped_ = TransformStamped()

        # Indicates the increment to add to the Transform in any iteration
        self.x_increment_ = 0.05
        # The last value on the X direction
        self.last_x_ = 0.0

        # Variables
        self.rotation_counter_ = 0 # The number of rotation that the bumperbot_bse is doing with the respect to the odom frame
        # The last known orientation of the rotating frame. Its in quaternions.
        # First, express them as euler angles, and then transform them to quaternions using the python Library called: TF Transformation
        self.last_orientation_ =  quaternion_from_euler(0, 0, 0) # The three rotation angles
        self.orientation_increment_ = quaternion_from_euler(0, 0, 0.05) # Rotate 0.05 radians in z axis

        self.tf_bufer_ = Buffer()
        self.tf_listener_ = TransformListener(self.tf_bufer_, self)

        self.static_transform_stamped_.header.stamp = self.get_clock().now().to_msg()
        self.static_transform_stamped_.header.frame_id = "bumperbot_base"
        self.static_transform_stamped_.child_frame_id = "bumperbot_top"
        # Translation vector
        self.static_transform_stamped_.transform.translation.x = 0.0
        self.static_transform_stamped_.transform.translation.y = 0.0
        self.static_transform_stamped_.transform.translation.z = 0.3
        # Rotation matrix (using quaternions)
        self.static_transform_stamped_.transform.rotation.x = 0.0
        self.static_transform_stamped_.transform.rotation.y = 0.0
        self.static_transform_stamped_.transform.rotation.z = 0.0
        self.static_transform_stamped_.transform.rotation.w = 1.0

        self.get_logger().info("Publishing sttaic transform between %s and %s" %(self.static_transform_stamped_.header.frame_id,
                                                                                 self.static_transform_stamped_.child_frame_id))
        
        self.timer_ = self.create_timer(0.1, self.timer_callback)

        self.get_transform_srv_ = self.create_service(GetTransform, "get_transform", self.getTransformCallback)
        
    def timer_callback(self):
        # Fill the dynamic_tf_broadcaster with the information of the transformation
        self.dynamic_transform_stamped_.header.stamp = self.get_clock().now().to_msg()
        self.dynamic_transform_stamped_.header.frame_id = "odom"
        self.dynamic_transform_stamped_.child_frame_id = "bumperbot_base"
        # Transformation matrix
        self.dynamic_transform_stamped_.transform.translation.x = self.last_x_ + self.x_increment_
        self.dynamic_transform_stamped_.transform.translation.y = 0.0
        self.dynamic_transform_stamped_.transform.translation.z = 0.0 
        # Rotation matrix (using quaternions)
        quaternion_rot = quaternion_multiply(self.last_orientation_, self.orientation_increment_) 
        self.dynamic_transform_stamped_.transform.rotation.x = quaternion_rot[0]
        self.dynamic_transform_stamped_.transform.rotation.y = quaternion_rot[1]
        self.dynamic_transform_stamped_.transform.rotation.z = quaternion_rot[2]
        self.dynamic_transform_stamped_.transform.rotation.w = quaternion_rot[3] # bumperbot_base and bumperbot_top oriented in the same way

        self.dynamic_tf_broadcaster_.sendTransform(self.dynamic_transform_stamped_)

        # Update the value of the last_x_ variable
        self.last_x_ = self.dynamic_transform_stamped_.transform.translation.x

        # use static_tf_broadcaster to publish the transform in ROS2
        self.static_tf_broadcaster_.sendTransform(self.static_transform_stamped_)
        self.rotation_counter_ += 1
        self.last_orientation_ = quaternion_rot

        if self.rotation_counter_ >= 100:
            self.orientation_increment_ = quaternion_inverse(self.orientation_increment_)
            self.rotation_counter_ = 0

    def getTransformCallback(self, request, response):  
        self.get_logger().info("Requested tranform between %s and %s" % (request.frame_id, request.child_frame_id))   
        requested_transform = TransformStamped() # New transform Stamped variable

        try:
            # Look up the transform between the two frames and know the current transformation matrix
            requested_transform = self.tf_bufer_.lookup_transform(request.frame_id, request.child_frame_id, rclpy.time.Time()) 
        except TransformException as e:
            self.get_logger().error("An error ocurred while transforming %s and %s" % (request.frame_id, request.child_frame_id))
            response.success = False
            return response
        
        # IF there is an transform between the two frames:
        response.transform = requested_transform
        response.success = True
        return response 
                                                                



def main(args=None):
    rclpy.init()
    node = SimpleTfKInematicsNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown

if __name__ == "__main__":
    main()

        
