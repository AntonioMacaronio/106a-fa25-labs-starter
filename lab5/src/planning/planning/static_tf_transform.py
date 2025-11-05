#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from tf2_ros import StaticTransformBroadcaster

import numpy as np

class ConstantTransformPublisher(Node):
    def __init__(self):
        super().__init__('constant_tf_publisher')
        self.br = StaticTransformBroadcaster(self)

        # Homogeneous transform G_ar->base
        G = np.array([
            [-1, 0, 0, 0.0],
            [ 0, 0, 1, 0.16],
            [ 0, 1, 0, -0.13],
            [ 0, 0, 0, 1.0]
        ])
        # The [x,y,z,w] quaternions for this are [0, 0.7071068, 0.7071068, 0]

        # Create TransformStamped
        self.transform = TransformStamped()
        # ---------------------------
        # TODO: Fill out TransformStamped message
        # --------------------------
        self.transform.header.frame_id = "ar_marker_8" # to find this frame, launch ros2_aruco and then add the tf transformation. Make sure aruco tag is visible!
        self.transform.child_frame_id = "base_link"
        
        self.transform.transform.translation.x = 0.0
        self.transform.transform.translation.y = 0.16
        self.transform.transform.translation.z = -0.13
        
        self.transform.transform.rotation.x = 0.0
        self.transform.transform.rotation.y = 0.7071068
        self.transform.transform.rotation.z = 0.7071068
        self.transform.transform.rotation.w = 0.0

        self.timer = self.create_timer(0.05, self.broadcast_tf)

    def broadcast_tf(self):
        self.transform.header.stamp = self.get_clock().now().to_msg()
        self.br.sendTransform(self.transform)

def main():
    rclpy.init()
    node = ConstantTransformPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()