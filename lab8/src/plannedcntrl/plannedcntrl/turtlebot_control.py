#!/usr/bin/env python3

import math
import rclpy
from rclpy.node import Node
import tf2_ros
import numpy as np
import transforms3d.euler as euler
from geometry_msgs.msg import TransformStamped, PoseStamped, Twist, PointStamped
from tf2_geometry_msgs.tf2_geometry_msgs import do_transform_pose
from plannedcntrl.trajectory import plan_curved_trajectory  # Your existing Bezier planner
import time

class TurtleBotController(Node):
    def __init__(self):
        super().__init__('turtlebot_controller')

        # Publisher and TF setup
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Controller gains
        self.Kp = np.diag([2.0, 0.8])
        self.Kd = np.diag([-0.5, 0.5])
        self.Ki = np.diag([-0.1, 0.1])

        # Subscriber
        self.create_subscription(PointStamped, '/goal_point', self.planning_callback, 10)

        self.get_logger().info('TurtleBot controller node initialized.')

    # ------------------------------------------------------------------
    # Main waypoint controller
    # ------------------------------------------------------------------
    def controller(self, waypoint):
        while rclpy.ok():
            # TODO: Transform the waypoint from the odom/world frame into the robot's base_link frame 
            # before computing errors — you'll need this so x_err and yaw_err are in the robot's coordinate system.
            try:
                trans = tf_buffer.lookup_transform('odom', 'base_footprint', rclpy.time.Time()) ## TODO: Apply a lookup transform between our world frame and turtlebot frame
                # trans = T_world_turtlebot
                break
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                node.get_logger().warn('TF lookup failed, retrying...')
                rclpy.spin_once(node, timeout_sec=0.1)
            
            # NOTE: USE do_transform_pose AND FIX THIS
            waypoint_x_world, waypoint_y_world, waypoint_theta = waypoint # just parsing the waypoint variable
            waypoint_baselink = np.array([waypoint_x_world - trans.transform.translation.x, waypoint_y_world -  trans.transform.translation.y])

            #TODO: Calculate x and yaw errors! 
            x_err = waypoint_baselink[0]
            yaw_err = TurtleBotController._quat_from_yawtrans.transform.rotation

            if abs(x_err) < 0.03 and abs(yaw_err) < 0.2:
                self.get_logger().info("Waypoint reached, moving to next.")
                return

            time.sleep(0.1)

    # ------------------------------------------------------------------
    # Callback when goal point is published
    # ------------------------------------------------------------------
    def planning_callback(self, msg: PointStamped):
        trajectory = plan_curved_trajectory((msg.point.x, msg.point.y)) 
        # 'trajectory' is length=num_points list of length3 tuples (x, y, change_theta) in world coordinates

        for waypoint in trajectory:
            self.controller(waypoint)

    # ------------------------------------------------------------------
    # Utility
    # ------------------------------------------------------------------
    @staticmethod
    def _quat_from_yaw(yaw):
        """Return quaternion (x, y, z, w) from yaw angle."""
        return [0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0)]


# ----------------------------------------------------------------------
# Main
# ----------------------------------------------------------------------
def main(args=None):
    rclpy.init(args=args)
    node = TurtleBotController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
