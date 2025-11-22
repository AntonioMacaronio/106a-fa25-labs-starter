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
        self.Kp = np.diag([5.0, 0.5])
        self.Ki = np.diag([0.0, 0.0])
        self.Kd = np.diag([-0.5, -0.07])

        # Subscriber
        self.create_subscription(PointStamped, '/goal_point', self.planning_callback, 10)

        self.get_logger().info('TurtleBot controller node initialized.')

    # ------------------------------------------------------------------
    # Main waypoint controller
    # ------------------------------------------------------------------
    def controller(self, waypoint):
        x_i_err, x_d_err, yaw_r_err, yaw_d_err = 0, 0, 0 ,0
        prev_x_dest_robotframe, prev_y_dest_robotframe = None, None # note: dest = destination (our cone coords)

        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)
            # TODO: Transform the waypoint from the odom/world frame into the robot's base_link frame 
            # before computing errors — you'll need this so x_err and yaw_err are in the robot's coordinate system.
            try:
                trans = self.tf_buffer.lookup_transform('base_link', 'odom', rclpy.time.Time()).transform ## TODO: Apply a lookup transform between our world frame and turtlebot frame
                # trans = T_world_turtlebot
                break
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                node.get_logger().warn('TF lookup failed, retrying...')
                rclpy.spin_once(node, timeout_sec=0.1)
            
            x1 = trans.translation.x
            y1 = trans.translation.y
            q = trans.rotation
            yaw = euler.quat2euler([q.w, q.x, q.y, q.z])[2]
            
            # TODO: Calculate x and yaw errors! # yaw error = y coordinate of waypoint
            x_dest_robotframe = x1 + waypoint[0] * np.cos(yaw) - waypoint[1] * np.sin(yaw)
            y_dest_robotframe = y1 + waypoint[0] * np.sin(yaw) + waypoint[1] * np.cos(yaw)

            if abs(x_dest_robotframe) < 0.03 and abs(y_dest_robotframe) < 0.2:
                self.get_logger().info("Waypoint reached, moving to next.")
                return
            
            if prev_x_err is not None and prev_yaw_err is not None:
                x_diff = x_dest_robotframe - prev_x_dest_robotframe
                y_diff = y_dest_robotframe - prev_x_dest_robotframe

            x_i_err += x_diff
            y_i_err += y_diff

            control_cmd = Twist()
            control_cmd.linear.x = self.Kp[0, 0] * x_dest_robotframe + self.Ki[0, 0] * x_i_err + self.Kd[0, 0] * x_diff
            control_cmd.angular.z = self.Kp[1, 1] * y_dest_robotframe + self.Ki[1, 1] * yaw_i_err + self.Kd[1,1] * yaw_d_err
            self.pub.publish(control_cmd)

            prev_x_dest_robotframe = x_dest_robotframe
            prev_y_dest_robotframe = y_dest_robotframe

            time.sleep(0.1)

    # ------------------------------------------------------------------
    # Callback when goal point is published
    # ------------------------------------------------------------------
    def planning_callback(self, msg: PointStamped):
        trajectory = plan_curved_trajectory((msg.point.x, msg.point.y)) 
        # 'trajectory' is length=num_points list of length3 tuples (x, y, change_theta) in world coordinates

        for waypoint in trajectory:
            self.controller(waypoint)
        control_cmd = Twist()
        self.pub.publish(control_cmd)

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
