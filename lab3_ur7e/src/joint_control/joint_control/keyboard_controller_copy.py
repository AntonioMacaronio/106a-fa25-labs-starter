#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import sys
import tty
import termios
import threading
import select
import time
import sys

# Key mappings
INCREMENT_KEYS = ['1','2','3','4','5','6']
DECREMENT_KEYS = ['q','w','e','r','t','y']
JOINT_STEP = 0.065 # radians per key press

class KeyboardController(Node):
    def __init__(self, joint_positions):
        super().__init__('ur7e_keyboard_controller')
        
        self.joint_names = [
            'shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint',
            'wrist_2_joint', 'wrist_3_joint'
        ]
        self.joint_positions = joint_positions    
        
        self.pub = self.create_publisher(JointTrajectory, '/scaled_joint_trajectory_controller/joint_trajectory', 10)
        self.timer = self.create_timer(1, self.callback)

    def callback(self):
        traj = JointTrajectory()
        traj.joint_names = self.joint_names
        point = JointTrajectoryPoint()
        point.positions = self.joint_positions
        point.velocities = [0.0]*6
        point.time_from_start.sec = 1
        traj.points.append(point)
        self.pub.publish(traj)

def main(args=None):
    rclpy.init(args=args)
    joint_positions = [float(x) for x in sys.argv[-6:]]
    node = KeyboardController(joint_positions)
    try:
        rclpy.spin_once(node)
        time.sleep(1)
    except KeyboardInterrupt:
        node.running = False
        print("\nExiting keyboard controller...")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
