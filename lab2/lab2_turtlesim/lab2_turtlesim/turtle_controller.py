import rclpy
from rclpy.node import Node 
from geometry_msgs.msg import Twist, Vector3
import sys
import math

key_to_vector = {
    "w": [Vector3(x= 0.0, y=1.0, z=0.0), Vector3(x=0.0, y=0.0, z=0.0)],
    "a": [Vector3(x=-1.0, y=0.0, z=0.0), Vector3(x=0.0, y=0.0, z=0.0)],
    "s": [Vector3(x=0.0, y=-1.0, z=0.0), Vector3(x=0.0, y=0.0, z=0.0)],
    "d": [Vector3(x= 1.0, y=0.0, z=0.0), Vector3(x=0.0, y=0.0, z=0.0)],
    "z": [Vector3(x= 0.0, y=0.0, z=0.0), Vector3(x=0.0, y=0.0, z=math.pi/2)],
    "x": [Vector3(x= 0.0, y=0.0, z=0.0), Vector3(x=0.0, y=0.0, z=-math.pi/2)],
    "c": [Vector3(x= 0.0, y=0.0, z=0.0), Vector3(x=0.0, y=0.0, z=0.0)],
    "v": [Vector3(x= 0.0, y=0.0, z=0.0), Vector3(x=0.0, y=0.0, z=0.0)],
}

class TurtleControllerPublisher(Node):
    def __init__(self, turtle_num):
        super().__init__('turtle_controller_publisher')
        self.turtle_num = turtle_num
        self.publisher_ = self.create_publisher(Twist, f"{self.turtle_num}/cmd_vel", 10)
        self.input_message = ""

        while self.input_message != "exit":
            self.input_message = input("Please enter a movement key and press <Enter>: ")
            self.generate_and_send_message(self.input_message)
    
    def generate_and_send_message(self, key):
        linear_velocity, angular_velocity = key_to_vector[key]
        twist_msg = Twist()
        twist_msg.linear = linear_velocity
        twist_msg.angular = angular_velocity
        self.publisher_.publish(twist_msg)


def main(args = None):
    turtle_num = sys.argv[-1]
    rclpy.init(args = args)

    turtle_controller_publisher = TurtleControllerPublisher(turtle_num)
    rclpy.spin(turtle_controller_publisher)

    turtle_controller_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == "main":
    main()