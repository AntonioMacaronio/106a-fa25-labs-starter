# These lines allow us to import rclpy so we can use Python and its Node class
import rclpy
from rclpy.node import Node

# This line imports the built-in string message type that our node will use to structure its data to pass on our topic
from std_msgs.msg import String
from my_chatter_msgs.msg import TimestampString
import datetime

class SimplePublisher(Node):
    def __init__(self):
        super().__init__('simple_publisher')
        self.publisher_ = self.create_publisher(TimestampString, 'my_chatter_talk', 10)
        
        # We create a timer with a callback (a function that runs automatically when something happens so you don't have to constantly check if something has happened) 
        self.input_message = ""
        while self.input_message != "q":
            self.input_message = input("Please enter a line of text and press <Enter>:")
            self.generate_and_send_message()

    def generate_and_send_message(self):
        # get the data needed to generate the message
        
        current_datetime = datetime.datetime.now()
        # timestamp = current_datetime.strftime("%m-%d-%Y %H:%M:%S")
        timestamp = self.get_clock().now().nanoseconds

        # generate the message
        msg = TimestampString()
        msg.message = self.input_message
        msg.timestamp = timestamp

        # send the message and log it
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    simple_publisher = SimplePublisher()
    rclpy.spin(simple_publisher)

    simple_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
