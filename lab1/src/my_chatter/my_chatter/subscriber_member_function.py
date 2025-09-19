import rclpy
from rclpy.node import Node

from my_chatter_msgs.msg import TimestampString
import datetime

class SimpleSubscriber(Node):
    def __init__(self):
        super().__init__('simple_subscriber')
        self.subscription = self.create_subscription(
            TimestampString,
            'my_chatter_talk',
            self.listener_callback,
            10)
    
    def listener_callback(self, msg):
        # let's parse the message and create the data to be logged
        sent_datetime = datetime.fromtimestamp(int(msg.timestamp) / 1e9)
        received_datatime = datetime.fromtimestamp(self.get_clock().now() / 1e9)
        sent_readable, received_datatime = sent_datetime.strftime("%m-%d-%Y %H:%M:%S"), received_datatime.strftime("%m-%d-%Y %H:%M:%S")
        self.get_logger().info(
            f'Message" {msg.message}, Sent at: {sent_readable}, Received at {received_datatime}'
        )

def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = MinimalSubscriber()

    rclpy.spin(minimal_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()