import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

from forward_kinematics.forward_kinematics import ur7e_forward_kinematics_from_joint_state

class JointStateListener(Node):
    def __init__(self):
        super().__init__('jointstatelistener')
        self.subscription = self.create_subscription(
            JointState,
            '/joint_states',
            self.listener_callback,
            10)
    
    def listener_callback(self, msg):
        # let's parse the message and create the data to be logged
        
        g = ur7e_forward_kinematics_from_joint_state(msg)

        print(g)

def main(args=None):
    rclpy.init(args=args)

    joint_state_listener = JointStateListener()

    rclpy.spin(joint_state_listener)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    joint_state_listener.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()