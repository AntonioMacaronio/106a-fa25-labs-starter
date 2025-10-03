import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped

import tf2_ros
import sys

class Tf2Listener(Node):
    def __init__(self, source_frame, target_frame):
        super().__init__('echoclone')

        self.tfBuffer = tf2_ros.Buffer()
        self.tfListener = tf2_ros.TransformListener(self.tfBuffer, self)

        self.target_frame = target_frame
        self.source_frame = source_frame

        self.timer = self.create_timer(1, self.listener_callback)
        
    def listener_callback(self):
        try:
            currtime = rclpy.time.Time()
            tfsmsg = self.tfBuffer.lookup_transform(self.source_frame, self.target_frame, currtime)

            time = tfsmsg.header.stamp
            translation = tfsmsg.transform.translation
            rotation = tfsmsg.transform.rotation

            print(
            f"""\nAt time {time.sec}.{time.nanosec}
            \nTranslation: [{translation.x:.3}, {translation.y:.3}, {translation.z:.3}]
            \nRotation: in Quaternion (xyzw): [{rotation.x:.3}, {rotation.y:.3}, {rotation.z:.3}, {rotation.w:.3}] \n
            """
            )
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            pass

def main(args = None):
    rclpy.init(args=args)
    source_frame = sys.argv[-2]
    target_frame = sys.argv[-1]
    print(source_frame)
    print(target_frame)

    tf2_listener = Tf2Listener(source_frame=source_frame, target_frame=target_frame)

    rclpy.spin(tf2_listener)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    tf2_listener.destroy_node()
    rclpy.shutdown()



if __name__ == "__main__":
    main()