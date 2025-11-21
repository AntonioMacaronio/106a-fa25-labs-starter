import rclpy
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener
from geometry_msgs.msg import PointStamped 

class TransformCubePose(Node):
    def __init__(self):
        super().__init__('transform_cube_pose')

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.cube_pose_sub = self.create_subscription(
            PointStamped,
            '/cube_pose',
            self.cube_pose_callback,
            10
        )

        self.cube_pose_pub = self.create_publisher(PointStamped, '/transformed_cube_pose', 1)

        rclpy.spin_once(self, timeout_sec=2)
        self.cube_pose = None

    def cube_pose_callback(self, msg: PointStamped):
        if self.cube_pose is None:
            self.cube_pose = self.transform_cube_pose(cube_pose)
            self.get_logger().info('Received cube pose')
        self.cube_pub.publish(self.cube_pose)

    def transform_cube_pose(self, msg: PointStamped):
        """ 
        Transform point into base_link frame
        Args: 
            - msg: PointStamped - The message from /cube_pose, of the position of the cube in camera_depth_optical_frame
        Returns:
            Point: point in base_link_frame in form [x, y, z]
        """
        point_homogenous_camframe = np.array([msg.point.x, msg.point.y, msg.point.z, 1.0])[:, None] # (4, 1)
        T_world_cam_transformmsg = self.tf_buffer.lookup_transform("base_link", "camera_depth_optical_frame", rclpy.time.Time()) # here world = base_link
        quat = np.array([
            T_world_cam_transformmsg.transform.rotation.x, 
            T_world_cam_transformmsg.transform.rotation.y, 
            T_world_cam_transformmsg.transform.rotation.z, 
            T_world_cam_transformmsg.transform.rotation.w
        ])
        scipy_rotationobj = Rotation.from_quat(quat)
        T_world_cam_nparray4x4 = np.zeros((4, 4))
        T_world_cam_nparray4x4[:3, :3] = scipy_rotationobj.as_matrix()
        T_world_cam_nparray4x4[:3, 3]  = np.array([T_world_cam_transformmsg.transform.translation.x, T_world_cam_transformmsg.transform.translation.y, T_world_cam_transformmsg.transform.translation.z])
        point_worldframe = (T_world_cam_nparray4x4 @ point_homogenous_camframe)[:3, 0]
        
        output_msg = PointStamped()
        output_msg.header.frame_id = 'base_link'
        output_msg.header.stamp = self.get_clock().now().to_msg()
        output_msg.point.x = point_worldframe[0]
        output_msg.point.y = point_worldframe[1]
        output_msg.point.z = point_worldframe[2]

        return output_msg

def main(args=None):
    rclpy.init(args=args)
    node = TransformCubePose()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
