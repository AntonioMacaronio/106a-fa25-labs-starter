import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from std_srvs.srv import Empty
from turtlesim.srv import TeleportAbsolute
from turtle_patrol_interface.srv import Patrol


class Turtle1PatrolServer(Node):
    def __init__(self):
        super().__init__('turtle_patrol_server')

        self._srv = self.create_service(Patrol, '/turtle_patrol', self.patrol_callback)

        # Publisher: actually drives each turtle with constant updates
        self._turtle_publishers = {}        # maps turtle names to publishers
        self.data = {}              # maps turtle_name to [x, y, theta, velocity, omega]
        self.teleport_clients = {}  # maps turtle_name to TeleportAbsolute Service

        # Timer: publish current speeds at 10 Hz
        self._pub_timer = self.create_timer(0.1, self._publish_current_cmd)

        self.get_logger().info('Turtle1PatrolServer ready (continuous publish mode).')

    # -------------------------------------------------------
    # Timer publishes current Twist
    # -------------------------------------------------------
    def _publish_current_cmd(self):
        # Run for every turtle
        
        for turtle_name, (x, y, theta, velocity, omega) in self.data.items():
            # publish the linear and angular speeeds
            msg = Twist()
            msg.linear.x = velocity
            msg.angular.z = omega
            self._turtle_publishers[turtle_name].publish(msg)

            

    # -------------------------------------------------------
    # Service callback: update speeds + positions
    # -------------------------------------------------------
    def patrol_callback(self, request: Patrol.Request, response: Patrol.Response):
        self.get_logger().info(
            f"Patrol request: vel={request.vel:.2f}, omega={request.omega:.2f}, x={request.x:.2f}, y={request.y:.2f}, theta={request.theta:.2f}"
        )

        # If we've never seen this turtle before, create the service client and the publisher
        if request.turtle_name not in self.data:
            self._turtle_publishers[request.turtle_name] = self.create_publisher(Twist, f'/{request.turtle_name}/cmd_vel', 10)
            self.teleport_clients[request.turtle_name] = self.create_client(TeleportAbsolute, f'/{request.turtle_name}/teleport_absolute')
        
        self.data[request.turtle_name] = [
            float(request.x), 
            float(request.y), 
            float(request.theta), 
            float(request.vel), 
            float(request.omega)
        ]

        # teleport the turtle to each position
        req = TeleportAbsolute.Request()
        req.x = request.x
        req.y = request.y
        req.theta = request.theta
        self.teleport_clients[request.turtle_name].call_async(req)

        # Prepare response Twist reflecting current command
        cmd = Twist()
        cmd.linear.x = float(request.vel)
        cmd.angular.z = float(request.omega)
        response.cmd = cmd

        self.get_logger().info(
            f"Streaming cmd_vel: lin.x={float(request.vel):.2f}, ang.z={float(request.omega):.2f} (10 Hz)"
        )
        return response


def main(args=None):
    rclpy.init(args=args)
    node = Turtle1PatrolServer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
