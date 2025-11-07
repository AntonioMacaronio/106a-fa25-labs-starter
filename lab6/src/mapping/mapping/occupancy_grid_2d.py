################################################################################
#
# OccupancyGrid2d class for ROS 2
#
################################################################################

import rclpy
from rclpy.node import Node

import tf2_ros
from transforms3d.euler import quat2euler

from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA

import numpy as np

from rclpy.qos import QoSProfile, ReliabilityPolicy

class OccupancyGrid2d(Node):
    def __init__(self):
        super().__init__('occupancy_grid_2d')
        self._initialized = False

        # TF buffer + listener
        self._tf_buffer = tf2_ros.Buffer() # Stores the previous transforms
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        # Load parameters
        if not self.load_parameters():
            self.get_logger().error("Error loading parameters.")
            return

        # Callbacks
        if not self.register_callbacks():
            self.get_logger().error("Error registering callbacks.")
            return

        # Initialize grid map
        self._map = np.zeros((self._x_num, self._y_num))
        self._initialized = True
        self.get_logger().info("OccupancyGrid2d initialized successfully.")

    def load_parameters(self):
        self.declare_parameter("random_downsample", 0.1)
        self._random_downsample = self.get_parameter("random_downsample").value

         # Dimensions and bounds.
        # TODO! You'll need to set values for class variables called:
        # -- self._x_num
        # -- self._x_min
        # -- self._x_max
        # -- self._x_res # The resolution in x. Note: This isn't a ROS parameter. What will you do instead?
        # -- self._y_num
        # -- self._y_min
        # -- self._y_max
        # -- self._y_res # The resolution in y. Note: This isn't a ROS parameter. What will you do instead?
        self.declare_parameter("x/num", 25)
        self.declare_parameter("x/min", -10.0)
        self.declare_parameter("x/max", 10.0)
        self._x_num = self.get_parameter("x/num").value
        self._x_min = self.get_parameter("x/min").value
        self._x_max = self.get_parameter("x/max").value
        self._x_res = (self._x_max - self._x_min) / self._x_num

        self.declare_parameter("y/num", 25)
        self.declare_parameter("y/min", -10.0)
        self.declare_parameter("y/max", 10.0)
        self._y_num = self.get_parameter("y/num").value
        self._y_min = self.get_parameter("y/min").value
        self._y_max = self.get_parameter("y/max").value
        self._y_res = (self._y_max - self._y_min) / self._y_num
        

        self.declare_parameter("update/occupied", 0.7)
        self._occupied_update = self.get_parameter("update/occupied").value
        self.declare_parameter("update/occupied_threshold", 0.97)
        self._occupied_threshold = self.get_parameter("update/occupied_threshold").value
        self.declare_parameter("update/free", 0.3)
        self._free_update = self.get_parameter("update/free").value
        self.declare_parameter("update/free_threshold", 0.03)
        self._free_threshold = self.get_parameter("update/free_threshold").value

        # Topics.
        # TODO! You'll need to set values for class variables called:
        # -- self._sensor_topic
        # -- self._vis_topic
        self.declare_parameter("topic/sensor", "/scan")
        self._sensor_topic = self.get_parameter("topic/sensor").value
        self.declare_parameter("topic/vis", "/vis/map")
        self._vis_topic = self.get_parameter("topic/vis").value

        # Frames.
        # TODO! You'll need to set values for class variables called:
        # -- self._sensor_frame
        # -- self._fixed_frame
        self.declare_parameter("frames/sensor", "base_link")
        self._sensor_frame = self.get_parameter("frames/sensor").value
        self.declare_parameter("frames/fixed", "odom")
        self._fixed_frame = self.get_parameter("frames/fixed").value

        return True

    def register_callbacks(self):

        qos_profile = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT
        )
        # Subscriber.
        self._sensor_sub = self.create_subscription(
            LaserScan, self._sensor_topic, self.sensor_callback, qos_profile
        )
        # Publisher.        
        self._vis_pub = self.create_publisher(Marker, self._vis_topic, 10)
        return True

    # Callback to process sensor measurements.
    def sensor_callback(self, msg): # msg = LaserScan object
        if not self._initialized:
            self.get_logger().error("Node not initialized.")
            return

        self.get_logger().debug(f"Sensor pose Callback")
        # Get our current pose from TF.
        try:
            pose = self._tf_buffer.lookup_transform(
                self._fixed_frame, self._sensor_frame, rclpy.time.Time()) 
            # pose = T_world_robot = T_fixed_sensor
        except Exception as e:
            # Writes an error message to the ROS log but does not raise an exception
            self.get_logger().error(f"TF lookup failed: {e}")
            return

        self.get_logger().debug(f"Sensor pose: {pose}")
        # Extract x, y coordinates and heading (yaw) angle of the turtlebot, 
        # assuming that the turtlebot is on the ground plane.
        sensor_x = pose.transform.translation.x
        sensor_y = pose.transform.translation.y
    
        qx = pose.transform.rotation.x
        qy = pose.transform.rotation.y
        qz = pose.transform.rotation.z
        qw = pose.transform.rotation.w
        roll, pitch, yaw = quat2euler([qw, qx, qy, qz])  # [w,x,y,z]

        if abs(pose.transform.translation.z) > 0.05:
            self.get_logger().warn("Robot not on ground plane.")
        if abs(roll) > 0.1 or abs(pitch) > 0.1:
            self.get_logger().warn("Robot roll/pitch too large.")
        # Loop over all ranges in the LaserScan.
        """
            The msg has a min and max angle, so the range is just the distance (how far away) 
            that corresponds to each of the n evenly-spaced angles between min and max angle.
        """
        for idx, r in enumerate(msg.ranges):
            if np.random.rand() > self._random_downsample or np.isnan(r):
                continue
            
            # Get angle of this ray in fixed frame.
            # TODO!
            angle = msg.angle_min + idx * msg.angle_increment + yaw

            if r > msg.range_max or r < msg.range_min:
                continue

            # Walk along this ray from the scan point to the sensor.
            # Update log-odds at each voxel along the way.
            # Only update each voxel once. 
            # The occupancy grid is stored in self._map
            # TODO!
            
            # first let's update the last voxel to be occupied
            try:
                last_voxel_r, last_voxel_c = self.point_to_voxel(sensor_x + r*np.cos(angle), sensor_y + r*np.sin(angle))
                self._map[last_voxel_r, last_voxel_c] += self._occupied_update
                self._map[last_voxel_r, last_voxel_c] = min(self._map[last_voxel_r, last_voxel_c], self._occupied_threshold)
            
                # This loop is finding every voxel that needs to be free-updated and adding that to a set()
                voxels_to_be_free_updated = set((last_voxel_r, last_voxel_c)) # set of (x, y) tuples, we add last_voxel b/c we don't want it to be free_updated
                for dist in np.arange(0, r, min(self._x_res, self._y_res) / 2): # dist along ray
                    # break up dist into x and y components
                    x_change, y_change = np.cos(angle) * dist, np.sin(angle) * dist
                    raywalkpoint = (sensor_x + x_change, sensor_y + y_change) # tuple with position x and y 
                    row, col = self.point_to_voxel(raywalkpoint[0], raywalkpoint[1])
                    prior_voxel_prob = self.colormap(row, col)
                    # Need to update voxels once
                    # Need to have some logic that tells us whether we're on the destination cell
                    if (row, col) not in voxels_to_be_free_updated:
                        # print(self._free_update)
                        # print(row, col, self._map[row, col])
                        self._map[row, col] -= self._free_update
                        self._map[row, col] = max(self._map[row, col], self._free_threshold)
                        voxels_to_be_free_updated.add((row, col))
            except:
                continue
            
        # Visualize.
        self.visualize()

    # Convert (x, y) coordinates in fixed frame to grid coordinates.
    def point_to_voxel(self, x, y):
        ii = int((x - self._x_min) / self._x_res)
        jj = int((y - self._y_min) / self._y_res)

        if ii < 0 or ii >= self._x_num or jj < 0 or jj >= self._y_num:
            print(x, y, ii, jj)
            return None  # invalid voxel
        return ii, jj

    # Get the center point (x, y) corresponding to the given voxel.
    def voxel_center(self, ii, jj):
        return (self._x_min + (0.5 + ii) * self._x_res,
                self._y_min + (0.5 + jj) * self._y_res)

    # Convert between probabity and log-odds.
    def probability_to_logodds(self, p):
        return np.log(p / (1.0 - p))

    def logodds_to_probability(self, l):
        return 1.0 / (1.0 + np.exp(-l))

    # Colormap to take log odds at a voxel to a RGBA color.
    def colormap(self, ii, jj):
        if (ii == 0 and jj == 0) or (ii == 1 and jj == 0) or (ii == 0 and jj == 2):
            return ColorRGBA(r=0.0, g=0.0, b=1.0, a=0.75)
        p = self.logodds_to_probability(self._map[ii, jj])
        return ColorRGBA(r=p, g=0.1, b=1.0 - p, a=0.75)

    # Visualize the map as a collection of flat cubes instead of
    # as a built-in OccupancyGrid message, since that gives us more
    # flexibility for things like color maps and stuff.
    # See http://wiki.ros.org/rviz/DisplayTypes/Marker for a brief tutorial.
    def visualize(self):
        m = Marker()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = self._fixed_frame
        m.ns = "map"
        m.id = 0
        m.type = Marker.CUBE_LIST
        m.action = Marker.ADD
        m.scale.x = self._x_res
        m.scale.y = self._y_res
        m.scale.z = 0.01

        for ii in range(self._x_num):
            for jj in range(self._y_num):
                p = Point()
                p.x, p.y = self.voxel_center(ii, jj)
                m.points.append(p)
                m.colors.append(self.colormap(ii, jj))

        self._vis_pub.publish(m)