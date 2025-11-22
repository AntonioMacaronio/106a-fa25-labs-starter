#!/usr/bin/env python3
import math
import rclpy
import tf2_ros
import numpy as np
import matplotlib.pyplot as plt
import transforms3d.euler as euler
from geometry_msgs.msg import TransformStamped


def plot_trajectory(waypoints):
    """
    waypoints: list of length2 tuples    
    """
    x_vals = [p[0] for p in waypoints]
    y_vals = [p[1] for p in waypoints]
    plt.figure(figsize=(10, 6))
    plt.plot(x_vals, y_vals, '-o', label='Trajectory')
    plt.scatter(x_vals[0], y_vals[0], color='green', s=100, zorder=5, label='Start')
    plt.scatter(x_vals[-1], y_vals[-1], color='red', s=100, zorder=5, label='End')
    for x, y, theta in waypoints:
        plt.arrow(x, y, 0.05*np.cos(theta), 0.05*np.sin(theta),
                  head_width=0.01, head_length=0.005, fc='blue', ec='blue')
    plt.xlabel('X')
    plt.ylabel('Y')
    plt.title('Robot Trajectory')
    plt.grid(True)
    plt.legend()
    plt.axis('equal')
    plt.show()


def bezier_curve(p0, p1, p2, p3, t):
    """Cubic Bézier interpolation between 4 control points."""
    return (1 - t)**3 * p0 + 3*(1 - t)**2*t*p1 + 3*(1 - t)*t**2*p2 + t**3*p3


def generate_bezier_waypoints(x1, y1, theta1, x2, y2, theta2, offset=1.0, num_points=10):
    """Generate (x, y, theta) waypoints along a smooth Bézier path.
    Input:
        1. (x1, y1): start of the curve
        2. (x2, y2): end of the curve 
        3. (theta1, theta2): used to define the control points
    Output:
        1. waypoints: length=num_points list of length3 tuples (x, y, change_theta)
    """
    d1 = np.array([np.cos(theta1), np.sin(theta1)])     # (2, )
    d2 = np.array([-np.cos(theta2), -np.sin(theta2)])   # (2, )
    c1 = np.array([x1, y1]) + offset * d1               # (2, )
    c2 = np.array([x2, y2]) + offset * d2               # (2, )
    t_vals = np.linspace(0, 1, num_points)              # (num_points)
    pts = [bezier_curve(np.array([x1, y1]), c1, c2, np.array([x2, y2]), t) for t in t_vals] # length=num_points list of (2, ) nparrays

    thetas = [] # list of scalars representing the change in angle between each discrete point on our curve.
                # theta[i] = angle change from point[i] to point[i+1]
                # theta[-1] = angle change from point[0] to point[1] (repeated first)
    for i in range(len(pts) - 1):
        dx = pts[i+1][0] - pts[i][0]
        dy = pts[i+1][1] - pts[i][1]
        thetas.append(np.arctan2(dy, dx))
    thetas.append(thetas[-1])
    return [(pts[i][0], pts[i][1], thetas[i]) for i in range(len(pts))] # returns a length=num_points list of length3 tuples (x, y, change_theta)


def plan_curved_trajectory(target_position):
    """Plan a curved trajectory from current odom→base_footprint transform.
    Input:
        1. target_position: length2 tuple of the target position in the robot-frame's coordinates
    
    Output:
        1. waypoints: length=num_points list of length3 tuples (x, y, change_theta)
    """
    node = rclpy.create_node('turtlebot_controller')
    tf_buffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer, node)

    # Keep trying until transform available
    while rclpy.ok():
        try:
            trans = tf_buffer.lookup_transform('odom', 'base_footprint', rclpy.time.Time()) ## TODO: Apply a lookup transform between our world frame and turtlebot frame
            # trans = T_world_turtlebot
            break
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            node.get_logger().warn('TF lookup failed, retrying...')
            rclpy.spin_once(node, timeout_sec=0.1)

    # Extract current pose
    x1 = trans.transform.translation.x
    y1 = trans.transform.translation.y
    q = trans.transform.rotation

    # transforms3d expects [w, x, y, z]
    roll, pitch, yaw = euler.quat2euler([q.w, q.x, q.y, q.z])

    # Compute absolute target position in odom frame
    #x2 = target_position[0] - x1 ## TODO: How would you get x2 from our target position? Remember this is relative to x1
    #y2 = target_position[1] - y1 ## TODO: How would you get x2 from our target position? Remember this is relative to x1
    x2 = x1 + target_position[0] * np.cos(yaw) - target_position[1] * np.sin(yaw)
    y2 = y1 + target_position[0] * np.sin(yaw) + target_position[1] * np.cos(yaw)

    # Generate Bézier waypoints and visualize
    waypoints = generate_bezier_waypoints(x1, y1, yaw, x2, y2, yaw, offset=0.2, num_points=10) # length=num_points list of length3 tuples (x, y, change_theta)
    plot_trajectory(waypoints) # I sometimes comment where I 

    node.destroy_node()
    return waypoints


def main(args=None):
    rclpy.init(args=args)

    # Example: test without TF (offline)
    waypoints = generate_bezier_waypoints(0.0, 0.0, np.pi/2,
                                          0.2, 0.2, np.pi/2,
                                          offset=0.2, num_points=100)
    plot_trajectory(waypoints)

    # Example: with live TF
    plan_curved_trajectory((0.2, 0.2))

    rclpy.shutdown()


if __name__ == '__main__':
    main()