#!usr/bin/env python

import numpy as np
import scipy as sp
import forward_kinematics.kin_func_skeleton as kfs 

def ur7e_foward_kinematics_from_angles(joint_angles):
    """
    Calculate the orientation of the ur7e's end-effector tool given
    the joint angles of each joint in radians

    Parameters:
    ------------
    joint_angles ((6x) np.ndarray): 6 joint angles (s0, s1, e0, w1, w2, w3)

    Returns: 
    ------------
    (4x4) np.ndarray: homogenous transformation matrix
    """
    q0 = np.ndarray((3, 6)) # Points on each joint axis in the zero config
    w0 = np.ndarray((3, 6)) # Axis vector of each joint axis in the zero config


    q0[:, 0] = [0., 0., 0.1625] # shoulder pan joint - shoulder_link from base link
    q0[:, 1] = [0., 0., 0.1625] # shoulder lift joint - upper_arm_link from shoulder_link
    q0[:, 2] = [0.425, 0., 0.1625] # elbow_joint - forearm_link from shoulder_lift_joint
    q0[:, 3] = [0.817, 0.1333, 0.1625] # wrist 1 - wrist_1_link from elbow_joint
    q0[:, 4] = [0.817, 0.1333, 0.06285] # wrist 2 - wrist_2_link from wrist_1
    q0[:, 5] = [0.817, 0.233, 0.06285] # wrist 3 - wrist_3_link from wrist_2

    w0[:, 0] = [0., 0., 1] # shoulder pan joint
    w0[:, 1] = [0, 1., 0] # shoulder lift joint
    w0[:, 2] = [0., 1., 0] # elbow_joint
    w0[:, 3] = [0., 1., 0] # wrist 1
    w0[:, 4] = [0., 0., -1] # wrist 2 
    w0[:, 5] = [0., 1., 0] # wrist 3

    # Rotation matrix from base_link to wrist_3_link in zero config
    R = np.array([[-1., 0., 0.],
                  [0., 0., 1.], 
                  [0., 1., 0.]])
    
    xi = np.ndarray((6, 6))

    for i in range(6):
        qi = q0[:, i] # (3, )
        wi = w0[:, i] # (3, )
        vi = np.cross(qi, wi) # (3, )shoulder_link

        xi[:3, i] = vi
        xi[3:, i] = wi

    
    g0 = np.zeros((4, 4))
    g0[0:3, 0:3] = R
    g0[3, 3] = 1
    g0[:3, 3] = q0[:, 5]
    

    return kfs.prod_exp(xi, joint_angles) @ g0
    # YOUR CODE HERE (Task 1)


def ur7e_forward_kinematics_from_joint_state(joint_state):
    """
    Computes the orientation of the ur7e's end-effector given the joint
    state.

    Parameters
    ----------
    joint_state (sensor_msgs.JointState): JointState of ur7e robot

    Returns
    -------
    (4x4) np.ndarray: homogenous transformation matrix
    """
    joint_order = {
        'shoulder_pan_joint': 0,
        'shoulder_lift_joint': 1,
        'elbow_joint': 2,
        'wrist_1_joint': 3,
        'wrist_2_joint': 4,
        'wrist_3_joint': 5
    }

    angles = np.zeros(6)
    # YOUR CODE HERE (Task 2)
    names = joint_state.name
    position = np.array(joint_state.position)
    for i in range(6):
        name = names[i]
        angles[joint_order[name]] = position[i]

    return ur7e_foward_kinematics_from_angles(joint_angles = angles)


    # END YOUR CODE HERE
