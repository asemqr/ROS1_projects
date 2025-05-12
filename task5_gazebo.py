#!/usr/bin/env python

import rospy
from std_msgs.msg import Float64
import numpy as np

# Robot Link Lengths
L1 = 0.675 * 2  # Length of Link 1
L2 = 0.675 * 2  # Length of Link 2
L3 = 0.467      # Length of Link 3

# Define the publishers for each joint
def initialize_publishers():
    rospy.init_node('robot_mover', anonymous=True)
    publishers = [
        rospy.Publisher('/robot/joint1_position_controller/command', Float64, queue_size=10),  # Joint 1
        rospy.Publisher('/robot/joint2_position_controller/command', Float64, queue_size=10),      # Joint 2 (set to 0)
        rospy.Publisher('/robot/joint3_position_controller/command', Float64, queue_size=10),      # Joint 3
        rospy.Publisher('/robot/joint4_position_controller/command', Float64, queue_size=10),      # Joint 4 (set to 0)
        rospy.Publisher('/robot/joint5_position_controller/command', Float64, queue_size=10)       # Joint 5
    ]
    return publishers

def inverse_kinematics_with_orientation(x, y, phi):
    # Calculate wrist center
    x_wrist = x - L3 * np.cos(phi)
    y_wrist = y - L3 * np.sin(phi)
    D = (x_wrist**2 + y_wrist**2 - L1**2 - L2**2) / (2 * L1 * L2)

    if abs(D) > 1:
        raise ValueError("Target is outside of the reachable workspace.")

    q2 = np.arctan2(np.sqrt(1 - D**2), D)  # Elbow-up solution
    q1 = np.arctan2(y_wrist, x_wrist) - np.arctan2(L2 * np.sin(q2), L1 + L2 * np.cos(q2))
    q3 = phi - q1 - q2

    return q1, q2, q3

def cubic_polynomial(q0, qf, v0, vf, t0, tf, t):
    M = np.array([[1, t0, t0**2, t0**3],
                  [0, 1, 2*t0, 3*t0**2],
                  [1, tf, tf**2, tf**3],
                  [0, 1, 2*tf, 3*tf**2]])
    
    b = np.array([q0, v0, qf, vf])
    a = np.linalg.solve(M, b)
    return a[0] + a[1]*t + a[2]*t**2 + a[3]*t**3

def generate_trajectory(via_points, time_intervals):
    joint_trajectory = []
    
    for i in range(len(via_points) - 1):
        q0 = via_points[i]
        qf = via_points[i + 1]
        t0 = sum(time_intervals[:i])
        tf = t0 + time_intervals[i]

        for t in np.linspace(t0, tf, 50):  # 50 points per segment
            q1 = cubic_polynomial(q0[0], qf[0], 0, 0, t0, tf, t)
            q2 = cubic_polynomial(q0[1], qf[1], 0, 0, t0, tf, t)
            q3 = cubic_polynomial(q0[2], qf[2], 0, 0, t0, tf, t)
            joint_trajectory.append((q1, q2, q3))

    return joint_trajectory

def publish_joint_angles(publishers, angles):
    if len(angles) != 3:  # Only 3 angles are being used
        rospy.logerr("Expected 3 angles for joints 1, 3, and 5 but got %d.", len(angles))
        return

    # Create message objects
    msg = [Float64() for _ in range(5)]
    
    # Assign joint angles; set joint 2 and joint 4 to 0
    msg[0].data = angles[0]  # Joint 1
    msg[1].data = 0.0        # Joint 2 (set to 0)
    msg[2].data = angles[1]  # Joint 3
    msg[3].data = 0.0        # Joint 4 (set to 0)
    msg[4].data = angles[2]  # Joint 5

    # Publish the angles
    for i in range(len(publishers)):
        publishers[i].publish(msg[i])
        rospy.loginfo("Published joint %d: %f", i + 1, msg[i].data)

# Main function to run the robot mover
if __name__ == '__main__':
    publishers = initialize_publishers()

    # Define via points in Cartesian space with varying orientations (x, y, phi)
    via_points_cartesian = [
        (3, 0, np.pi/4),
        (2, 1, np.pi/6),
        (1.5, 1.5, np.pi/3),
        (1, 2, np.pi/2),
        (0, 3, np.pi/2)
    ]

    # Define time intervals between each via point
    time_intervals = [2, 2, 2, 2]  # seconds for each transition between via points

    # Convert Cartesian via points to joint angles using inverse kinematics
    via_points = []
    for point in via_points_cartesian:
        x, y, phi = point
        try:
            q1, q2, q3 = inverse_kinematics_with_orientation(x, y, phi)
            via_points.append((q1, q2, q3))
        except ValueError as e:
            print("Error: Point ({}, {}) is outside of the reachable workspace: {}".format(x, y, e))

    # Generate the joint-space trajectory
    joint_trajectory = generate_trajectory(via_points, time_intervals)

    # Move through the joint trajectory
    for angles in joint_trajectory:
        publish_joint_angles(publishers, angles)
        rospy.sleep(0.1)  # Sleep for a short duration to allow for message processing

