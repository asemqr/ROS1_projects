#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from std_msgs.msg import Float64
import numpy as np

# Robot Link Lengths
L1 = 0.675 * 2  # Length of Link 1
L2 = 0.675 * 2  # Length of Link 2
L3 = 0.467  # Length of Link 3

def inverse_kinematics_with_orientation(x, y, phi):
    # Your inverse kinematics implementation
    x_wrist = x - L3 * np.cos(phi)
    y_wrist = y - L3 * np.sin(phi)
    D = (x_wrist**2 + y_wrist**2 - L1**2 - L2**2) / (2 * L1 * L2)

    if abs(D) > 1:
        raise ValueError("Target is outside of the reachable workspace.")

    q2 = np.arctan2(np.sqrt(1 - D**2), D)  # Elbow-up solution
    q1 = np.arctan2(y_wrist, x_wrist) - np.arctan2(L2 * np.sin(q2), L1 + L2 * np.cos(q2))
    q3 = phi - q1 - q2

    return q1, q2, q3

def publish_angles(angles, pubs):
    for pub, angle in zip(pubs, angles):
        msg = Float64()
        msg.data = angle
        pub.publish(msg)
        rospy.loginfo("Published angle: %f", msg.data)

def main():
    rospy.init_node('robot_control', anonymous=True)

    # Create publishers for each joint
    pubs = [
        rospy.Publisher('/robot/joint1_position_controller/command', Float64, queue_size=10),  # First joint
        rospy.Publisher('/robot/joint2_position_controller/command', Float64, queue_size=10),      # Second joint (set to 0)
        rospy.Publisher('/robot/joint3_position_controller/command', Float64, queue_size=10),      # Third joint
        rospy.Publisher('/robot/joint4_position_controller/command', Float64, queue_size=10),      # Fourth joint (set to 0)
        rospy.Publisher('/robot/joint5_position_controller/command', Float64, queue_size=10)          # Fifth joint
    ]

    rate = rospy.Rate(10)  # 10 Hz

    target_cartesian = (2, 1, np.pi/4)  # Example target (x, y, phi)
    time_interval = 5  # Time to reach the target

    try:
        # Compute joint angles for the target
        q_target = inverse_kinematics_with_orientation(target_cartesian[0], target_cartesian[1], target_cartesian[2])
        angles_to_publish = [q_target[0], 0, q_target[1], 0, q_target[2]]  # Set joint 2 and joint 6 to 0

        # Publish angles while ROS is active
        start_time = rospy.Time.now()
        while not rospy.is_shutdown():
            current_time = rospy.Time.now() - start_time
            if current_time.to_sec() < time_interval:
                publish_angles(angles_to_publish, pubs)
                rate.sleep()
            else:
                break

    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()
