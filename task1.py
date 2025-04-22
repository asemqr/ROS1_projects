#!/usr/bin/env python
import numpy as np
import matplotlib.pyplot as plt

def dh_transform(theta, d, a, alpha):
    """Compute the D-H transformation matrix."""
    return np.array([
        [np.cos(theta), -np.sin(theta) * np.cos(alpha), np.sin(theta) * np.sin(alpha), a * np.cos(theta)],
        [np.sin(theta), np.cos(theta) * np.cos(alpha), -np.cos(theta) * np.sin(alpha), a * np.sin(theta)],
        [0, np.sin(alpha), np.cos(alpha), d],
        [0, 0, 0, 1]
    ])

def forward_kinematics(theta1, theta2, theta3, L1, L2, L3):
    """Compute the end-effector position using forward kinematics."""
    # D-H Parameters
    d1, d2, d3 = 0, 0, 0  # Link offsets
    a1, a2, a3 = L1, L2, L3  # Link lengths
    alpha1, alpha2, alpha3 = 0, 0, 0  # Link twists

    # Transformation matrices
    T1 = dh_transform(theta1, d1, a1, alpha1)
    T2 = dh_transform(theta2, d2, a2, alpha2)
    T3 = dh_transform(theta3, d3, a3, alpha3)

    # Combine transformations
    T02 = np.dot(T1, T2)  # From base to joint 2
    T03 = np.dot(T02, T3)  # From base to end-effector

    # Return end-effector position
    return T03[0, 3], T03[1, 3]  # x, y

def plot_workspace(L1, L2, L3):
    """Plot the workspace of the robot."""
    theta1_range = np.linspace(-np.pi/2, np.pi/2, 100)
    theta2_range = np.linspace(-np.pi/2, np.pi/2, 100)
    theta3_range = np.linspace(-np.pi/2, np.pi/2, 100)

    x_positions = []
    y_positions = []

    # Loop through all combinations of theta1, theta2, theta3
    for theta1 in theta1_range:
        for theta2 in theta2_range:
            for theta3 in theta3_range:
                x, y = forward_kinematics(theta1, theta2, theta3, L1, L2, L3)
                x_positions.append(x)
                y_positions.append(y)

    # Plot the workspace
    plt.figure()
    plt.scatter(x_positions, y_positions, c='blue', marker='o', s=1)
    plt.xlabel('X Position (mm)')
    plt.ylabel('Y Position (mm)')
    plt.title('Workspace of the 3 DoF Planar Robot using D-H Parameters')
    plt.axis('equal')
    plt.grid(True)
    plt.show()

# Example usage with given link lengths
L1 = 135
L2 = 135
L3 = 46.7
plot_workspace(L1, L2, L3)


