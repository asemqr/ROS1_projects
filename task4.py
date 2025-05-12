#!/usr/bin/env python
import numpy as np
import matplotlib.pyplot as plt

# DH Transformation Matrix
def dh_transform(theta, d, a, alpha):
    """Compute the D-H transformation matrix."""
    return np.array([
        [np.cos(theta), -np.sin(theta) * np.cos(alpha), np.sin(theta) * np.sin(alpha), a * np.cos(theta)],
        [np.sin(theta), np.cos(theta) * np.cos(alpha), -np.cos(theta) * np.sin(alpha), a * np.sin(theta)],
        [0, np.sin(alpha), np.cos(alpha), d],
        [0, 0, 0, 1]
    ])

# Forward Kinematics Calculation
def forward_kinematics(theta1, theta2, theta3, L1, L2, L3):
    """Compute the end-effector position using forward kinematics."""
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

# Cubic Trajectory Calculation in Joint Space
def cubic_coefficients(q0, qf, t0, tf):
    """Compute cubic polynomial coefficients."""
    a0 = q0
    a1 = 0
    a2 = (3 * (qf - q0)) / (tf ** 2)
    a3 = (-2 * (qf - q0)) / (tf ** 3)
    return a0, a1, a2, a3

# Generate trajectory in joint space
def cubic_trajectory(a0, a1, a2, a3, t):
    """Generate cubic trajectory."""
    return a0 + a1 * t + a2 * t**2 + a3 * t**3

# Plot Workspace
def plot_workspace(L1, L2, L3):
    """Plot the workspace of the robot."""
    theta1_range = np.linspace(-np.pi/2, np.pi/2, 50)
    theta2_range = np.linspace(-np.pi/2, np.pi/2, 50)
    theta3_range = np.linspace(-np.pi/2, np.pi/2, 50)

    x_positions = []
    y_positions = []

    # Loop through all combinations of theta1, theta2, theta3
    for theta1 in theta1_range:
        for theta2 in theta2_range:
            for theta3 in theta3_range:
                x, y = forward_kinematics(theta1, theta2, theta3, L1, L2, L3)
                x_positions.append(x)
                y_positions.append(y)

    # Plot the workspace points
    plt.scatter(x_positions, y_positions, c='blue', marker='o', s=1, label="Workspace")

    return x_positions, y_positions

# Main function to plot the workspace and the trajectory
def plot_trajectory_within_workspace(L1, L2, L3):
    # Step 1: Plot workspace
    plot_workspace(L1, L2, L3)

    # Step 2: Generate random joint angles for start and end points
    theta1_start = np.random.uniform(-np.pi/2, np.pi/2)
    theta2_start = np.random.uniform(-np.pi/2, np.pi/2)
    theta3_start = np.random.uniform(-np.pi/2, np.pi/2)
    
    theta1_end = np.random.uniform(-np.pi/2, np.pi/2)
    theta2_end = np.random.uniform(-np.pi/2, np.pi/2)
    theta3_end = np.random.uniform(-np.pi/2, np.pi/2)

    # Step 3: Time parameters for interpolation
    t0 = 0
    tf = 1
    t = np.linspace(t0, tf, 100)

    # Step 4: Compute cubic coefficients for each joint
    a0_1, a1_1, a2_1, a3_1 = cubic_coefficients(theta1_start, theta1_end, t0, tf)
    a0_2, a1_2, a2_2, a3_2 = cubic_coefficients(theta2_start, theta2_end, t0, tf)
    a0_3, a1_3, a2_3, a3_3 = cubic_coefficients(theta3_start, theta3_end, t0, tf)

    # Step 5: Generate joint angle trajectories
    theta1_traj = cubic_trajectory(a0_1, a1_1, a2_1, a3_1, t)
    theta2_traj = cubic_trajectory(a0_2, a1_2, a2_2, a3_2, t)
    theta3_traj = cubic_trajectory(a0_3, a1_3, a2_3, a3_3, t)

    # Step 6: Compute end-effector trajectory using forward kinematics
    x_traj = []
    y_traj = []
    for theta1, theta2, theta3 in zip(theta1_traj, theta2_traj, theta3_traj):
        x, y = forward_kinematics(theta1, theta2, theta3, L1, L2, L3)
        x_traj.append(x)
        y_traj.append(y)

    # Step 7: Plot the curved trajectory on the workspace
    plt.plot(x_traj, y_traj, 'r--', label="Curved Trajectory")

# Mark the start and end points in the trajectory
    plt.plot(x_traj[0], y_traj[0], 'ro', label="Start Point")
    plt.plot(x_traj[-1], y_traj[-1], 'go', label="End Point")

    # Step 8: Finalize the plot
    plt.xlabel('X Position (mm)')
    plt.ylabel('Y Position (mm)')
    plt.title('Workspace and Curved Trajectory of the 3 DoF Planar Robot')
    plt.axis('equal')
    plt.grid(True)
    plt.legend()
    plt.show()

# Robot parameters (link lengths in mm)
L1 = 135.0
L2 = 135.0
L3 = 46.7

# Call the function to plot the workspace and the trajectory
plot_trajectory_within_workspace(L1, L2, L3)
