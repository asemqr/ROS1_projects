#!/usr/bin/env python
import numpy as np
import matplotlib.pyplot as plt

# Correct Robot parameters (Lengths of links)
L1 = 135.0  # Length of link 1
L2 = 135.0  # Length of link 2
L3 = 46.7   # Length of link 3

# Function to compute cubic coefficients
def cubic_coefficients(q0, qf, t0, tf):
    a0 = q0
    a1 = 0
    a2 = (3 * (qf - q0)) / (tf ** 2)
    a3 = (-2 * (qf - q0)) / (tf ** 3)
    return a0, a1, a2, a3

# Function to compute the cubic trajectory
def cubic_trajectory(a0, a1, a2, a3, t):
    return a0 + a1 * t + a2 * t ** 2 + a3 * t ** 3

# Plot the manipulator configuration with end-effector orientation
def plot_manipulator(q1, q2, q3, title="Manipulator Configuration with End-Effector Orientation"):
    # Forward kinematics to get the joint positions
    joint0 = np.array([0, 0])  # Base position
    joint1 = joint0 + np.array([L1 * np.cos(q1), L1 * np.sin(q1)])
    joint2 = joint1 + np.array([L2 * np.cos(q1 + q2), L2 * np.sin(q1 + q2)])
    end_effector = joint2 + np.array([L3 * np.cos(q1 + q2 + q3), L3 * np.sin(q1 + q2 + q3)])
    
    # Plot the links
    plt.figure(figsize=(6, 6))
    plt.plot([joint0[0], joint1[0]], [joint0[1], joint1[1]], 'ro-', label="Link 1", color='red')
    plt.plot([joint1[0], joint2[0]], [joint1[1], joint2[1]], 'go-', label="Link 2", color='green')
    plt.plot([joint2[0], end_effector[0]], [joint2[1], end_effector[1]], 'bo-', label="Link 3", color='blue')

    # End-effector orientation arrow
    orientation_x = end_effector[0] + 20 * np.cos(q1 + q2 + q3)  # Adjust for visibility
    orientation_y = end_effector[1] + 20 * np.sin(q1 + q2 + q3)  # Adjust for visibility
    plt.arrow(end_effector[0], end_effector[1], orientation_x - end_effector[0], orientation_y - end_effector[1],
              head_width=5, head_length=10, fc='red', ec='red')

    # Set plot limits and labels
    plt.xlim(-300, 300)
    plt.ylim(-300, 300)
    plt.xlabel("X Position")
    plt.ylabel("Y Position")
    plt.title(title)
    plt.grid(True)
    plt.legend()
    plt.show()

# Time duration
t0 = 0  # Initial time
tf = 5  # Final time (seconds)
time_steps = np.linspace(t0, tf, 100)  # Time steps

# Initial and final joint angles (adjust these based on your task)
q1_0, q1_f = 0, -1.2   # Joint 1: 0 -> -0.8
q2_0, q2_f = 0, 1.2    # Joint 2: 0 -> 0.8
q3_0, q3_f = 1.57, 1.2  # Joint 3: 1.57 -> 0.8

# Calculate cubic coefficients for each joint
a0_1, a1_1, a2_1, a3_1 = cubic_coefficients(q1_0, q1_f, t0, tf)
a0_2, a1_2, a2_2, a3_2 = cubic_coefficients(q2_0, q2_f, t0, tf)
a0_3, a1_3, a2_3, a3_3 = cubic_coefficients(q3_0, q3_f, t0, tf)

# Generate joint trajectories
q1_traj = [cubic_trajectory(a0_1, a1_1, a2_1, a3_1, t) for t in time_steps]
q2_traj = [cubic_trajectory(a0_2, a1_2, a2_2, a3_2, t) for t in time_steps]
q3_traj = [cubic_trajectory(a0_3, a1_3, a2_3, a3_3, t) for t in time_steps]

# Plot the joint trajectories
plt.figure(figsize=(10, 6))

# Plot joint 1 trajectory
plt.plot(time_steps, q1_traj, label="Theta 1 (q1: 0 -> -0.8)", color='blue', linestyle='-')

# Plot joint 2 trajectory
plt.plot(time_steps, q2_traj, label="Theta 2 (q2: 0 -> 0.8)", color='green', linestyle='--')

# Plot joint 3 trajectory
plt.plot(time_steps, q3_traj, label="Theta 3 (q3: 1.57 -> 0.8)", color='orange', linestyle='-.')

# Add titles and labels
plt.title("Cubic Trajectory for the 3-DOF Planar Manipulator")
plt.xlabel("Time [s]")
plt.ylabel("Joint Angles [rad]")
plt.legend(loc="best")
plt.grid(True)
plt.show()

# Plot initial and final configurations of the manipulator
plot_manipulator(q1_traj[0], q2_traj[0], q3_traj[0], title="Initial Configuration")
plot_manipulator(q1_traj[-1], q2_traj[-1], q3_traj[-1], title="Final Configuration")

