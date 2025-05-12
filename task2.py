#!/usr/bin/env python
import numpy as np
import matplotlib.pyplot as plt

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

# Time duration
t0 = 0  # Initial time
tf = 5  # Final time (seconds)
time_steps = np.linspace(t0, tf, 100)  # Time steps

# Initial and final joint angles
q1_0, q1_f = 0, -0.8   # Joint 1: 0 -> -0.8
q2_0, q2_f = 0, 0.8    # Joint 2: 0 -> 0.8
q3_0, q3_f = 1.57, 0.8  # Joint 3: 1.57 -> 0.8

# Calculate cubic coefficients for each joint
a0_1, a1_1, a2_1, a3_1 = cubic_coefficients(q1_0, q1_f, t0, tf)
a0_2, a1_2, a2_2, a3_2 = cubic_coefficients(q2_0, q2_f, t0, tf)
a0_3, a1_3, a2_3, a3_3 = cubic_coefficients(q3_0, q3_f, t0, tf)

# Generate trajectories
q1_traj = [cubic_trajectory(a0_1, a1_1, a2_1, a3_1, t) for t in time_steps]
q2_traj = [cubic_trajectory(a0_2, a1_2, a2_2, a3_2, t) for t in time_steps]
q3_traj = [cubic_trajectory(a0_3, a1_3, a2_3, a3_3, t) for t in time_steps]

# Plotting
plt.figure(figsize=(10, 6))

# Plot joint 1 trajectory
plt.plot(time_steps, q1_traj, label="Joint 1 (q1: 0 -> -0.8)", color='r', linestyle='-')

# Plot joint 2 trajectory
plt.plot(time_steps, q2_traj, label="Joint 2 (q2: 0 -> 0.8)", color='g', linestyle='--')

# Plot joint 3 trajectory
plt.plot(time_steps, q3_traj, label="Joint 3 (q3: 1.57 -> 0.8)", color='b', linestyle='-.')

# Add titles and labels
plt.title("Cubic Trajectory of Joints")
plt.xlabel("Time (s)")
plt.ylabel("Joint Angles (radians)")
plt.legend(loc="best")
plt.grid(True)
plt.show()

