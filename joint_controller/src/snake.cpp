#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include <cmath>  // For sine wave calculations

// Number of joints in the robot
const int num_joints = 6;

// Publisher array for each joint
ros::Publisher joint_pubs[num_joints];

// Function to generate a sine wave for each joint with a phase shift
double generateSineWave(double time, double frequency, double amplitude, double phase_shift) {
    return amplitude * std::sin(2.0 * M_PI * frequency * time + phase_shift);
}

int main(int argc, char **argv) {
    // Initialize the ROS node
    ros::init(argc, argv, "snake_wave_publisher");
    ros::NodeHandle nh;

    // Define joint names (adjust these to match your joint names)
    std::string joint_names[num_joints] = {
        "/robot/joint1_position_controller/command",
        "/robot/joint2_position_controller/command",
        "/robot/joint3_position_controller/command",
        "/robot/joint4_position_controller/command",
        "/robot/joint5_position_controller/command",
        "/robot/joint6_position_controller/command"
    };

    // Initialize publishers for each joint
    for (int i = 0; i < num_joints; i++) {
        joint_pubs[i] = nh.advertise<std_msgs::Float64>(joint_names[i], 10);
    }

    // Parameters for the sine wave
    double amplitude = 0.15;  // Maximum position value (adjust for your robot)
    double frequency = 0.3;  // Frequency of the wave (Hz)
    double phase_shift = M_PI / 6;  // Phase shift between each joint (radians)
    
    ros::Rate loop_rate(50);  // 50 Hz loop rate

    // Main loop
    while (ros::ok()) {
        double time = ros::Time::now().toSec();

        // Publish a sine wave to each joint with a phase shift
        for (int i = 0; i < num_joints; i++) {
            std_msgs::Float64 msg;
            msg.data = generateSineWave(time, frequency, amplitude, i * phase_shift);
            joint_pubs[i].publish(msg);

            // Log the command
            ROS_INFO("Joint %d command: %f", i + 1, msg.data);
        }

        // Spin once and sleep for the remainder of the loop
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}

