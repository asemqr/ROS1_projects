#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include <cmath>
#include <thread>

// Publisher for the joint command
ros::Publisher joint_pub;

// Function to generate sine wave
double sineWave(double time, double amplitude, double frequency) {
    return amplitude * std::sin(2.0 * M_PI * frequency * time);
}

// Callback function for publishing sine-wave command
void publishSineWaveCommand(ros::Publisher &pub, double amplitude, double frequency) {
    ros::Time start_time = ros::Time::now();
    ros::Rate loop_rate(10);  // 10 Hz rate

    while (ros::ok()) {
        ros::Time current_time = ros::Time::now();
        double elapsed_time = (current_time - start_time).toSec();
        double command_value = sineWave(elapsed_time, amplitude, frequency);

        std_msgs::Float64 msg;
        msg.data = command_value;
        pub.publish(msg);

        ROS_INFO("Publishing command value: %f", command_value);

        ros::spinOnce();
        loop_rate.sleep();
    }
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "sine_wave_command_publisher");
    ros::NodeHandle nh;

    // Initialize publisher for the joint
    joint_pub = nh.advertise<std_msgs::Float64>("/robot/joint1_position_controller/command", 100);

    // Parameters for sine wave
    double amplitude = 1.0;  // Change as needed
    double frequency = 0.1;  // Frequency in Hz (e.g., 0.1 Hz for a 10-second period)

    // Start publishing sine-wave commands
    publishSineWaveCommand(joint_pub, amplitude, frequency);

    return 0;
}

