#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include <cmath>   // For std::fmod
#include <thread>  // For std::thread

// Function to generate square wave
double squareWave(double time, double period, double amplitude) {
    double half_period = period / 2.0;
    return amplitude * (std::fmod(time, period) < half_period ? 1.0 : -1.0);
}

// Callback function for publishing square-wave command
void publishSquareWaveCommand(ros::Publisher &pub, double amplitude, double period) {
    ros::Time start_time = ros::Time::now();
    ros::Rate loop_rate(100);  // 100 Hz rate

    while (ros::ok()) {
        ros::Time current_time = ros::Time::now();
        double elapsed_time = (current_time - start_time).toSec();
        double command_value = squareWave(elapsed_time, period, amplitude);

        std_msgs::Float64 msg;
        msg.data = command_value;
        pub.publish(msg);

        ROS_INFO("Publishing command value: %f", command_value);

        ros::spinOnce();
        loop_rate.sleep();
    }
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "square_wave_publisher");
    ros::NodeHandle nh;

    // Initialize publisher for a single joint (example: joint1)
    ros::Publisher joint_pub = nh.advertise<std_msgs::Float64>("/robot/joint1_position_controller/command", 100);

    // Parameters for square wave
    double amplitude = 1.0;  // Change as needed
    double period = 10.0;    // Change as needed (10 seconds for full cycle)

    // Start publishing square-wave commands
    publishSquareWaveCommand(joint_pub, amplitude, period);

    return 0;
}

