#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include <limits>  // For std::numeric_limits

// Variable to store the previous value for comparison
double previous_value = -std::numeric_limits<double>::infinity();  // Initialize to very small value

// Publisher for joint command
ros::Publisher joint_pub;

// Callback function for the subscriber
void jointCommandCallback(const std_msgs::Float64::ConstPtr& msg) {
    std_msgs::Float64 command_msg;

    ROS_INFO("Received value: %f", msg->data);

    // Check if the new incoming value is higher than the previous one
    if (msg->data > previous_value) {
        command_msg.data = msg->data;
        joint_pub.publish(command_msg);
        ROS_INFO("Publishing new joint command: %f", command_msg.data);
    } else {
        ROS_INFO("Received value is not higher. No command sent.");
    }

    // Update the previous value
    previous_value = msg->data;
}

int main(int argc, char **argv) {
    // Initialize the ROS node
    ros::init(argc, argv, "rotate");
    ros::NodeHandle nh;

    // Initialize the publisher for the joint's command topic
    // Update this topic to the one your controller subscribes to
    joint_pub = nh.advertise<std_msgs::Float64>("/robot/joint5_position_controller/command", 100);

    // Create a subscriber to listen for incoming std_msgs/Float64 messages
    ros::Subscriber sub = nh.subscribe("/joint5/input", 100, jointCommandCallback);

    // Log the setup
    ROS_INFO("Node initialized and ready to process incoming messages.");

    // Spin to handle incoming messages and invoke callbacks
    ros::spin();

    return 0;
}

