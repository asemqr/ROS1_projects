// #include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
// #include <moveit_visual_tools/moveit_visual_tools.h>

// Main MoveIt libraries are included
int main(int argc, char **argv)
{
    // Initialize the ROS node
    ros::init(argc, argv, "move_group_interface_tutorial");
    ros::NodeHandle node_handle;
    ros::AsyncSpinner spinner(0);
    spinner.start(); // Required for MoveIt, cannot use ros::spinOnce()

    // Define the planning group
    static const std::string PLANNING_GROUP = "move_assem"; // Specify your MoveIt group here
    moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);

    // For joint control
    const robot_state::JointModelGroup *joint_model_group =
        move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

    // Define current and target poses
    geometry_msgs::PoseStamped current_pose;
    geometry_msgs::PoseStamped target_pose;

    // Get current end-effector pose
    current_pose = move_group.getCurrentPose();
    target_pose = current_pose;

    // Set target pose slightly back on the x-axis
    target_pose.pose.position.x -= 1.4;

    // Set loop frequency
    ros::Rate loop_rate(50);

    while (ros::ok())
    {
        // Calculate the trajectory and move the robot
        move_group.setApproximateJointValueTarget(target_pose);
        move_group.move();
        
        // Check if the robot has reached the target position
        current_pose = move_group.getCurrentPose();
        if (abs(current_pose.pose.position.x - target_pose.pose.position.x) < 0.01)
        {
            break;
        }

        loop_rate.sleep();
    }

    ROS_INFO("Done");
    ros::shutdown();
    return 0;
}
