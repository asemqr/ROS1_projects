#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "move_group_interface_tutorial");
    ros::NodeHandle node_handle;
    ros::AsyncSpinner spinner(0);
    spinner.start();

    static const std::string PLANNING_GROUP = "move_aidana";
    moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);

    const robot_state::JointModelGroup *joint_model_group =
        move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

    geometry_msgs::PoseStamped current_pose;
    geometry_msgs::PoseStamped target_pose;

    current_pose = move_group.getCurrentPose();
    target_pose = current_pose;
    target_pose.pose.position.x -= 1.4;

    ros::Rate loop_rate(50);

    while (ros::ok())
    {
        move_group.setApproximateJointValueTarget(target_pose);
        move_group.move();
        current_pose = move_group.getCurrentPose();

        if (abs(current_pose.pose.position.x - target_pose.pose.position.x) < 0.01)
        {
            break;
        }
        loop_rate.sleep();
    }

    target_pose.pose.position.y -= 0.7;

    while (ros::ok())
    {
        move_group.setApproximateJointValueTarget(target_pose);
        move_group.move();
        current_pose = move_group.getCurrentPose();

        if (abs(current_pose.pose.position.y - target_pose.pose.position.y) < 0.01)
        {
            break;
        }
        loop_rate.sleep();
    }

    target_pose.pose.position.x += 0.7;

    while (ros::ok())
    {
        move_group.setApproximateJointValueTarget(target_pose);
        move_group.move();
        current_pose = move_group.getCurrentPose();

        if (abs(current_pose.pose.position.x - target_pose.pose.position.x) < 0.01)
        {
            break;
        }
        loop_rate.sleep();
    }

    target_pose.pose.position.y += 0.7;

    while (ros::ok())
    {
        move_group.setApproximateJointValueTarget(target_pose);
        move_group.move();
        current_pose = move_group.getCurrentPose();

        if (abs(current_pose.pose.position.y - target_pose.pose.position.y) < 0.01)
        {
            break;
        }
        loop_rate.sleep();
    }

    ROS_INFO("Done");
    ros::shutdown();
    return 0;
}
