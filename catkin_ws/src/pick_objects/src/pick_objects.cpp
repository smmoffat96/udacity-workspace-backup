#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <geometry_msgs/Pose.h>
#include <nav_msgs/Odometry.h>

// Define a client to send goal requests to the move_base server through a SimpleActionClient
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;


int main(int argc, char** argv) {
    // Initialize the pick_objects node
    ros::init(argc, argv, "pick_objects");
    ros::NodeHandle n;
    ros::Rate r(1);
    ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry::pose>("odom_pose", 1);
    ros::Publisher goal_pub = n.advertise<move_base_msgs::MoveBaseGoal>("goal_pose", 1);

    // Tell the action client that we want to spin a thread by default
    MoveBaseClient ac("move_base", true);
    // Wait 5 sec for move_base action server to come up
    while(!ac.waitForServer(ros::Duration(5.0))) {
        ROS_INFO("Waiting for the move_base action server to come up");
    }

    move_base_msgs::MoveBaseGoal goal_pick;
    // Set up the frame parameters
    goal_pick.target_pose.header.frame_id = "map";
    goal_pick.target_pose.header.stamp = ros::Time::now();
    // Define a position and orientation for the robot to reach
    goal_pick.target_pose.pose.position.x = 10.0;
    goal_pick.target_pose.pose.orientation.w = 1.0;
    // Send the goal position and orientation for the robot to reach
    ROS_INFO("Sending pickup goal");
    ac.sendGoal(goal_pick);
    // Wait an infinite time for the results
    ac.waitForResult();

    // Check if the robot reached its goal
    if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
        ROS_INFO("The robot reached the pickup zone");
    else
        ROS_INFO("The robot failed to reach the pickup zone");




    // Wait 5 sec for move_base action server to come up
    while(!ac.waitForServer(ros::Duration(5.0))) {
        ROS_INFO("Waiting for the move_base action server to come up");
    }

    move_base_msgs::MoveBaseGoal goal_drop;

    // Set up the frame parameters
    goal_drop.target_pose.header.frame_id = "map";
    goal_drop.target_pose.header.stamp = ros::Time::now();

    // Define a position and orientation for the robot to reach
    goal_drop.target_pose.pose.position.x = -6.0;
    goal_drop.target_pose.pose.orientation.w = -1.0;

    // Send the goal position and orientation for the robot to reach
    ROS_INFO("Sending dropoff goal");
    ac.sendGoal(goal_drop);

    // Wait an infinite time for the results
    ac.waitForResult();

    // Check if the robot reached its goal
    if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
        ROS_INFO("The robot reached the dropoff zone");
    else
        ROS_INFO("The robot failed to reach the dropoff zone");

    return 0;
}