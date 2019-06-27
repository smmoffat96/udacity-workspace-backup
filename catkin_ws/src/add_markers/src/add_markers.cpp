#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Pose.h>
#include <nav_msgs/Odometry.h>

void odomCallback(const nav_msgs::Odometry::pose odom_pose) {
    ROS_INFO("Position -> x: [%f], y: [%f]", odom_pose->pose.pose.position.x, odom_pose.pose.position.y);
}

void goalCallback(const move_base_msgs::MoveBaseGoal goal) {

}

int main(int argc, char** argv) {
    ros::init(argc, argv, "add_markers");
    ros::NodeHandle n;
    ros::Rate r(1);
    ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);
    ros::Subscriber odom_sub = n.subscribe("odom_pose", 1, odomCallback);
    ros::Subscriber goal_sub = n.subscribe("goal_pose", 1, goalCallback);
    ros::spin();

    // Odometry pose
    marker.pose.position.x = goal_pose->target_pose.pose.position.x;
    float current_robot_pose_x = odom_sub->pose.pose.position.x;
    marker.pose.position.y = odom_sub->pose.pose.position.y;

    // Set our initial shape type to be a cube
    uint32_t shape = visualization_msgs::Marker::CUBE;
    uint32_t pick_del_drop = 0;

    while (ros::ok()) {
        visualization_msgs::Marker marker;
        // Set the frame ID and timestamp
        marker.header.frame_id = "map";
        marker.header.stamp = ros::Time::now();

        marker.ns = "add_markers";
        marker.id = 0;

        marker.type = shape;

        marker.scale.x = 0.5;
        marker.scale.y = 0.5;
        marker.scale.z = 0.5;

        marker.color.r = 0.0f;
        marker.color.g = 1.0f;
        marker.color.b = 0.0f;
        marker.color.a = 1.0;

        marker.pose.position.z = 0;
        marker.pose.orientation.x = 0;
        marker.pose.orientation.y = 0;
        marker.pose.orientation.z = 0;
        marker.pose.orientation.w = 1.0;

        marker.lifetime = ros::Duration();

        switch (pick_del_drop)
        {
        // Pick
        case 0:
            marker.action = visualization_msgs::Marker::ADD;

            // Publish the marker
            while (marker_pub.getNumSubscribers() < 1) {
                if (!ros::ok()) {
                    return 0;
                }
                ROS_WARN_ONCE("Please create a subscriber to the marker");
                sleep(1);
            }
            marker_pub.publish(marker);
            ROS_INFO("Pickup");
            

            pick_del_drop = 1;

            break;
        // Delete
        case 1:
            marker.action = visualization_msgs::Marker::DELETE;

            // Publish the marker
            while (marker_pub.getNumSubscribers() < 1) {
                if (!ros::ok()) {
                    return 0;
                }
                ROS_WARN_ONCE("Please create a subscriber to the marker");
                sleep(1);
            }
            marker_pub.publish(marker);
            ROS_INFO("Delete");

            pick_del_drop = 2;

            break;
        // Drop
        case 2:
            marker.action = visualization_msgs::Marker::ADD;

            // Publish the marker
            while (marker_pub.getNumSubscribers() < 1) {
                if (!ros::ok()) {
                    return 0;
                }
                ROS_WARN_ONCE("Please create a subscriber to the marker");
                sleep(1);
            }
            marker_pub.publish(marker);
            ROS_INFO("Dropoff");

            pick_del_drop = 3;

            break;
        // End
        case 3:
            ROS_INFO("End");
            break;
        }

        ros::Duration(5.0).sleep();
    }
}