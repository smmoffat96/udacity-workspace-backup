#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

int main(int argc, char** argv) {
    ros::init(argc, argv, "add_markers");
    ros::NodeHandle n;
    ros::Rate r(1);
    ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);

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

        switch (pick_del_drop)
        {
        // Pick
        case 0:
            marker.action = visualization_msgs::Marker::ADD;
            
            // Pickup Pose
            marker.pose.position.x = 0;
            marker.pose.position.y = 0;
            marker.pose.position.z = 0;
            marker.pose.orientation.x = 0;
            marker.pose.orientation.y = 0;
            marker.pose.orientation.z = 0;
            marker.pose.orientation.w = 1.0;

            marker.lifetime = ros::Duration();

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

            marker.lifetime = ros::Duration();

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

            // Dropoff Pose
            marker.pose.position.x = -5;
            marker.pose.position.y = 0;
            marker.pose.position.z = 0;
            marker.pose.orientation.x = 0;
            marker.pose.orientation.y = 0;
            marker.pose.orientation.z = 0;
            marker.pose.orientation.w = 1.0;

            marker.lifetime = ros::Duration();

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

        r.sleep();
    }
}