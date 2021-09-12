#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

int main(int argc, char** argv) {

    ros::init(argc, argv, "keyboard");

    ros::NodeHandle nh;
    ros::Publisher keyboard_pub = nh.advertise<visualization_msgs::Marker>( "visualization_marker", 0 );

    visualization_msgs::Marker marker;
    marker.header.frame_id = "base_link";
    marker.header.stamp = ros::Time();
    marker.ns = "keyboard";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::MESH_RESOURCE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = 0.15;
    marker.pose.position.y = 0;
    marker.pose.position.z = 0;
    marker.pose.orientation.x = 1.0;
    marker.pose.orientation.y = -1.0;
    marker.pose.orientation.z = -1.0;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 1.0;
    marker.scale.y = 1.0;
    marker.scale.z = 1.0;
    marker.color.a = 1.0; // Don't forget to set the alpha!
    marker.color.r = 1.0;
    marker.color.g = 1.0;
    marker.color.b = 1.0;

    marker.mesh_resource = "package://keyboard_description/meshes/keyboard.obj";

    while (ros::ok()) {
        keyboard_pub.publish(marker);
    }

    ros::Duration(1).sleep();
    ros::shutdown();
}