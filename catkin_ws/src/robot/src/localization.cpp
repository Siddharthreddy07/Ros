#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Quaternion.h>

ros::Publisher odometry_pub;

void gpsCallback(const sensor_msgs::NavSatFix::ConstPtr& gps_msg) {
    nav_msgs::Odometry odometry_msg;
    odometry_msg.header = gps_msg->header;

    // Populate the Odometry message with the GPS data
    odometry_msg.pose.pose.position.x = gps_msg->latitude;
    odometry_msg.pose.pose.position.y = gps_msg->longitude;
    odometry_msg.pose.pose.position.z = gps_msg->altitude;

    // Convert GPS data to quaternion for orientation (assuming no orientation data in NavSatFix)
    geometry_msgs::Quaternion quaternion;
    quaternion.x = 0.0;
    quaternion.y = 0.0;
    quaternion.z = 0.0;
    quaternion.w = 1.0;
    odometry_msg.pose.pose.orientation = quaternion;

    // Set linear and angular velocities to zero (assuming no velocity information in NavSatFix)
    odometry_msg.twist.twist.linear.x = 0.0;
    odometry_msg.twist.twist.linear.y = 0.0;
    odometry_msg.twist.twist.angular.z = 0.0;

    odometry_pub.publish(odometry_msg);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "gps_to_odometry_republisher");
    ros::NodeHandle nh;

    ros::Subscriber gps_sub = nh.subscribe("/gps/fix", 10, gpsCallback);
    odometry_pub = nh.advertise<nav_msgs::Odometry>("/gps/odometry", 10);

    ros::spin();

    return 0;
}
