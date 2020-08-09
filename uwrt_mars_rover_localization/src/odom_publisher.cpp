#include <nav_msgs/Odometry.h>

#include <boost/array.hpp>

#include "ros/ros.h"

#define UPDATE_FREQ 0.5
#define FRAME_ID "odom_link"
#define CHILD_FRAME_ID FRAME_ID

boost::array<double, 36> pose_covariance = {0};
boost::array<double, 36> twist_covariance = {0};

int main(int argc, char** argv) {
  ros::init(argc, argv, "odom_publisher");

  ros::NodeHandle n;
  ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("wheel_encoder/odometry", 1000);

  ros::Rate rate(UPDATE_FREQ);
  uint32_t id = 0;

  // Continuously publish odometry data at update frequency
  while (ros::ok()) {
    // Create msg
    nav_msgs::Odometry odom_msg;

    // Header
    odom_msg.header.seq = id++;
    odom_msg.header.stamp = ros::Time::now();
    odom_msg.header.frame_id = FRAME_ID;

    odom_msg.child_frame_id = CHILD_FRAME_ID;

    // Pose
    odom_msg.pose.pose.position.x = 0;
    odom_msg.pose.pose.position.y = 0;
    odom_msg.pose.pose.position.z = 0;
    odom_msg.pose.pose.orientation.x = 0;
    odom_msg.pose.pose.orientation.y = 0;
    odom_msg.pose.pose.orientation.z = 0;
    odom_msg.pose.pose.orientation.w = 1;
    odom_msg.pose.covariance = pose_covariance;

    // Twist
    odom_msg.twist.twist.linear.x = 0;
    odom_msg.twist.twist.linear.y = 0;
    odom_msg.twist.twist.linear.z = 0;
    odom_msg.twist.twist.angular.x = 0;
    odom_msg.twist.twist.angular.y = 0;
    odom_msg.twist.twist.angular.z = 0;
    odom_msg.twist.covariance = twist_covariance;

    // Publish message
    odom_pub.publish(odom_msg);
    ROS_INFO_STREAM("odom message published");
    rate.sleep();
  }
}