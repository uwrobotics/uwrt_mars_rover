#include <sensor_msgs/Imu.h>

#include <boost/array.hpp>

#include "ros/ros.h"

#define UPDATE_FREQ 1.0
#define FRAME_ID "imu_link"

boost::array<double, 9> orientation_covariance = {0, 0, 0, 0, 0, 0, 0, 0, 0};

boost::array<double, 9> ang_vel_covariance = {0, 0, 0, 0, 0, 0, 0, 0, 0};

boost::array<double, 9> lin_accel_covariance = {0, 0, 0, 0, 0, 0, 0, 0, 0};

int main(int argc, char** argv) {
  ros::init(argc, argv, "imu_publisher");

  ros::NodeHandle n;
  ros::Publisher imu_pub = n.advertise<sensor_msgs::Imu>("imu/data", 1000);

  ros::Rate rate(UPDATE_FREQ);
  uint32_t id = 0;

  // Continuously publish imu data at update frequency
  while (ros::ok()) {
    // Create msg
    sensor_msgs::Imu imu_msg;

    // Header
    imu_msg.header.seq = id++;
    imu_msg.header.stamp = ros::Time::now();
    imu_msg.header.frame_id = FRAME_ID;

    // Orientation
    imu_msg.orientation.x = 0;
    imu_msg.orientation.y = 0;
    imu_msg.orientation.z = 0;
    imu_msg.orientation.w = 1;
    imu_msg.orientation_covariance = orientation_covariance;

    // Angular velocity
    imu_msg.angular_velocity.x = 0;
    imu_msg.angular_velocity.y = 0;
    imu_msg.angular_velocity.z = 0;
    imu_msg.angular_velocity_covariance = ang_vel_covariance;

    // Linear acceleration
    imu_msg.linear_acceleration.x = 0;
    imu_msg.linear_acceleration.y = 0;
    imu_msg.linear_acceleration.z = 0;
    imu_msg.linear_acceleration_covariance = lin_accel_covariance;

    // Publish message
    imu_pub.publish(imu_msg);
    ROS_INFO_STREAM("imu message published");
    rate.sleep();
  }
}