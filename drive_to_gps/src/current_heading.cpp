#include "calculate_degrees.h"
#include <drive_to_gps/CurrentHeadingConfig.h>
#include <drive_to_gps/heading.h>
#include <dynamic_reconfigure/server.h>
#include <math.h>
#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>

/*
Current heading is calculated in degrees with 0 degrees at North and positive is
clockwise
*/

ros::Publisher pub_curr_head;
sensor_msgs::NavSatFixPtr prev_gps = NULL;
int refresh_rate = 1; // initialize 1hz as the loop rate (can be changed)

void reconfigure_callback(drive_to_gps::CurrentHeadingConfig &config,
                          uint32_t level) {
  ROS_INFO("Rate = %d", config.refresh_rate);
  refresh_rate = config.refresh_rate;
}

// determine the current heading of the rover
void determine_curr_heading(const sensor_msgs::NavSatFixConstPtr &curr_gps) {
  // if there is no prev_gps coordinate return flag for unknown heading
  drive_to_gps::heading msg;
  if (prev_gps == NULL) {
    prev_gps = sensor_msgs::NavSatFixPtr(new sensor_msgs::NavSatFix());
    ROS_INFO("Prev GPS is NULL and curr GPS has latitude %lf and longitude %lf",
             curr_gps->latitude, curr_gps->longitude);
    msg.valid = false;
    msg.degrees = 0;
  } else {
    ROS_INFO("Lat1 %lf, Lat2 %lf, Lon1 %lf, lon2 %lf", prev_gps->latitude,
             curr_gps->latitude, prev_gps->longitude, curr_gps->longitude);
    msg.valid = true;
    ROS_INFO("Calculating degrees");
    msg.degrees = calculate_degrees(curr_gps, prev_gps);
    pub_curr_head.publish(msg);
  }
  prev_gps->latitude = curr_gps->latitude;
  prev_gps->longitude = curr_gps->longitude;
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "current_heading");
  ros::NodeHandle n;

  pub_curr_head = n.advertise<drive_to_gps::heading>("/heading/curr", 1);

  ros::Subscriber sub_heading =
      n.subscribe("/gps/fix", 2, determine_curr_heading);
  dynamic_reconfigure::Server<drive_to_gps::CurrentHeadingConfig> server;
  dynamic_reconfigure::Server<drive_to_gps::CurrentHeadingConfig>::CallbackType
      f;

  ros::Rate loop_rate(1);
  f = boost::bind(&reconfigure_callback, _1, _2);
  server.setCallback(f);

  while (ros::ok()) {
    ros::spinOnce();
    loop_rate = ros::Rate(refresh_rate);
    loop_rate.sleep();
  }

  return 0;
}