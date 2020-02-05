#ifndef CALCULATE_DEGREES_H
#define CALCULATE_DEGREES_H

#include "math.h"
#include "ros/ros.h"
#include "sensor_msgs/NavSatFix.h"

double deg_to_rad(double deg){
  return M_PI/180.0 * deg;
}

int calculate_degrees(sensor_msgs::NavSatFix& curr_gps, sensor_msgs::NavSatFix * prev){
  double dlong = deg_to_rad(curr_gps.longitude) - deg_to_rad(prev->longitude);
  double lat1 = deg_to_rad(prev->latitude);
  double lat2 = deg_to_rad(curr_gps.latitude);
  double dx = cos(lat2) * sin(dlong);
  double dy = cos(lat1) * sin(lat2) - sin(lat1)*cos(lat2)*cos(dlong);
  double angle = 180.0/M_PI * atan2(dx, dy);  // convert radians to degrees
  ROS_INFO("Angle is %lf", angle);
  int heading = (int(std::round(angle)) + 360) % 360;
  ROS_INFO("Heading is %d degrees", heading);
  return heading;
}

#endif