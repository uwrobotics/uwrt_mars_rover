#include "utils/utils.h"

static inline double deg_to_rad(double deg) {
  return M_PI / 180.0 * deg;
}

int calculate_degrees(const sensor_msgs::NavSatFixConstPtr &curr_gps, const sensor_msgs::NavSatFixConstPtr &prev) {
  double dlong = deg_to_rad(curr_gps->longitude - prev->longitude);
  double lat1 = deg_to_rad(prev->latitude);
  double lat2 = deg_to_rad(curr_gps->latitude);
  double dx = cos(lat2) * sin(dlong);
  double dy = cos(lat1) * sin(lat2) - sin(lat1) * cos(lat2) * cos(dlong);
  double angle = 180.0 / M_PI * atan2(dx, dy);  // convert radians to degrees
  ROS_INFO("Angle is %lf", angle);
  int heading = (int(round(angle)) + 360) % 360;
  ROS_INFO("Heading is %d degrees", heading);
  return heading;
}

double calculate_distance(const sensor_msgs::NavSatFixConstPtr &goal, const sensor_msgs::NavSatFixConstPtr &curr) {
  double lat1 = deg_to_rad(curr->latitude);
  double lat2 = deg_to_rad(goal->latitude);
  double dlat = deg_to_rad(goal->latitude - curr->latitude);
  double dlong = deg_to_rad(goal->longitude - curr->longitude);
  double calc = sin(dlat / 2) * sin(dlat / 2) + cos(lat1) * cos(lat2) * sin(dlong / 2) * sin(dlong / 2);
  double c = 2 * atan2(sqrt(calc), sqrt(1 - calc));

  return RADIUS_E * c;
}