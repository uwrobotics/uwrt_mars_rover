#pragma once
#include "math.h"
#include "ros/ros.h"
#include "sensor_msgs/NavSatFix.h"

constexpr int RADIUS_E = 6371e3;

static inline double deg_to_rad(double deg);

int calculate_degrees(const sensor_msgs::NavSatFixConstPtr &curr_gps,
                      const sensor_msgs::NavSatFixConstPtr &prev);

double calculate_distance(const sensor_msgs::NavSatFixConstPtr &goal,
                          const sensor_msgs::NavSatFixConstPtr &curr);