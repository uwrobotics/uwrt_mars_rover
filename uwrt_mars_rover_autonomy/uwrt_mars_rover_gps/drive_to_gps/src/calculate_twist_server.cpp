#include <actionlib/server/simple_action_server.h>
#include <drive_to_gps/gps_goalAction.h>
#include <ros/callback_queue.h>
#include "drive_to_gps/heading.h"
#include "geometry_msgs/Twist.h"
#include "ros/ros.h"
#include "sensor_msgs/NavSatFix.h"
#include "utils/utils.h"

#define MAX_ERROR 5          // error for distance comparisons in metres
#define MAX_LINEAR_VEL 1.0   // max linear velocity of the rover (change)
#define MAX_ANGULAR_VEL 1.0  // max angular velocity of the rover (change)

typedef actionlib::SimpleActionServer<drive_to_gps::gps_goalAction> Server;

ros::Publisher pub;
ros::CallbackQueue my_queue;
drive_to_gps::headingPtr curr_head = NULL;  // current heading of the rover
sensor_msgs::NavSatFixPtr curr_gps = NULL;  // current gps coordinates
int degrees_to_target = 0;

void execute(const drive_to_gps::gps_goalGoalConstPtr &goal, Server *as) {
  drive_to_gps::gps_goalFeedback feedback;
  if (curr_gps == NULL || curr_head == NULL) {
    ros::Rate r(1);
    ROS_INFO("Waiting for topics to publish data");
    while (ros::ok() && (curr_gps == NULL || curr_head == NULL)) {
      my_queue.callAvailable();
      r.sleep();
    }
  }
  sensor_msgs::NavSatFixPtr gps_goal(new sensor_msgs::NavSatFix(goal->gps_goal));
  bool end = false;
  ros::Rate rate(1);
  while (calculate_distance(gps_goal, curr_gps) > MAX_ERROR && !end && ros::ok()) {
    ROS_INFO("Calculating degrees, heading and sending twist message");
    geometry_msgs::Twist msg;
    int goal_heading = calculate_degrees(gps_goal, curr_gps);
    int multiplier = goal_heading > curr_head->degrees ? 1 : -1;
    int heading_diff = abs(goal_heading - curr_head->degrees);
    ROS_INFO("Heading between current heading and goal is %d and direction is %d", heading_diff, multiplier);
    msg.angular.z = heading_diff > 4 ? multiplier * MAX_ANGULAR_VEL : 0;
    msg.linear.x = MAX_LINEAR_VEL;
    feedback.go_to_goal = msg;

    as->publishFeedback(feedback);
    my_queue.callAvailable();
    rate.sleep();
  }
  drive_to_gps::gps_goalResult r;
  r.arrived = !end;
  as->setSucceeded(r);
}

void update_current_heading(const drive_to_gps::headingConstPtr &c_head) {
  if (curr_head == NULL) curr_head = drive_to_gps::headingPtr(new drive_to_gps::heading);
  curr_head->degrees = c_head->degrees;
}

void update_current_gps(const sensor_msgs::NavSatFixConstPtr &gps_current) {
  if (curr_gps == NULL) curr_gps = sensor_msgs::NavSatFixPtr(new sensor_msgs::NavSatFix);
  curr_gps->latitude = gps_current->latitude;
  curr_gps->longitude = gps_current->longitude;
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "calculate_twist_server");
  ros::NodeHandle n, nq;

  nq.setCallbackQueue(&my_queue);
  drive_to_gps::headingPtr curr_head;
  // publish current heading of the rover
  Server server(n, "calculate_twist", boost::bind(&execute, _1, &server), false);
  ros::Subscriber sub_curr_heading = nq.subscribe("/heading/curr", 1, update_current_heading);
  ros::Subscriber sub_curr_gps = nq.subscribe("/gps/fix", 1, update_current_gps);
  server.start();

  ros::spin();

  return 0;
}