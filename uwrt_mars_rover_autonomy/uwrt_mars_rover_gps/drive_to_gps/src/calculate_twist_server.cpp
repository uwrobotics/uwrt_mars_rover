#include <ros/ros.h>
#include <utils.h>
#include <uwrt_mars_rover_msgs/gps_goalAction.h>
#include <uwrt_mars_rover_msgs/gps_heading.h>
#include <geometry_msgs/Twist.h>
#include <calculate_twist_server.h>

#include <ros/callback_queue.h>
#include <actionlib/server/simple_action_server.h>

int main(int argc, char **argv) {
  ros::init(argc, argv, "calculate_twist_server");
  ros::NodeHandle n, nq;
  TwistAction server("calculate_twist", n);

  nq.setCallbackQueue(&server.queue_);
  server.init();

  nq.subscribe("/heading/curr", 1, &TwistAction::update_current_heading, &server);
  nq.subscribe("/gps/fix", 1, &TwistAction::update_current_gps, &server);

  ros::spin();

  return 0;
}