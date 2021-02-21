#include <ros/ros.h>
#include <calculate_twist_server.h>

int main(int argc, char **argv) {
  ros::init(argc, argv, "calculate_twist_server");
  ros::NodeHandle n, nq;
  TwistAction server("calculate_twist", n);

  nq.setCallbackQueue(&server.queue_);
  nq.subscribe("/heading/curr", 1, &TwistAction::update_current_heading, &server);
  nq.subscribe("/gps/fix", 1, &TwistAction::update_current_gps, &server);

  server.init();

  ros::spin();

  return 0;
}