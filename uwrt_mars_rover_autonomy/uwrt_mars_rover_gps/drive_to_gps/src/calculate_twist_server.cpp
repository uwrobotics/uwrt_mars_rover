#include <calculate_twist_server.h>

const std::string DEFAULT_GPS_TOPIC = "/gps/fix";

int main(int argc, char **argv) {
  ros::init(argc, argv, "calculate_twist_server");
  ros::NodeHandle n, nq;
  TwistAction server("calculate_twist", n);

  server.pub = n.advertise<geometry_msgs::Twist>("/turtlebot/cmd_vel", 1); // for testing

  std::string topic = uwrt_mars_rover_utils::getParam(n, "calculate_twist", "gps_topic", DEFAULT_GPS_TOPIC);

  nq.setCallbackQueue(&server.queue_);
  nq.subscribe("/heading/curr", 1, &TwistAction::update_current_heading, &server);
  nq.subscribe(topic, 1, &TwistAction::update_current_gps, &server);

  server.init();

  ros::spin();

  return 0;
}