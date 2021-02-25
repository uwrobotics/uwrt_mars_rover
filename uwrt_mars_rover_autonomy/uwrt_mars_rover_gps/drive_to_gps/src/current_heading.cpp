#include <current_heading.h>

/* // dynamic reconfigure method
int refresh_rate = 3;  // configure the timer to publish curr heading every 3 seconds

void reconfigure_callback(drive_to_gps::CurrentHeadingConfig &config, uint32_t level) {
  ROS_INFO_STREAM("Rate = " << config.refresh_rate);
  refresh_rate = config.refresh_rate;
}
*/

/*
Current heading is calculated in degrees with 0 degrees at North and positive is
clockwise
*/

const std::string DEFAULT_GPS_TOPIC = "/gps/fix";

int main(int argc, char **argv) {
  ros::init(argc, argv, "current_heading");
  ros::NodeHandle n;

  ros::Publisher pub_curr_head = n.advertise<uwrt_mars_rover_msgs::gps_heading>("/heading/curr", 1);
  std::stack<sensor_msgs::NavSatFixPtr> gps_stack;

  CurrentHeading currHead(pub_curr_head, gps_stack);

  std::string topic = uwrt_mars_rover_utils::getParam(n, "current_heading", "gps_topic", DEFAULT_GPS_TOPIC);
  
  ros::Subscriber sub_heading = n.subscribe(topic, 3, &CurrentHeading::add_gps_to_queue, &currHead);
  ros::Timer timer = n.createTimer(ros::Duration(4), &CurrentHeading::determine_curr_heading, &currHead);

  /*
  dynamic_reconfigure::Server<drive_to_gps::CurrentHeadingConfig> server;
  dynamic_reconfigure::Server<drive_to_gps::CurrentHeadingConfig>::CallbackType f;

  f = boost::bind(&reconfigure_callback, _1, _2);
  server.setCallback(f);
  */

  ros::spin();

  return 0;
}