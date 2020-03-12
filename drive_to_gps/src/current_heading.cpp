#include <drive_to_gps/CurrentHeadingConfig.h>
#include <drive_to_gps/heading.h>
#include <dynamic_reconfigure/server.h>
#include <math.h>
#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>
#include <stack>
#include "utils/utils.h"

/*
Current heading is calculated in degrees with 0 degrees at North and positive is
clockwise
*/

ros::Publisher pub_curr_head;
std::stack<sensor_msgs::NavSatFixPtr> gps_stack;
int refresh_rate = 3;  // configure the timer to publish curr heading every 3 seconds

static inline double ewma(double value, double prev_val, double a = 0.5) {
  return value * a + prev_val * a;
}

void reconfigure_callback(drive_to_gps::CurrentHeadingConfig &config, uint32_t level) {
  ROS_INFO("Rate = %d", config.refresh_rate);
  refresh_rate = config.refresh_rate;
}

void add_gps_to_queue(const sensor_msgs::NavSatFixConstPtr &curr_gps) {
  if (std::isnan(curr_gps->latitude) || std::isnan(curr_gps->longitude == NAN)) {
    return;
  }
  sensor_msgs::NavSatFixPtr item(new sensor_msgs::NavSatFix());
  if (gps_stack.size() > 0) {
    item->latitude = ewma(curr_gps->latitude, gps_stack.top()->latitude);
    item->longitude = ewma(curr_gps->longitude, gps_stack.top()->longitude);
  } else {
    item->latitude = curr_gps->latitude;
    item->longitude = curr_gps->longitude;
  }
  item->header.stamp = curr_gps->header.stamp;
  ROS_INFO_STREAM("Pushed to queue with lat " << item->latitude << " long " << item->longitude << " and time stamp "
                                              << item->header.stamp);
  gps_stack.push(item);
}

// determine the current heading of the rover
void determine_curr_heading(const ros::TimerEvent &event) {
  drive_to_gps::heading msg;
  ROS_INFO("Called timer callback");
  // check front of queue to make sure we can pop
  if (gps_stack.size() > 0) {
    ROS_INFO("Size queue big");
    bool gps_after_time = false;
    sensor_msgs::NavSatFixPtr curr(gps_stack.top());
    gps_stack.pop();
    sensor_msgs::NavSatFixPtr prev = NULL;
    while (gps_stack.size() > 0) {
      prev = gps_stack.top();
      ROS_INFO("Went into loop");
      // ROS_INFO_STREAM("Event real " << event.last_real << " temp stamp " << temp->header.stamp);
      if (event.last_real >= prev->header.stamp && !gps_after_time) {
        ROS_INFO_STREAM("Prev lat and long " << prev->latitude << " " << prev->longitude << " curr lat and long "
                                             << curr->latitude << " " << curr->longitude);
        msg.degrees = calculate_degrees(curr, prev);
        ROS_INFO_STREAM("Published " << msg.degrees);
        pub_curr_head.publish(msg);
        gps_after_time = true;
      }
      gps_stack.pop();
    }
    gps_stack.push(curr);
  }
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "current_heading");
  ros::NodeHandle n;

  pub_curr_head = n.advertise<drive_to_gps::heading>("/heading/curr", 1);

  ros::Subscriber sub_heading = n.subscribe("/fix", 3, add_gps_to_queue);

  ros::Timer timer = n.createTimer(ros::Duration(4), determine_curr_heading);
  dynamic_reconfigure::Server<drive_to_gps::CurrentHeadingConfig> server;
  dynamic_reconfigure::Server<drive_to_gps::CurrentHeadingConfig>::CallbackType f;

  f = boost::bind(&reconfigure_callback, _1, _2);
  server.setCallback(f);

  ros::spin();

  return 0;
}