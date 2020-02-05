#include "ros/ros.h"
#include "sensor_msgs/NavSatFix.h"
#include "geometry_msgs/Twist.h"
#include "drive_to_gps/heading.h"
#include "drive_to_gps/dist_heading.h"
#include "calculate_degrees.h"

// calculates the required angular and linear velocity of the robot to reach the destination

ros::Publisher pub;

double calculate_distance();

void determine_heading_callback(const drive_to_gps::heading curr_head){
  // if there is no prev_gps coordinate return flag for unknown heading
  if (!curr_head.valid){
    ROS_INFO("Invalid Heading");
  }
  else{
    ROS_INFO("Current heading is %d", curr_head.degrees);
  }
}

/* determine desired heading based on current reference frame 
void determine_target_heading(sensor_msgs::NavSatFix gps_goal){
  drive_to_gps::dist_heading msg;
  if (prev_gps == NULL){
    msg.valid = false;
    msg.degrees = 0;
    msg.distance = 0;
  }
  else{
    msg.degrees = (calculate_degrees(gps_goal, prev_gps) - curr_head + 360) % 360;
    msg.valid = true;
    msg.distance = 
  }
  pub_target_head.publish(msg);
  ROS_INFO("Degrees to target %d", msg.degrees);
}
*/

int main(int argc, char **argv)
{
  ros::init(argc, argv, "calculate_twist");
  ros::NodeHandle n;

  // publish current heading of the rover
  pub = n.advertise<geometry_msgs::Twist>("/gps_goal_twist", 1);
  ros::Subscriber sub_curr_heading = n.subscribe("/heading/curr", 1, determine_heading_callback);
  /**
   * A count of how many messages we have sent. This is used to create
   * a unique string for each message.
   */
  // grab gps
  // wait 1 sec
  // grab gps 
  while(ros::ok()){
    ros::spin();
  }

  return 0;
}