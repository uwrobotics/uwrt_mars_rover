
#include "uwrt_arm_msgs/UWRTArmTwist.h"

#include <geometry_msgs/Twist.h>
#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/Float64MultiArray.h>

#include <vector>

class TeleopTurtle {
 public:
  TeleopTurtle();

 private:
  void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);
  ros::NodeHandle nh_;

  // axes
  int left_stick_up_down_, left_stick_left_right_, LT_, right_stick_left_right_, right_stick_up_down_, RT_,
      cross_key_left_right_, cross_key_up_down_;

  // buttons
  int A_, B_, X_, Y_, LB_, RB_, start_, back_, power_, button_stick_left_, button_stick_right_;

  // scales
  double left_stick_up_down_scale_fast_, left_stick_up_down_scale_slow_, left_stick_left_right_scale_fast_,
      left_stick_left_right_scale_slow_, right_stick_up_down_scale_, right_stick_left_right_scale_;

  ros::Publisher drivetrain_twist_publisher_;
  ros::Publisher arm_publisher;
  ros::Publisher claw_publisher;
  ros::Publisher camera_publisher;

  ros::Subscriber joy_sub_;
};