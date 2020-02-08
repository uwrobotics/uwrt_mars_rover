#include "geometry_msgs/Twist.h"
#include "sensor_msgs/NavSatFix.h"
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <drive_to_gps/gps_goalAction.h>
#include <ros/ros.h>

/*
this is a test action client for calculate_twist_server action service
it takes the feedback from the service and publishes it to turtlesim node
this can be used to test the service
*/

double lat = 0.0;
double lon = 0.0;
ros::Publisher pub;

void doneCb(const actionlib::SimpleClientGoalState &state,
            const drive_to_gps::gps_goalResultConstPtr &result) {
  ROS_INFO("Finished in state [%s]", state.toString().c_str());
  ROS_INFO("Answer: %i", result->arrived);
  ros::shutdown();
}

// Called once when the goal becomes active
void activeCb() { ROS_INFO("Goal just went active"); }

// Called every time feedback is received for the goal
void feedbackCb(const drive_to_gps::gps_goalFeedbackConstPtr &feedback) {
  ROS_INFO("Got linear %lf and angular %lf", feedback->go_to_goal.linear.x,
           feedback->go_to_goal.angular.z);
  pub.publish(feedback->go_to_goal);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "turtle_client");
  ros::NodeHandle n;
  // create the action client
  // true causes the client to spin its own thread
  actionlib::SimpleActionClient<drive_to_gps::gps_goalAction> ac(
      "calculate_twist", true);
  pub = n.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 1);

  ROS_INFO("Waiting for action server to start.");
  ROS_INFO("Goal has lat %lf and long %lf", lat, lon);
  // wait for the action server to start
  ac.waitForServer(); // will wait for infinite time
  ros::spinOnce();

  ROS_INFO("Action server started, sending goal.");
  // send a goal to the action
  sensor_msgs::NavSatFix msg;
  msg.latitude = lat;
  msg.longitude = lon;
  ROS_INFO("Goal has lat %lf and long %lf", lat, lon);
  drive_to_gps::gps_goalGoal goal;
  goal.gps_goal = msg;
  ac.sendGoal(goal, &doneCb, &activeCb, &feedbackCb);

  ros::spin();
  // exit
  return 0;
}