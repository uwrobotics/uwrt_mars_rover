#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <ros/ros.h>
#include <uwrt_mars_rover_spiral/spiralSearchAction.h>

int main(int argc, char **argv) {
  ros::init(argc, argv, "spiralSearch_client");

  // create the action client
  // true causes the client to spin its own thread
  actionlib::SimpleActionClient<uwrt_mars_rover_spiral::spiralSearchAction> ac("spiralSearch", true);

  ROS_INFO("Waiting for action server to start.");
  ac.waitForServer();  // will wait for infinite time

  ROS_INFO("Action server started, sending goal.");
  // send a goal to the action
  uwrt_mars_rover_spiral::spiralSearchGoal goal;
  goal.radius = 3;
  goal.angular_velocity = 2;   // for state machine use this or use 1.5 and 0.1 for tighter spirals
  goal.spiral_constant = 0.1;  // increase this for looser spirals
  ac.sendGoal(goal);

  // wait for the action to return
  bool finished_before_timeout = ac.waitForResult(ros::Duration(300.00));

  if (finished_before_timeout) {
    actionlib::SimpleClientGoalState state = ac.getState();
    ROS_INFO("Action finished: %s", state.toString().c_str());
  } else {
    ROS_INFO("Action did not finish before the time out.");
  }

  // exit
  return 0;
}