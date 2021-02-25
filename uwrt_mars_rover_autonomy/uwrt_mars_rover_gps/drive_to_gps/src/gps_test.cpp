#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>
#include <uwrt_mars_rover_msgs/gps_goalAction.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>

int main(int argc, char **argv) {

    actionlib::SimpleActionClient<uwrt_mars_rover_msgs::gps_goalAction> ac ("turtlebot/calculate_twist", true);

    ROS_INFO("Waiting for action server to start.");

    ac.waitForServer();

    ROS_INFO("Action server started, sending goal.");

    uwrt_mars_rover_msgs::gps_goalGoal goal_;
    goal_.gps_goal.latitude = atoll(argv[1]);
    goal_.gps_goal.longitude = atoll(argv[2]);

    ac.sendGoal(goal_);
    bool reached_goal = ac.waitForResult(ros::Duration(60.0));

    if(reached_goal) {
        actionlib::SimpleClientGoalState state = ac.getState();
        ROS_INFO("Action finished: %s",state.toString().c_str());
    }
    else
        ROS_INFO("Action did not finish before the time out.");

    return 0;
}
