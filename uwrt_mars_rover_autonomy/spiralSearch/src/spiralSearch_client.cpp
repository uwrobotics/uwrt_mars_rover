#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <spiralSearch/spiralSearchAction.h>

int main (int argc, char **argv)
{
    ros::init(argc, argv, "spiralSearch_client");

    // create the action client
    // true causes the client to spin its own thread
    actionlib::SimpleActionClient<spiralSearch::spiralSearchAction> ac("spiralSearch", true);

    ROS_INFO("Waiting for action server to start.");
    ac.waitForServer(); //will wait for infinite time

    ROS_INFO("Action server started, sending goal.");
    // send a goal to the action
    spiralSearch::spiralSearchGoal goal;
    goal.goal_x = 10; 
    goal.goal_y = 10;
    ac.sendGoal(goal);

    //wait for the action to return
    bool finished_before_timeout = ac.waitForResult(ros::Duration(300.00));

    if (finished_before_timeout)
    {
        actionlib::SimpleClientGoalState state = ac.getState();
        ROS_INFO("Action finished: %s",state.toString().c_str());
    }
    else
        ROS_INFO("Action did not finish before the time out.");

    //exit
    return 0;
}