/*
Action server for the spiral search algorithm
Currently works with turtlesim for testing 
*/

#include "ros/ros.h"
#include <actionlib/server/simple_action_server.h>
#include <spiralSearch/spiralSearchAction.h>
#include "geometry_msgs/Twist.h" // contains the ros message type Twist
#include "turtlesim/Pose.h" // this allows the node read the pose message type published by the node
#include <sstream> 
#include <cmath>

//Action Server variables 
typedef actionlib::SimpleActionServer<spiralSearch::spiralSearchAction> Server;
spiralSearch::spiralSearchFeedback feedback; 
spiralSearch::spiralSearchResult result; 

const double PI = 3.14159265359;

//To publish commands to turtlesim
ros::Publisher velocity_publisher;
ros::Subscriber pose_subscriber;	
turtlesim::Pose turtlesim_pose;

//Helper callback function for the turtlesim 
void poseCallback(const turtlesim::Pose::ConstPtr & pose_message){
    turtlesim_pose.x = pose_message->x;
    turtlesim_pose.y = pose_message->y;
    turtlesim_pose.theta = pose_message->theta;
}

//Spiral Search Algorithm is implemeted here
void execute(const spiralSearch::spiralSearchGoalConstPtr &goal, Server* as){
    
    //Main spiral search algorithm
    //declare vel_msg as a "Twist" type to be able to change linear and angular velocities
    geometry_msgs::Twist vel_msg;

    double angular_velocity = 5; //this is the constant angular velocity, it controls the speed of rover as it moves around
    double spiral_constant = 0.005; //This primarily controls the space between each radius/how tight it is

    double Vx = 0; 
    double Vy = 0;
    double time = 0; 
    
    //this is the frequency that the loop will follow, by keeping track of "sleep()"
    ros::Rate loop_rate(10);
    
    //Run until the goal has been reached
    //Add capability to be stopped
    while( (turtlesim_pose.x < goal->goal_x) && (turtlesim_pose.y < goal->goal_y)){
        
        time += 0.01;  //Increment time

        //Determine Vx and Vy after each interation 
        Vx = spiral_constant*angular_velocity*(cos(angular_velocity*time) - angular_velocity*time*sin(angular_velocity*time) );
        Vy = spiral_constant*angular_velocity*(sin(angular_velocity*time) - angular_velocity*time*cos(angular_velocity*time) );
        
        //this makes the linear change
        vel_msg.linear.x = Vx;
        vel_msg.linear.y = Vy;

        velocity_publisher.publish(vel_msg);
        ros::spinOnce();//this will call all the callbacks waiting to be called
        loop_rate.sleep();//this allows us to sleep for the required time to keep the desired frequency

        //Add publishing feedback etc here

    }
    
    //this stops the turtle
    vel_msg.linear.x = 0;
    velocity_publisher.publish(vel_msg);
    //cout<<"DONE "<<endl;

}

int main(int argc, char **argv) {

    ros::init(argc, argv, "spiralSearch");
    ros::NodeHandle n, nh;

    Server server(nh, "spiralSearch", boost::bind(&execute, _1, &server), false);

    //For interfacing with turtlesim
	velocity_publisher = n.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 1000);
	pose_subscriber = n.subscribe("/turtle1/pose", 10, poseCallback);	
 
	ROS_INFO("\n\n\n ********Server Started*********\n");
    server.start();
    ros::spin();

    return 0; 

}