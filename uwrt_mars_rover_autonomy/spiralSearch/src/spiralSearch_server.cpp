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

//Global Variable to stop the server if need be
bool runSpiralSearch = true; 

//Global Variables
static double angular_velocity; //this is the constant angular velocity, it controls the speed of rover as it moves around
static double spiral_constant; //This primarily controls the space between each radius/how tight it is

//Action Server variables 
typedef actionlib::SimpleActionServer<spiralSearch::spiralSearchAction> Server;
spiralSearch::spiralSearchFeedback feedback; 
spiralSearch::spiralSearchResult result; 

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

    //double Vx = 0; 
    //double Vy = 0;
    double velocity = 0;
    
    //this is the frequency that the loop will follow, by keeping track of "sleep()"
    ros::Rate loop_rate(10);

    //Get ROS Time 
    double startTime = ros::Time::now().toSec();
    double currentTime = ros::Time::now().toSec();
    double timeDelta = 0; 
    
    //Run until the goal has been reached
    //Add capability to be stopped
    while( (turtlesim_pose.x < goal->goal_x) && (turtlesim_pose.y < goal->goal_y) && runSpiralSearch){
        
        //Spiral motion using cartisian coordinates and linear velocities 
        //Vx = spiral_constant*angular_velocity*(cos(angular_velocity*time) - angular_velocity*time*sin(angular_velocity*time) );
        //Vy = spiral_constant*angular_velocity*(sin(angular_velocity*time) - angular_velocity*time*cos(angular_velocity*time) );

        //this makes the linear change
        //vel_msg.linear.x = Vx;
        //vel_msg.linear.y = Vy;

        //Use current ROS Time
        currentTime = ros::Time::now().toSec();
        timeDelta = currentTime - startTime; 

        //Send linear and angular velocity commands
        vel_msg.linear.x = spiral_constant*timeDelta;
        vel_msg.angular.z = angular_velocity;  

        velocity_publisher.publish(vel_msg);
        ros::spinOnce();//this will call all the callbacks waiting to be called
        loop_rate.sleep();//this allows us to sleep for the required time to keep the desired frequency

        //Add publishing feedback etc here

    }
    
    //this stops the turtle
    vel_msg.linear.x = 0;
    vel_msg.angular.z = 0;
    velocity_publisher.publish(vel_msg);

}

int main(int argc, char **argv) {

    ros::init(argc, argv, "spiralSearch");
    ros::NodeHandle n, nh;

    nh.param("spiral_constant", spiral_constant, 0.25);
    nh.param("angular_velocity", angular_velocity, 1.0);

    //If parameters are not defined for some ensure the varaibles are not undefined
    if(!nh.hasParam("spiral_constant") || !nh.hasParam("angular_velocity")){
        spiral_constant = 0.1; 
        angular_velocity = 1; 
    }

    Server server(nh, "spiralSearch", boost::bind(&execute, _1, &server), false);

    //For interfacing with turtlesim
	velocity_publisher = n.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 1000);
	pose_subscriber = n.subscribe("/turtle1/pose", 10, poseCallback);	
 
	ROS_INFO("\n\n\n ********Server Started*********\n");
    server.start();
    ros::spin();

    return 0; 

}