#include "ros/ros.h" //library with ros services
#include "geometry_msgs/Twist.h" // contains the ros message type Twist
#include "turtlesim/Pose.h" // this allows the node read the pose message type published by the node
#include <sstream> 

using namespace std;

//the publisher object will send the data to the /turtle1/cmd_vel topic, which the turtlesim node is subscribed to
ros::Publisher velocity_publisher;

//this makes a subcriber object (that will be specified later) that will listen to the pose channel from the turtleism node
ros::Subscriber pose_subscriber;	

//makes a "Pose" object to store information from the callback
turtlesim::Pose turtlesim_pose;

//const double x_min = 0.0;
//const double y_min = 0.0;
const double x_max = 10.5;
const double y_max = 10.5;

const double PI = 3.14159265359;

//Callback fn everytime the turtle pose msg is published over the /turtle1/pose topic.
void poseCallback(const turtlesim::Pose::ConstPtr & pose_message);	

void spiralClean();

int main(int argc, char **argv)
{
	// Initiate new ROS node named "talker"
	ros::init(argc, argv, "turtlesim_cleaner");
	ros::NodeHandle n;
	double speed, angular_speed;
	double distance, angle;
	bool isForward, clockwise;

	velocity_publisher = n.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 1000);
	
	//call poseCallback everytime the turtle pose msg is published over the /turtle1/pose topic.
	pose_subscriber = n.subscribe("/turtle1/pose", 10, poseCallback);	
	//ros::Rate loop_rate(0.5);

	//	/turtle1/cmd_vel is the Topic name
	//	/geometry_msgs::Twist is the msg type 
	ROS_INFO("\n\n\n ********START TESTING*********\n");

	spiralClean();

	//this enters a loop that will call all message callbacks, it will exit once it has been asked to shutdown
	ros::spin();

	return 0;
}


/*

this is the callback function, that will get called every time a new message 
arrives at the "/turtle1/pose" topic, and it will have a queue size of 10

it will send the parts of the pose message to "turtlesim_pose"

turltesim_pose is the actual turtlesim node, while the pose_message is what arrived at the topic

*/


void poseCallback(const turtlesim::Pose::ConstPtr & pose_message){
	turtlesim_pose.x = pose_message->x;
	turtlesim_pose.y = pose_message->y;
	turtlesim_pose.theta = pose_message->theta;
}


void spiralClean()
{
	//declare vel_msg as a "Twist" type to be able to change linear and angular velocities
	geometry_msgs::Twist vel_msg;

	//this is the constant angular velocity, increasing it will result in tighter spirals
	double constant_speed = 4;
	
	//this is the initial linear velocity, as it is increased it will create a spiral
	double rk = 0.5;

	//this is the frequency that the loop will follow, by keeping track of "sleep()"
	ros::Rate loop_rate(1);

	
	// the statement here checks that the turlte is still within the square before executing
	while((turtlesim_pose.x<x_max)&&(turtlesim_pose.y<y_max))
         {
		
		//it adds 0.5 each loop
		rk = rk + 0.5;

		//this makes the linear x change
		vel_msg.linear.x = rk;

		//this makes the the turtle spin at a constant rate
		vel_msg.angular.z = constant_speed;


		//sends out linear and angular velocities
		//cout<<"vel_msg.linear.x = "<<vel_msg.linear.x<<endl;
		//cout<<"vel_msg.angular.z = "<<vel_msg.angular.z<<endl;

		//this publishes the "vel_msg"
		velocity_publisher.publish(vel_msg);

		//this will call all the callbacks waiting to be called
		ros::spinOnce();

		
		//this allows us to sleep for the required time to keep the desired frequency
		loop_rate.sleep();

	}
	
	//this stops the turtle
	vel_msg.linear.x = 0;
	velocity_publisher.publish(vel_msg);
	cout<<"DONE "<<endl;

}



