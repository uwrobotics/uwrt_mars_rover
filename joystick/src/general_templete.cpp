#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>

class TeleopTurtle{
	public:
	TeleopTurtle();
	
	private:
	void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);
	ros::NodeHandle nh_;
	
	//axes
	int left_stick_up_down, left_stick_left_right
	LT,right_stick_left_right, right_stick_up_down, RT,
	cross_key_left_right, cross_key_up_down;
	
	//buttons
	int A, B, X, Y, LB, RB, start, back, power, button_stick_left, button_stick_right;
	
	double left_stick_up_down_scale, left_stick_left_right_scale, right_stick_up_down_scale, right_stcik_left_right_scale;
	
	
	ros::Publisher vel_pub_;
	ros::Subscriber joy_sub_;
};

	TeleopTurtle::TeleopTurtle():
		left_stick_up_down(0),
		left_stick_left_right(1),
		LT (2),
		right_stick_left_right(3),
		right_stick_up_down(4),
		RT (5),
		cross_key_left_right(6),
		cross_key_up_down(7),
		
		A(0),
		B(1),
		X(2),
		Y(3),
		LB(4),
		RB(5),
		start(6),
		back(7),
		power(8),
		button_stick_left(9),
		button_stick_right(10)
		
	{
		//axes
		nh_.param("left_stick_vertical", left_stick_up_down, left_stick_up_down);
		nh_.param("left_stick_horizontal", left_stick_left_right, left_stick_left_right);
		nh_.param("left_T", LT, LT);
		nh_.param("right_stick_vertical", right_stick_up_down, right_stick_up_down);
		nh_.param("right_stick_horizontal", left_stick_left_right, left_stick_left_right);
		nh_.param("right_T", RT,RT);
		nh_.param("cross_key_horizontal", cross_key_left_right, cross_key_left_right);
		nh_.param("cross_key_vertical", cross_key_up_down,cross_key_up_down);
		
		nh_.param("A", A , A);
		nh_.param("B", B , B);
		nh_.param("X", X , X);
		nh_.param("Y", Y , Y);
		nh_.param("LB", LB , LB);
		nh_.param("RB", RB , RB);
		nh_.param("start", start , start);
		nh_.param("back", back , back);
		nh_.param("power", power , power);
		nh_.param("button_stick_left", button_stick_left , button_stick_left);
		nh_.param("button_stick_right", button_stick_right , button_stick_right);
		
	
		
		nh_.param("left_stick_vertical_scale", left_stick_up_down_scale, left_stick_up_down_scale);
		nh_.param("left_stick_horizontal_scale", left_stick_left_right_scale, left_stick_left_right_scale);
		nh_.param("right_stick_vertical_scale", right_stick_up_down_scale, right_stick_up_down_scale);
		nh_.param("right_stick_horizontal_scale", right_stcik_left_right_scale, right_stcik_left_right_scale);
		
		vel_pub_ = nh_.advertise<geometry_msgs::Twist>("turtle1/cmd_vel", 1);
		
		joy_sub_ = nh_.subscribe<sensor_msgs::Joy>("joy", 10, &TeleopTurtle::joyCallback, this);
		
}

	void TeleopTurtle::joyCallback(const sensor_msgs::Joy::ConstPtr& joy){
	geometry_msgs::Twist twist;
	twist.angular.z = a_scale_*joy->axes[angular_];
	twist.linear.x = l_scale_*joy->axes[linear_];
	vel_pub_.publish(twist);
	
}

int main(int argc, char** argv){
	ros::init(argc, argv, "teleop_turtle");
	TeleopTurtle teleop_turtle;
	ros::spin();
}


