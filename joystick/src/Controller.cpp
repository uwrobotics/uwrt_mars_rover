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
	int left_stick_up_down_,
		left_stick_left_right_,
		LT_,
		right_stick_left_right_,
		right_stick_up_down_,
		RT_,
		cross_key_left_right_,
		cross_key_up_down_;
	
	//buttons
	int A_,
		B_,
		X_,
		Y_,
		LB_,
		RB_,
		start_,
		back_,
		power_,
		button_stick_left_,
		button_stick_right_;
	
	//scales
	double  left_stick_up_down_scale_fast_,
			left_stick_up_down_scale_slow_, 
			left_stick_left_right_scale_fast_,
			left_stick_left_right_scale_slow_, 
			right_stick_up_down_scale_,
			right_stick_left_right_scale_;
	
	
	ros::Publisher drivetrain_twist_publisher_;
	ros::Subscriber joy_sub_;
};

	TeleopTurtle::TeleopTurtle():
		left_stick_up_down_(0),
		left_stick_left_right_(1),
		LT_(2),
		right_stick_left_right_(3),
		right_stick_up_down_(4),
		RT_(5),
		cross_key_left_right_(6),
		cross_key_up_down_(7),
		
		A_(0),
		B_(1),
		X_(2),
		Y_(3),
		LB_(4),
		RB_(5),
		start_(6),
		back_(7),
		power_(8),
		button_stick_left_(9),
		button_stick_right_(10)
		
	{
		//axes
		nh_.param("left_stick_vertical", left_stick_up_down_, left_stick_up_down_);
		nh_.param("left_stick_horizontal", left_stick_left_right_, left_stick_left_right_);
		nh_.param("left_T", LT_, LT_);
		nh_.param("right_stick_vertical", right_stick_up_down_, right_stick_up_down_);
		nh_.param("right_stick_horizontal", left_stick_left_right_, left_stick_left_right_);
		nh_.param("right_T", RT_,RT_);
		nh_.param("cross_key_horizontal", cross_key_left_right_, cross_key_left_right_);
		nh_.param("cross_key_vertical", cross_key_up_down_,cross_key_up_down_);
		
		//buttons
		nh_.param("A", A_ , A_);
		nh_.param("B", B_ , B_);
		nh_.param("X", X_ , X_);
		nh_.param("Y", Y_ , Y_);
		nh_.param("LB", LB_ , LB_);
		nh_.param("RB", RB_ , RB_);
		nh_.param("start", start_ , start_);
		nh_.param("back", back_ , back_);
		nh_.param("power", power_ , power_);
		nh_.param("button_stick_left", button_stick_left_ , button_stick_left_);
		nh_.param("button_stick_right", button_stick_right_ , button_stick_right_);
		nh_.param("left_stick_vertical_scale_fast", left_stick_up_down_scale_fast_, left_stick_up_down_scale_fast_);
		nh_.param("left_stick_vertical_scale_slow", left_stick_up_down_scale_slow_, left_stick_up_down_scale_slow_);
		nh_.param("left_stick_horizontal_scale_fast", left_stick_left_right_scale_fast_, left_stick_left_right_scale_fast_);
		nh_.param("left_stick_horizontal_scale_slow", left_stick_left_right_scale_slow_, left_stick_left_right_scale_slow_);
		nh_.param("right_stick_vertical_scale", right_stick_up_down_scale_, right_stick_up_down_scale_);
		nh_.param("right_stick_horizontal_scale", right_stick_left_right_scale_, right_stick_left_right_scale_);
		
		drivetrain_twist_publisher_ = nh_.advertise<geometry_msgs::Twist>("turtle1/cmd_vel", 1);
		
		joy_sub_ = nh_.subscribe<sensor_msgs::Joy>("joy", 10, &TeleopTurtle::joyCallback, this);
		
}

	void TeleopTurtle::joyCallback(const sensor_msgs::Joy::ConstPtr& joy){
		//drivetrain twist
		geometry_msgs::Twist drivetrain_twist;
		
		//when left stick is pushed, high gear
		if (button_stick_left_ == 1){
			drivetrain_twist.angular.z = left_stick_left_right_scale_fast_ * joy->axes[left_stick_left_right_];
			drivetrain_twist.linear.x = left_stick_up_down_scale_fast_ * joy->axes[left_stick_up_down_];
		} else { //low gear
			drivetrain_twist.angular.z = left_stick_left_right_scale_slow_ * joy->axes[left_stick_left_right_];
			drivetrain_twist.linear.x = left_stick_up_down_scale_slow_ * joy->axes[left_stick_up_down_];
		}
		drivetrain_twist_publisher_.publish(twist);

		//arm twist
		Twist arm_end_effector_twist;

		arm_end_eff_twist.linearx = joy->axes[cross_key_up_down_];
		arm_end_eff_twist.lineary = joy->axes[cross_key_left_right_];
		arm_end_eff_twist.linearz = joy->axes[cross_key_up_down_] == -1;
		arm_publisher.pubish(arm_end_eff_twst)

}

int main(int argc, char** argv){
	ros::init(argc, argv, "teleop_turtle");
	TeleopTurtle teleop_turtle;
	ros::spin();
}


