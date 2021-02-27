// left is 1 right is -1
#include <geometry_msgs/Twist.h>
#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/Float32MultiArray.h>

#include <vector>
#include <cmath>

typedef enum {
      DRIVETRAIN,
      ARM_CONTROL,
      WRIST_CONTROL,
      SCIENCE_CONTROL
} e_control_mode;

class XboxController {
 public:
  XboxController(std::string name);
  float helper(int & index, int num_joints, const sensor_msgs::Joy::ConstPtr& joy);

  void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);

 private:
  ros::NodeHandle nh_;

  e_control_mode econtrol_mode;

  std::vector<std::string> modes;
  std::vector<std::string> arm_joint_names;
  std::vector<std::string> science_joint_names;
  std::vector<std::string> wrist_joint_names;

  std::string node_name;

  bool emergency_stop;

  int arm_joint, science_joint;

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
  ros::Publisher science_publisher;

  ros::Subscriber joy_sub_;
};

XboxController::XboxController(std::string name)
    : left_stick_up_down_(1),
      left_stick_left_right_(0),
      LT_(5),
      right_stick_left_right_(2),
      right_stick_up_down_(3),
      RT_(4),
      cross_key_left_right_(6),
      cross_key_up_down_(7),

      A_(0),
      B_(1),
      X_(2),
      Y_(3),
      LB_(4),
      RB_(5),
      start_(7),
      back_(6),
      power_(8),
      button_stick_left_(9),
      button_stick_right_(10),

      left_stick_left_right_scale_fast_(1.5),
      left_stick_up_down_scale_fast_(5),
      left_stick_left_right_scale_slow_(0.5),
      left_stick_up_down_scale_slow_(2),

      econtrol_mode(DRIVETRAIN),

      arm_joint(0),
      science_joint(0),

      emergency_stop(false),
      node_name(std::move(name))

{
  // axes
  nh_.param("left_stick_vertical", left_stick_up_down_, left_stick_up_down_);
  nh_.param("left_stick_horizontal", left_stick_left_right_, left_stick_left_right_);
  nh_.param("left_T", LT_, LT_);
  nh_.param("right_stick_vertical", right_stick_up_down_, right_stick_up_down_);
  nh_.param("right_stick_horizontal", right_stick_left_right_, right_stick_left_right_);
  nh_.param("right_T", RT_, RT_);
  nh_.param("cross_key_horizontal", cross_key_left_right_, cross_key_left_right_);
  nh_.param("cross_key_vertical", cross_key_up_down_, cross_key_up_down_);

  // buttons
  nh_.param("A", A_, A_);
  nh_.param("B", B_, B_);
  nh_.param("X", X_, X_);
  nh_.param("Y", Y_, Y_);
  nh_.param("LB", LB_, LB_);
  nh_.param("RB", RB_, RB_);
  nh_.param("start", start_, start_);
  nh_.param("back", back_, back_);
  nh_.param("power", power_, power_);
  nh_.param("button_stick_left", button_stick_left_, button_stick_left_);
  nh_.param("button_stick_right", button_stick_right_, button_stick_right_);
  nh_.param("left_stick_vertical_scale_fast", left_stick_up_down_scale_fast_, left_stick_up_down_scale_fast_);
  nh_.param("left_stick_vertical_scale_slow", left_stick_up_down_scale_slow_, left_stick_up_down_scale_slow_);
  nh_.param("left_stick_angular_scale_fast", left_stick_left_right_scale_fast_, left_stick_left_right_scale_fast_);
  nh_.param("left_stick_angular_scale_slow", left_stick_left_right_scale_slow_, left_stick_left_right_scale_slow_);
  nh_.param("right_stick_vertical_scale", right_stick_up_down_scale_, right_stick_up_down_scale_);
  nh_.param("right_stick_angular_scale", right_stick_left_right_scale_, right_stick_left_right_scale_);

  modes = {"DRIVETRAIN", "ARM", "WRIST", "SCIENCE"};
  arm_joint_names = {"TURNTABLE", "SHOULDER", "ELBOW", "CLAW", "TOOL_TIP", "LEFT_WRIST", "RIGHT_WRIST"};
  science_joint_names = {"COVER_ANGLE", "GENEVA_INDEX", "ELEVATOR", "SCOOPER"};

  drivetrain_twist_publisher_ =
      nh_.advertise<geometry_msgs::Twist>("/uwrt_mars_rover/drivetrain_velocity_controller/cmd_vel", 1);
  arm_publisher = nh_.advertise<std_msgs::Float32MultiArray>("/sar_arm_commands", 1);
  science_publisher = nh_.advertise<std_msgs::Float32MultiArray>("/science_sar_commands", 1);
  //  claw_publisher = nh_.advertise<geometry_msgs::Twist>("placeholder/cmd_vel", 1);
  //  camera_publisher = nh_.advertise<geometry_msgs::Twist>("placeholder/cmd_vel", 1);

  joy_sub_ = nh_.subscribe<sensor_msgs::Joy>("joy", 10, &XboxController::joyCallback, this);
}

void XboxController::joyCallback(const sensor_msgs::Joy::ConstPtr& joy) {
  // check if we changed control mode
  // load control mode
    int arm_num_joints = 7;
    int science_num_joints = 4;
    int left_wrist = 5;
    int right_wrist = 6;
    float command;
    bool switch_mode = false;

    std::vector<float> arm_data, science_data;
    std_msgs::Float32MultiArray arm_command, science_command;
    std_msgs::MultiArrayDimension dim;

    arm_data.resize(arm_num_joints);
    science_data.resize(science_num_joints);

    if (joy->buttons[Y_] == 1) {
        switch_mode = true;
    } else {
        switch_mode = false;
    }

    if (joy->buttons[X_] == 1) {
        // emergency stop, dont send commands
        emergency_stop = !emergency_stop;
    }

    if (!emergency_stop) {
        switch(econtrol_mode) {
            case DRIVETRAIN:
                if (switch_mode) {
                    econtrol_mode = ARM_CONTROL;
                    ROS_INFO_STREAM_NAMED(node_name, "Switched control mode to: " << modes[econtrol_mode]);
                    switch_mode = false;
                }
                break;
            case ARM_CONTROL:
                dim.stride = arm_num_joints;
                dim.size = arm_num_joints;

                // cycle through joints
                command = helper(arm_joint, arm_num_joints - 2, joy);
                
                ROS_INFO_STREAM_NAMED(node_name, "Now controlling: " << arm_joint_names[arm_joint]);

                arm_data[arm_joint] = command;

                arm_command.layout.dim.push_back(dim);
                arm_command.data = arm_data;

                arm_publisher.publish(arm_command);

                if (switch_mode) {
                    econtrol_mode = WRIST_CONTROL;
                    ROS_INFO_STREAM_NAMED(node_name, "Switched control mode to: " << modes[econtrol_mode]);
                    switch_mode = false;
                }
                
                break;
            case WRIST_CONTROL:
                // we need to move entire wrist up or turn the wrist
                dim.stride = arm_num_joints;
                dim.size = arm_num_joints;

                if (joy->buttons[A_] == 1) {
                    ROS_INFO_STREAM_NAMED(node_name, "Controlling the wrist in raise mode.");
                    command = joy->axes[right_stick_up_down_];
                    arm_data[left_wrist] = command;
                    arm_data[right_wrist] = command;
                } else if (joy->buttons[A_] == 0) {
                    ROS_INFO_STREAM_NAMED(node_name, "Controlling the wrist in rotate mode.");
                    command = joy->axes[right_stick_left_right_];
                    arm_data[left_wrist] = command;
                    arm_data[right_wrist] = -command;
                }

                arm_command.layout.dim.push_back(dim);
                arm_command.data = arm_data;

                arm_publisher.publish(arm_command);

                if (switch_mode) {
                    econtrol_mode = SCIENCE_CONTROL;
                    ROS_INFO_STREAM_NAMED(node_name, "Switched control mode to: " << modes[econtrol_mode]);
                    switch_mode = false;
                }

                break;
            case SCIENCE_CONTROL:
                dim.stride = science_num_joints;
                dim.size = science_num_joints;

                // cycle through joints
                command = helper(science_joint, science_num_joints, joy);
                ROS_INFO_STREAM_NAMED(node_name, "Now controlling: " << science_joint_names[science_joint]);

                // are we a servo or a motor
                if (science_joint == 0 || science_joint == 3) {
                    // map -1 to 1 to -pi to pi?? TODO SAR: change depending on servo
                    command = command * M_PI;
                }

                science_data[science_joint] = command;

                science_command.layout.dim.push_back(dim);
                science_command.data = science_data;

                science_publisher.publish(science_command);

                if (switch_mode) {
                    econtrol_mode = DRIVETRAIN;
                    ROS_INFO_STREAM_NAMED(node_name, "Switched control mode to: " << modes[econtrol_mode]);
                    switch_mode = false;
                }

                break;
            default:
                break;
        };

        // drivetrain twist
        geometry_msgs::Twist drivetrain_twist;

        // when left stick is pushed, high gear
        if (joy->buttons[button_stick_left_] == 1) {
            drivetrain_twist.angular.z = left_stick_left_right_scale_fast_ * joy->axes[left_stick_left_right_];
            drivetrain_twist.linear.x = left_stick_up_down_scale_fast_ * joy->axes[left_stick_up_down_];
        }
        if (joy->buttons[button_stick_left_] == 0) {  // low gear
            drivetrain_twist.angular.z = left_stick_left_right_scale_slow_ * joy->axes[left_stick_left_right_];
            drivetrain_twist.linear.x = left_stick_up_down_scale_slow_ * joy->axes[left_stick_up_down_];
        }
        drivetrain_twist_publisher_.publish(drivetrain_twist);
        ROS_INFO_STREAM_NAMED(node_name, "Xbox sent commands to the rover.");
    }
    else 
    {
        ROS_WARN_STREAM_NAMED(node_name, "EMERGENCY: STOPPING THE ROVER!!!");
        geometry_msgs::Twist zero_twist;

        zero_twist.linear.x = 0;
        zero_twist.angular.z = 0;

        drivetrain_twist_publisher_.publish(zero_twist);

        // send all 0s to arm
        dim.stride = arm_num_joints;
        dim.size = arm_num_joints;

        arm_command.layout.dim.push_back(dim);

        arm_command.data = arm_data;
        
        // send all 0s to science
        dim.stride = science_num_joints;
        dim.size = science_num_joints;

        science_command.layout.dim.push_back(dim);
        science_command.data = science_data;

        science_publisher.publish(science_command);
        arm_publisher.publish(arm_command);
    }
}

float XboxController::helper(int & index, int num_joints, const sensor_msgs::Joy::ConstPtr& joy ) {
    if (joy->buttons[RB_]) {
        index = ( index + 1 ) % num_joints;
    } else if (joy->buttons[LB_]) {
        index = index == 0 ? num_joints - 1 : ( index - 1 ) % num_joints;
    }
    return joy->axes[right_stick_up_down_];
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "xbox_controller");
  XboxController xbox_controller("xbox_controller");
  ros::spin();
}