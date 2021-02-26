
#include "ros/ros.h"
#include "std_msgs/Float32.h"
#include "uwrt_mars_rover_utils/hw_bridge.h"
#include "uwrt_mars_rover_utils/uwrt_can.h"

class controller {
 public:
  controller() = delete;
  explicit controller(std::string _name);
  ~controller();

  bool send_commands(void);

  ros::Rate loop_rate{10};

  inline std::string getName() const {
    return _name;
  }

 private:
  // name of controller
  std::string _name;

  // Subs
  ros::Subscriber cap_servo;
  ros::Subscriber geneva_motor;
  ros::Subscriber elevator_motor;
  ros::Subscriber shovel_servo;

  ros::NodeHandle nh;

  // Callback functions
  void cap_servo_callback(const std_msgs::Float32::ConstPtr& msg);
  void geneva_motor_callback(const std_msgs::Float32::ConstPtr& msg);
  void elevator_motor_callback(const std_msgs::Float32::ConstPtr& msg);
  void shovel_servo_callback(const std_msgs::Float32::ConstPtr& msg);

  // variables to write to
  float index_cmds;
  float cap_cmds;
  float shovel_cmds;
  float elevator_cmds;

  // CAN
  uwrt_mars_rover_utils::UWRTCANWrapper* comm;

  // constants
  static constexpr int QUERY{1000};
};

controller::controller(std::string) : _name(_name) {
  elevator_motor =
      nh.subscribe<std_msgs::Float32>("/science/elevator", QUERY, &controller::elevator_motor_callback, this);
  cap_servo = nh.subscribe<std_msgs::Float32>("/science/cap", QUERY, &controller::cap_servo_callback, this);
  geneva_motor = nh.subscribe<std_msgs::Float32>("/science/indexer", QUERY, &controller::geneva_motor_callback, this);
  shovel_servo = nh.subscribe<std_msgs::Float32>("/science/shovel", QUERY, &controller::shovel_servo_callback, this);
  // set up CAN
  comm = new uwrt_mars_rover_utils::UWRTCANWrapper(_name, "can0", false);
  comm->init(std::vector<uint32_t>());  // empty list because no receive
}

controller::~controller() {
  delete comm;
  comm = nullptr;
}
void controller::cap_servo_callback(const std_msgs::Float32::ConstPtr& msg) {
  cap_cmds = msg->data;
}

void controller::geneva_motor_callback(const std_msgs::Float32::ConstPtr& msg) {
  index_cmds = msg->data;
}

void controller::elevator_motor_callback(const std_msgs::Float32::ConstPtr& msg) {
  elevator_cmds = msg->data;
}

void controller::shovel_servo_callback(const std_msgs::Float32::ConstPtr& msg) {
  shovel_cmds = msg->data;
}

bool controller::send_commands(void) {
  // write commands to CAN
  if (!comm->writeToID<float>(cap_cmds, static_cast<uint32_t>(HWBRIDGE::CANID::SET_COVER_ANGLE))) {
    ROS_ERROR_STREAM_NAMED(_name, "CAN MESSAGE FAILED TO SEND TO CAP");
    return false;
  }
  if (!comm->writeToID<float>(index_cmds, static_cast<uint32_t>(HWBRIDGE::CANID::SET_GENEVA_INDEX))) {
    ROS_ERROR_STREAM_NAMED(_name, "CAN MESSAGE FAILED TO SEND TO GENENVA INDEXER");
    return false;
  }
  if (!comm->writeToID<float>(elevator_cmds, static_cast<uint32_t>(HWBRIDGE::CANID::SET_ELEVATOR_HEIGHT))) {
    ROS_ERROR_STREAM_NAMED(_name, "CAN MESSAGE FAILED TO SEND TO ELEVATOR");
    return false;
  }
  if (!comm->writeToID<float>(shovel_cmds, static_cast<uint32_t>(HWBRIDGE::CANID::SET_SCOOPER_ANGLE))) {
    ROS_ERROR_STREAM_NAMED(_name, "CAN MESSAGE FAILED TO SEND TO SHOVEL");
    return false;
  }
  return true;
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "science_control_node");
  controller science = controller("science controller");

  while (ros::ok) {
    if (!science.send_commands()) {
      ROS_ERROR_STREAM_NAMED(science.getName(), "Failed to send CAN message");
    }
    ros::spinOnce();
    science.loop_rate.sleep();
  }

  return 0;
}
