
#include "ros/ros.h"
#include "std_msgs/Float32MultiArray.h"
#include "uwrt_mars_rover_utils/hw_bridge.h"
#include "uwrt_mars_rover_utils/uwrt_can.h"

class controller {
 public:
  controller() = delete;
  explicit controller(std::string _name);
  ~controller();

  ros::Rate loop_rate{10};

  inline std::string getName() const {
    return _name;
  }

  void scienceCallback(const std_msgs::Float32MultiArrayConstPtr& msg);

 private:
  // name of controller
  std::string _name;

  // Subs
  ros::Subscriber science_sub;

  ros::NodeHandle nh;

  // CAN
  uwrt_mars_rover_utils::UWRTCANWrapper* comm;

  // constants
  static constexpr int QUERY{1000};
};

controller::controller(std::string) : _name(_name) {
  science_sub =
      nh.subscribe<std_msgs::Float32MultiArray>("/science_sar_commands", QUERY, &controller::scienceCallback, this);

  // set up CAN
  comm = new uwrt_mars_rover_utils::UWRTCANWrapper(_name, "can0", false);
  comm->init(std::vector<uint32_t>());  // empty list because no receive
}

controller::~controller() {
  delete comm;
  comm = nullptr;
}

void controller::scienceCallback(const std_msgs::Float32MultiArrayConstPtr& msg) {
  static const unsigned NUM_JOINTS = 4;
  ROS_ASSERT(msg->layout.dim[0].size == NUM_JOINTS);
  ROS_ASSERT(msg->layout.dim[0].stride == NUM_JOINTS);

  std::stringstream ss;
  for (int i = 0; i < NUM_JOINTS; i++) {
    ss << msg->data[i] << " ";
  }
  ROS_INFO_STREAM("data being sent: " << ss.str());

  bool success = true;
  success &= comm->writeToID<float>(msg->data[0],
                                                 static_cast<uint32_t>(HWBRIDGE::CANID::SET_COVER_ANGLE));
  success &= comm->writeToID<float>(msg->data[1],
                                                 static_cast<uint32_t>(HWBRIDGE::CANID::SET_GENEVA_ANGLE));
  success &=
      comm->writeToID<float>(msg->data[2], static_cast<uint32_t>(HWBRIDGE::CANID::SET_ELEVATOR_HEIGHT));
  success &= comm->writeToID<float>(msg->data[3],
                                                 static_cast<uint32_t>(HWBRIDGE::CANID::SET_SCOOPER_ANGLE));

  ROS_ERROR_STREAM_COND(!success, "ERROR unable to send science msgs");
  ROS_INFO_STREAM_COND(success, "Science msgs sent");
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "science_control_node");
  controller science = controller("science_controller");

  while (ros::ok()) {
    ros::spinOnce();
    science.loop_rate.sleep();
  }

  return 0;
}
