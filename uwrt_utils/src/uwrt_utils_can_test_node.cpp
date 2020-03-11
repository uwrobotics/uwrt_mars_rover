#include <uwrt_utils/uwrt_can.h>

#include <string>
#include <vector>

#include "ros/ros.h"
#include "std_msgs/Float32.h"
#include "std_msgs/UInt32.h"

// ids
static constexpr uint32_t FLOAT_READ_ID1 = 0x001;
static constexpr uint32_t FLOAT_READ_ID2 = 0x002;
static constexpr uint32_t UINT_READ_ID1 = 0x003;
static constexpr uint32_t UINT_READ_ID2 = 0x004;
static constexpr uint32_t FLOAT_WRITE_ID = 0x005;
static constexpr uint32_t UINT_WRITE_ID = 0x006;

// ros constants
static constexpr int TOPIC_BUFFER_SIZE = 10;
static constexpr int LOOP_RATE = 10;

// two can_wrappers to make sure we can create multiple
uwrt_utils::UWRTCANWrapper can_wrapper_int;    // NOLINT(readability-identifier-naming)
uwrt_utils::UWRTCANWrapper can_wrapper_float;  // NOLINT(readability-identifier-naming)

// callback to get new float from topic, and send it over can
void sendCanFloatCallback(const std_msgs::Float32::ConstPtr& data) {
  auto msg = (float)data->data;
  if (can_wrapper_float.writeToID<float>(msg, FLOAT_WRITE_ID)) {
    ROS_INFO_STREAM("Successfully sent float msg " << msg << " to id 0x05");
  } else {
    ROS_INFO_STREAM("Failed to send float msg " << msg << " to id 0x05");
  }
}

// callback to get new int from topic, and send it over can
void sendCanUIntCallback(const std_msgs::UInt32::ConstPtr& data) {
  auto msg = (uint32_t)data->data;
  if (can_wrapper_int.writeToID<uint32_t>(msg, UINT_WRITE_ID)) {
    ROS_INFO_STREAM("Successfully sent uint32_t msg " << msg << " to id 0x05");
  } else {
    ROS_INFO_STREAM("Failed to send uint32_t msg " << msg << " to id 0x05");
  }
}

int main(int argc, char* argv[]) {  // NOLINT(bugprone-exception-escape)
  // init ROS
  ros::init(argc, argv, "uwrt_utils_tester");
  ros::NodeHandle nh;

  // get params for CAN wrapper, and init it
  std::string can_interface;
  if (!nh.getParam("can_interface", can_interface)) {
    ROS_ERROR_STREAM("Could not find name for CAN interface");
    return EXIT_FAILURE;
  }

  // construct both can wrappers
  can_wrapper_int = uwrt_utils::UWRTCANWrapper("can_test_int", can_interface, true);
  can_wrapper_float = uwrt_utils::UWRTCANWrapper("can_test_float", can_interface, true);

  // can ids for either wrapper
  std::vector<uint32_t> float_ids;
  float_ids.push_back(FLOAT_READ_ID1);
  float_ids.push_back(FLOAT_READ_ID2);
  std::vector<uint32_t> uint_ids;
  uint_ids.push_back(UINT_READ_ID1);
  uint_ids.push_back(UINT_READ_ID2);

  // init both can wrappers
  can_wrapper_int.init(uint_ids);
  can_wrapper_float.init(float_ids);

  // publisher and subscriber for testing CAN library
  ros::Publisher can_pub = nh.advertise<std_msgs::Float32>("recv_can_msgs", TOPIC_BUFFER_SIZE);
  ros::Subscriber can_float_sub =
      nh.subscribe<std_msgs::Float32>("send_can_float_msgs", TOPIC_BUFFER_SIZE, sendCanFloatCallback);
  ros::Subscriber can_uint_sub =
      nh.subscribe<std_msgs::UInt32>("send_can_uint_msgs", TOPIC_BUFFER_SIZE, sendCanUIntCallback);

  // main node loop
  ros::Rate loop_rate(LOOP_RATE);
  while (ros::ok()) {
    // try and read float ids
    for (const auto& id : float_ids) {
      float data;
      if (can_wrapper_float.getLatestFromID<float>(data, id)) {
        ROS_INFO_STREAM("Got float " << data << " from id " << id);
      }
    }

    // try and read float ids
    for (const auto& id : uint_ids) {
      uint32_t data;
      if (can_wrapper_int.getLatestFromID<uint32_t>(data, id)) {
        ROS_INFO_STREAM("Got uint32_t " << data << " from id " << id);
      }
    }

    ros::spinOnce();
    loop_rate.sleep();
  }

  return EXIT_SUCCESS;
}
// namespace uwrt_utils