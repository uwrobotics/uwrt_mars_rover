#include <string>
#include <vector>

#include "ros/ros.h"
#include "std_msgs/Float32.h"
#include "std_msgs/UInt32.h"

#include <uwrt_utils/uwrt_can.h>

uwrt_utils::UWRTCANWrapper can_wrapper;

void sendCanFloatCallback(const std_msgs::Float32::ConstPtr& data) {
  auto msg = (float)data->data;
  uwrt_utils::UWRTCANWrapper::UWRTCANStatus status = can_wrapper.writeToID<float>(msg, (uint32_t)0x05);
  if (status == uwrt_utils::UWRTCANWrapper::UWRTCANStatus::STATUS_OK) {
    ROS_INFO_STREAM("Successfully sent float msg " << msg << " to id 0x05");
  } else {
    ROS_INFO_STREAM("Failed to send float msg " << msg << " to id 0x05");
  }
}

void sendCanUIntCallback(const std_msgs::UInt32::ConstPtr& data) {
  auto msg = (uint32_t)data->data;
  uwrt_utils::UWRTCANWrapper::UWRTCANStatus status = can_wrapper.writeToID<uint32_t>(msg, 0x05);
  if (status == uwrt_utils::UWRTCANWrapper::UWRTCANStatus::STATUS_OK) {
    ROS_INFO_STREAM("Successfully sent uint32_t msg " << msg << " to id 0x05");
  } else {
    ROS_INFO_STREAM("Failed to send uint32_t msg " << msg << " to id 0x05");
  }
}

int main(int argc, char *argv[]) {
  // init ROS
  ros::init(argc, argv, "uwrt_utils_tester");
  ros::NodeHandle nh;

  // get params for CAN wrapper, and init it
  std::string can_interface;
  if (!nh.getParam("can_interface", can_interface)) {
    ROS_ERROR_STREAM("Could not find name for CAN interface");
    return EXIT_FAILURE;
  }
  uwrt_utils::UWRTCANWrapper can_wrapper("can_test", can_interface, 1);
  std::vector<uint32_t> float_ids;
  float_ids.push_back(0x01);
  float_ids.push_back(0x02);
  std::vector<uint32_t> uint_ids;
  uint_ids.push_back(0x03);
  uint_ids.push_back(0x04);

  std::vector<uint32_t> all_ids;
  all_ids.reserve(float_ids.size() + uint_ids.size());
  all_ids.insert(all_ids.end(), float_ids.begin(), float_ids.end());
  all_ids.insert(all_ids.end(), uint_ids.begin(), uint_ids.end());

  can_wrapper.init(all_ids);

  // publisher and subscriber for testing CAN library
  ros::Publisher can_pub = nh.advertise<std_msgs::Float32>("recv_can_msgs", 10);
  ros::Subscriber can_float_sub = nh.subscribe<std_msgs::Float32>("send_can_float_msgs", 10, sendCanFloatCallback);
  ros::Subscriber can_uint_sub = nh.subscribe<std_msgs::UInt32>("send_can_uint_msgs", 10, sendCanUIntCallback);

  // main node loop
  ros::Rate loop_rate(10);
  while(ros::ok()) {
    
    // try and read float ids
    for (const auto& id : float_ids) {
      float data;
      if(can_wrapper.getLatestFromID<float>(data, id) == uwrt_utils::UWRTCANWrapper::UWRTCANStatus::STATUS_OK) {
        ROS_INFO_STREAM("Got float " << data << " from id " << id);
      }
    }

     // try and read float ids
    for (const auto& id : uint_ids) {
      uint32_t data;
      if(can_wrapper.getLatestFromID<uint32_t>(data, id) == uwrt_utils::UWRTCANWrapper::UWRTCANStatus::STATUS_OK) {
        ROS_INFO_STREAM("Got uint32_t " << data << " from id " << id);
      }
    }

    ros::spinOnce();
    loop_rate.sleep();
  }

  return EXIT_SUCCESS;
}
// namespace uwrt_utils