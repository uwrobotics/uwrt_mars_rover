#include "ros/assert.h"
#include "ros/ros.h"
#include "std_msgs/Float32MultiArray.h"
#include "uwrt_mars_rover_utils/hw_bridge.h"
#include "uwrt_mars_rover_utils/uwrt_can.h"

// This code is PURE garbage and was written in 30 mins for a demo....... NEVER MERGE THIS INTO MASTER

uwrt_mars_rover_utils::UWRTCANWrapper can_wrapper_handle("can_test_int", "can0", false);

// @azum: multiarray published docs if u need it:
// https://answers.ros.org/question/226726/push-vector-into-multiarray-message-and-publish-it/
/**
 * Example on how to fake a multiarray with rostopic pub
 * rostopic pub /science_sar_commands std_msgs/Float32MultiArray "layout:
 *   dim:
 *   - label: ''
 *     size: 7
 *     stride: 7
 *   data_offset: 0
 * data: [1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0]" -r 1
 */
void scienceCallback(const std_msgs::Float32MultiArrayConstPtr& msg) {
  static const unsigned NUM_JOINTS = 4;
  ROS_ASSERT(msg->layout.dim[0].size == NUM_JOINTS);
  ROS_ASSERT(msg->layout.dim[0].stride == NUM_JOINTS);

  std::stringstream ss;
  for (int i = 0; i < NUM_JOINTS; i++) {
    ss << msg->data[i] << " ";
  }
  ROS_INFO_STREAM("data being sent: " << ss.str());

  bool success = true;
  success &= can_wrapper_handle.writeToID<float>(msg->data[0],
                                                 static_cast<uint32_t>(HWBRIDGE::CANID::SET_COVER_ANGLE));
  success &= can_wrapper_handle.writeToID<float>(msg->data[1],
                                                 static_cast<uint32_t>(HWBRIDGE::CANID::SET_GENEVA_ANGLE));
  success &=
      can_wrapper_handle.writeToID<float>(msg->data[2], static_cast<uint32_t>(HWBRIDGE::CANID::SET_ELEVATOR_HEIGHT));
  success &= can_wrapper_handle.writeToID<float>(msg->data[3],
                                                 static_cast<uint32_t>(HWBRIDGE::CANID::SET_SCOOPER_ANGLE));

  ROS_ERROR_STREAM_COND(!success, "ERROR unable to send science msgs");
  ROS_INFO_STREAM_COND(success, "Science msgs sent");
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "science_node");
  ros::NodeHandle nh;

  can_wrapper_handle.init(std::vector<uint32_t>());  // hack empty recv list cuz not reading anything

  ros::Subscriber science_sub = nh.subscribe("/science_sar_commands", 1000, scienceCallback);
  ros::Rate loop_rate(100);
  while (ros::ok()) {
    ros::spinOnce();
    loop_rate.sleep();
  }
}
