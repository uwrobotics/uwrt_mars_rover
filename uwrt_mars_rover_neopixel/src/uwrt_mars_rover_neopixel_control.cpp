#include "uwrt_mars_rover_neopixel/uwrt_mars_rover_neopixel_control.h"

constexpr uint8_t FRAME_PAYLOAD_LENGTH = 1;

Neopixel::Neopixel(ros::NodeHandle &nh, uint8_t loop_rate, const std::string &can_interface,
                   const std::string &log_filter, uint16_t neopixel_can_id_outgoing)
    : _nh(nh),
      _loop_rate(loop_rate),
      _can_interface(can_interface),
      _log_filter(log_filter),
      _neopixel_can_id_outgoing(neopixel_can_id_outgoing),
      _neopixel_can_msg(neopixel_can_id_outgoing, FRAME_PAYLOAD_LENGTH, can_interface, log_filter) {}
bool Neopixel::setState(uwrt_mars_rover_msgs::set_state::Request &req, uwrt_mars_rover_msgs::set_state::Response &res) {
  ROS_INFO_NAMED(_log_filter, "Neopixel callback function triggered.");
  _state = req.requested_mode.value;
  _neopixel_can_msg.sendCAN(_state);
  if (_neopixel_can_msg.waitforAck()) {
    ROS_DEBUG_NAMED(_log_filter, "Received acknowledgement from gimbtonomy board.");
    // git CI insists on using 1u
    res.success = 1U;
  } else {
    // As currently implemented, this will never run. See waitforAck() for more details
    ROS_ERROR("Unexpected acknowledgement message received.");
    res.success = 0U;
  }
  return res.success != 0U;
}
void Neopixel::run() {
  // object used for advertising service
  _ss = _nh.advertiseService("neopixel_set", &Neopixel::setState, this);
  ROS_INFO_NAMED(_log_filter, "Ready to update neopixel state.");
  while (ros::ok()) {
    ros::spinOnce();
    _loop_rate.sleep();
  }
}