#include "uwrt_mars_rover_neopixel/uwrt_mars_rover_neopixel_can.h"

constexpr uint16_t NEOPIXEL_CAN_ID_INCOMING = 0x785;

NeopixelCan::NeopixelCan(uint16_t can_id_outgoing, uint8_t dlc, const std::string &name, std::string log_filter)
    : _addr{}, _ifr{}, _log_filter(std::move(log_filter)) {
  // Prepare the outgoing can packet
  _outgoing_packet.can_id = can_id_outgoing;
  _outgoing_packet.can_dlc = dlc;
  // General work for socket binding
  _ifname = name;
  if ((_s = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0) {
    ROS_ERROR("Error while opening socket\n");
    throw - 1;
  }
  strcpy(_ifr.ifr_name, _ifname.c_str());
  ioctl(_s, SIOCGIFINDEX, &_ifr);
  _addr.can_family = AF_CAN;
  _addr.can_ifindex = _ifr.ifr_ifindex;
  // Bind socket CAN to an interface
  // NOLINTNEXTLINE(cppcoreguidelines-pro-type-reinterpret-cast): reinterpret cast required by syscall
  if (bind(_s, reinterpret_cast<struct sockaddr *>(&_addr), sizeof(_addr)) < 0) {
    ROS_ERROR("Error in socket bind");
    throw - 2;
  }
}
void NeopixelCan::sendCAN(const uint8_t data) {
  // Store data into the data potion of the CAN packet
  _outgoing_packet.data[0] = data;
  // Send out the CAN packet
  write(_s, &_outgoing_packet, sizeof(_outgoing_packet));
}
bool NeopixelCan::waitforAck() {
  ROS_INFO_NAMED(_log_filter, "Now waiting for acknowledgement message.");
  do {
    recv(_s, &_incoming_packet, sizeof(_incoming_packet), 0);
    ROS_INFO_NAMED(_log_filter, "CAN message received. Checking validity...");
    ROS_INFO_COND_NAMED(_incoming_packet.can_id != NEOPIXEL_CAN_ID_INCOMING || _incoming_packet.data[0] != 1,
                        _log_filter,
                        "The received CAN message did not match the expected acknowledgment message. Waiting again.");
  } while (_incoming_packet.can_id != NEOPIXEL_CAN_ID_INCOMING || _incoming_packet.data[0] != 1);
  ROS_INFO_NAMED(_log_filter, "The expected acknowledgement message was received.");
  return true;
  // for now, this function cannot time out and return false
  // this will be implemented in a wrapper lib for can later
}
