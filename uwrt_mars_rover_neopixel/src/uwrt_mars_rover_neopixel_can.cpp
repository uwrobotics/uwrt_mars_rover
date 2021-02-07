#include <uwrt_mars_rover_neopixel/uwrt_mars_rover_neopixel_can.h>

constexpr uint16_t NEOPIXEL_CAN_ID_INCOMING = 0x785;

NeopixelCan::NeopixelCan(uint16_t can_id_outgoing, uint8_t dlc, const std::string &name, std::string log_filter)
    : log_filter_(std::move(log_filter)) 
{
  // Prepare the outgoing can packet
  outgoing_can_frame_.can_id = can_id_outgoing;
  outgoing_can_frame_.can_dlc = dlc;
  // General work for socket binding
  if ((socket_handle_ = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0) 
  {
    ROS_ERROR("Error while opening socket\n");
    throw std::runtime_error("Error while opening socket in " __FILE__);
  }
  const std::string &can_interface_name{name};
  ifreq interface_request{};
  strcpy(interface_request.ifr_name, can_interface_name.c_str());
  ioctl(socket_handle_, SIOCGIFINDEX, &interface_request);

  sockaddr_can socket_addr{};
  socket_addr.can_family = AF_CAN;
  socket_addr.can_ifindex = interface_request.ifr_ifindex;

  // Bind socket CAN to an interface
  // NOLINTNEXTLINE(cppcoreguidelines-pro-type-reinterpret-cast): reinterpret cast required by syscall
  if (bind(socket_handle_, reinterpret_cast<struct sockaddr *>(&socket_addr), sizeof(socket_addr)) < 0) 
  {
    ROS_ERROR("Error in socket bind");
    throw std::runtime_error("Error in socket bind in " __FILE__);
  }
}
void NeopixelCan::sendCAN(const uint8_t data) 
{
  // Store data into the data potion of the CAN packet
  outgoing_can_frame_.data[0] = data;
  // Send out the CAN packet
  write(socket_handle_, &outgoing_can_frame_, sizeof(outgoing_can_frame_));
}
bool NeopixelCan::waitforAck() 
{
  ROS_INFO_NAMED(log_filter_, "Now waiting for acknowledgement message.");
  do 
  {
    recv(socket_handle_, &incoming_can_frame_, sizeof(incoming_can_frame_), 0);
    ROS_INFO_NAMED(log_filter_, "CAN message received. Checking validity...");
    ROS_INFO_COND_NAMED(incoming_can_frame_.can_id != NEOPIXEL_CAN_ID_INCOMING || incoming_can_frame_.data[0] != 1,
                        log_filter_,
                        "The received CAN message did not match the expected acknowledgment message. Waiting again.");
  } while (incoming_can_frame_.can_id != NEOPIXEL_CAN_ID_INCOMING || incoming_can_frame_.data[0] != 1);
  ROS_INFO_NAMED(log_filter_, "The expected acknowledgement message was received.");
  return true;
  // for now, this function cannot time out and return false
  // this will be implemented in a wrapper lib for can later
}
