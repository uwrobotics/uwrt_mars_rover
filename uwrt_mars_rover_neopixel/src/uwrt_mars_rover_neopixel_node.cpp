#include <cstdlib>

#include "uwrt_mars_rover_neopixel/uwrt_mars_rover_neopixel_can.h"
#include "uwrt_mars_rover_neopixel/uwrt_mars_rover_neopixel_control.h"

constexpr uint8_t DEFAULT_LOOP_RATE = 1;
constexpr uint16_t DEFAULT_NEOPIXEL_CAN_ID_OUTGOING = 0x784;
std::string DEFAULT_CAN_INTERFACE = "vcan0";
std::string DEFAULT_LOG_FILTER = "neopixel";

int main(int argc, char** argv) {
  ros::init(argc, argv, "neopixel_set_server");
  // Create node handle
  ros::NodeHandle nh;
  // Use Service Parameters
  int loop_rate;
  nh.param<int>("neopixel/loop_rate", loop_rate, DEFAULT_LOOP_RATE);
  std::string can_interface;
  nh.param<std::string>("neopixel/can_interface", can_interface, DEFAULT_CAN_INTERFACE);
  std::string log_filter;
  nh.param<std::string>("neopixel/log_filter", log_filter, DEFAULT_LOG_FILTER);
  int neopixel_can_id_outgoing;
  nh.param<int>("neopixel/can_id_outgoing", neopixel_can_id_outgoing, DEFAULT_NEOPIXEL_CAN_ID_OUTGOING);
  // Run the node
  Neopixel obj(nh, static_cast<uint8_t>(loop_rate), can_interface, log_filter,
               static_cast<uint16_t>(neopixel_can_id_outgoing));
  obj.run();
  return EXIT_SUCCESS;
}
