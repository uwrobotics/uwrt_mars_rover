#include <cstdlib>

#include "uwrt_mars_rover_neopixel/uwrt_mars_rover_neopixel_can.h"
#include "uwrt_mars_rover_neopixel/uwrt_mars_rover_neopixel_control.h"

constexpr uint8_t DEFAULT_LOOP_RATE = 1;
constexpr uint16_t DEFAULT_NEOPIXEL_CAN_ID_OUTGOING = 0x784;
std::string DEFAULT_CAN_INTERFACE = "vcan0";
std::string DEFAULT_LOG_FILTER = "neopixel";

int main(int argc, char** argv) {
  /*std::string can_interface = "vcan0";
  std::string log_filter = "neopixel_debug";*/
  ros::init(argc, argv, "neopixel_set_server");
  // Create node handle
  ros::NodeHandle nh;
  // Use Service Parameters
  uint8_t loop_rate;
  nh.param<uint8_t>("neopixel/loop_rate", loop_rate, DEFAULT_LOOP_RATE);
  std::string can_interface;
  nh.param<std::string>("neopixel/can_interface", can_interface, DEFAULT_CAN_INTERFACE);
  std::string log_filter;
  nh.param<std::string>("neopixel/log_filter", log_filter, DEFAULT_LOG_FILTER);
  uint16_t neopixel_can_id_outgoing;
  nh.param<uint16_t>("neopixel/can_id_outgoing", neopixel_can_id_outgoing, DEFAULT_NEOPIXEL_CAN_ID_OUTGOING);
  // Run the node
  // Neopixel obj(nh, 1, can_interface, log_filter, 0x784, argc, argv);
  Neopixel obj(nh, loop_rate, can_interface, log_filter, neopixel_can_id_outgoing, argc, argv);
  obj.run();
  return EXIT_SUCCESS;
}
