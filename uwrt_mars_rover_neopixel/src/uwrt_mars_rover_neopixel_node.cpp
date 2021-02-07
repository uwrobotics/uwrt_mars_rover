#include <uwrt_mars_rover_neopixel/uwrt_mars_rover_neopixel_control.h>
#include <uwrt_mars_rover_utils/uwrt_params.h>

constexpr uint8_t DEFAULT_LOOP_RATE{1};
constexpr uint16_t DEFAULT_NEOPIXEL_CAN_ID_OUTGOING{0x784};
const std::string DEFAULT_CAN_INTERFACE{"vcan0"};
const std::string DEFAULT_LOG_FILTER{"neopixel"};
const std::string NODE_NAME{"neopixel_control_server"};

int main(int argc, char** argv) 
{
  ros::init(argc, argv, NODE_NAME);
  // Create node handle
  ros::NodeHandle nh;

  // Use Service Parameters
  const int loop_rate
  {
      uwrt_mars_rover_utils::getParam<int>(nh, NODE_NAME, "neopixel/loop_rate", DEFAULT_LOOP_RATE)
  };
  const std::string can_interface
  {
      uwrt_mars_rover_utils::getParam(nh, NODE_NAME, "neopixel/can_interface", DEFAULT_CAN_INTERFACE)
  };
  const std::string log_filter
  {
      uwrt_mars_rover_utils::getParam(nh, NODE_NAME, "neopixel/log_filter", DEFAULT_LOG_FILTER)
  };
  const int neopixel_can_id_outgoing
  {
      uwrt_mars_rover_utils::getParam<int>(nh, NODE_NAME, "neopixel/can_id_outgoing", DEFAULT_NEOPIXEL_CAN_ID_OUTGOING)
  };

  // Run the node
  Neopixel neopixel_node(nh, static_cast<uint8_t>(loop_rate), can_interface, log_filter,
                         static_cast<uint16_t>(neopixel_can_id_outgoing));
  neopixel_node.run();
  return EXIT_SUCCESS;
}
