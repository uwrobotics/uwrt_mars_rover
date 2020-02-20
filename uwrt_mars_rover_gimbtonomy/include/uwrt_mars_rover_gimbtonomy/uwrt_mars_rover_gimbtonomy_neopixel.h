#pragma once
#include "uwrt_mars_rover_msgs/set_state.h"
#include "uwrt_mars_rover_msgs/NeopixelArrayMode.h"
#include "uwrt_mars_rover_gimbtonomy/uwrt_mars_rover_gimbtonomy_can.h"

constexpr uint16_t NEOPIXEL_CAN_ID = 0x794;
constexpr uint8_t FRAME_PAYLOAD_LENGTH = 4;
const char* VCAN_NUM = "vcan0";

class neopixel{
private:
    // State of neopixels {0:solid_red, 1:solid_blue, 2:flashing_green}
    int state_var;
    // Create handle for process
    ros::NodeHandle neopixel_node;
    // Create service
    ros::ServiceServer neopixel_service;
    // Loop rate controls speed of while loop
    ros::Rate loop_rate;
    // CLI arg count
    int arg_count;
    // CLI arg list
    char **arg_list;
    // Create a CAN object to send the can message
    neopixel_can neopixel_can_msg(NEOPIXEL_CAN_ID, FRAME_PAYLOAD_LENGTH, VCAN_NUM);
public:
    // neopixel constructor
    neopixel(int argc, char **argv);
    // Sets the state var
    bool set_state(uwrt_mars_rover_msgs::set_state::Request &req,
                   uwrt_mars_rover_msgs::set_state::Response &res);
    // Initializes the neopixels
    void initialize_neopixels();
    // Runs the while loop
    void run();
};