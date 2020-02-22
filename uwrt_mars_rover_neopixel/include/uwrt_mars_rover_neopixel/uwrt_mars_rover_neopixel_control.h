#pragma once
#include "uwrt_mars_rover_msgs/set_state.h"
#include "uwrt_mars_rover_msgs/NeopixelArrayMode.h"
#include "uwrt_mars_rover_neopixel/uwrt_mars_rover_neopixel_can.h"

class neopixel{
private:
    // State of neopixels {0:solid_red, 1:solid_blue, 2:flashing_green}
    uint8_t state;
    // Create handle for process
    ros::NodeHandle neopixel_node;
    // Create service
    ros::ServiceServer neopixel_service;
    // Loop rate controls speed of while loop
    ros::Rate loop_rate;
    // CLI arg count needed for intiliaze_neopixels
    uint8_t arg_count;
    // CLI arg list needed for intiliaze_neopixels
    char **arg_list;
    // Create a CAN object to send the can message
    neopixel_can neopixel_can_msg;
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