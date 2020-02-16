#define NEOPIXEL_CAN_ID 0x794
#define FRAME_PAYLOAD_LENGTH 4

class neopixel{
private:
    // State of neopixels {0:solid_red, 1:solid_blue, 2:flashing_green}
    int state;
    // Create handle for process
    ros::NodeHandle neopixel_node;
    // Create service
    ros::ServiceServer neopixel_service;
    // Loop rate controls speed of while loop
    ros::Rate loop_rate;
    // Callback function for ROS service
    void set_state(const int& s);
    // Create a CAN object to send the can message
    neopixel_can neopixel_can_msg(NEOPIXEL_CAN_ID, FRAME_PAYLOAD_LENGTH, "vcan0");
public:
    // neopixel constructor
    neopixel(int argc, char **argv);
    // Sets the state var
    void set_state(const int& s);
    // Runs the while loop
    void run();
};