#include "uwrt_mars_rover_neopixel/uwrt_mars_rover_neopixel_control.h"

constexpr uint16_t NEOPIXEL_CAN_ID = 0x794;
constexpr uint8_t FRAME_PAYLOAD_LENGTH = 1;
const char* VCAN = "vcan0";

bool neopixel::setState(uwrt_mars_rover_msgs::set_state::Request &req,
                         uwrt_mars_rover_msgs::set_state::Response &res){

    // Add if statemetn                     
    res.success = true;
    _state = req.requested_mode.value;
    ROS_INFO_STREAM("STATE in CB: " << static_cast<unsigned>(_state));
    return res.success;
}
neopixel::neopixel(int argc, char **argv) : _arg_count(argc), _arg_list(argv),
                                            _neopixel_can_msg(NEOPIXEL_CAN_ID, FRAME_PAYLOAD_LENGTH, VCAN){
        ros::init(argc, argv, "neopixel_set_server");   
}
/*void neopixel::initializeNeopixels(){
    neopixel obj(_arg_count, _arg_list);
    _neopixel_service = _neopixel_node.advertiseService("neopixel_set", &neopixel::setState, &obj);
    ROS_INFO("Ready to update neopixel state.");
}*/
void neopixel::run(){
    // Create handle for process
    ros::NodeHandle _neopixel_node;
    // Loop rate controls speed of while loop
    ros::Rate _loop_rate = 1;
    // object used for advertising service
    neopixel obj(_arg_count, _arg_list);
    _neopixel_service = _neopixel_node.advertiseService("neopixel_set", &neopixel::setState, &obj);

    ROS_INFO("Ready to update neopixel state.");
    while(ros::ok()){
        ROS_INFO_STREAM("STATE b4 sendcan: " << static_cast<unsigned>(_state));
        _neopixel_can_msg.sendCAN(_state);
        ros::spinOnce();
        _loop_rate.sleep();
    }  
}