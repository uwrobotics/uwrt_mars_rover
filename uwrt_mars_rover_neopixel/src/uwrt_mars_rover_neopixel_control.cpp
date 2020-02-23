#include "uwrt_mars_rover_neopixel/uwrt_mars_rover_neopixel_control.h"

constexpr uint16_t NEOPIXEL_CAN_ID = 0x794;
constexpr uint8_t FRAME_PAYLOAD_LENGTH = 4;
const char* VCAN = "vcan0";

bool neopixel::setState(uwrt_mars_rover_msgs::set_state::Request &req,
                         uwrt_mars_rover_msgs::set_state::Response &res){
    _state = req.requested_mode.value;
    return true;
}
neopixel::neopixel(int argc, char **argv) : _loop_rate(1), _arg_count(argc), _arg_list(argv),
                                            _neopixel_can_msg(NEOPIXEL_CAN_ID, FRAME_PAYLOAD_LENGTH, VCAN){
        ros::init(argc, argv, "neopixel_set_server");   
}
void neopixel::initializeNeopixels(){
    neopixel obj(_arg_count, _arg_list);
    _neopixel_service = _neopixel_node.advertiseService("neopixel_set", &neopixel::setState, &obj);
    ROS_INFO("Ready to update neopixel state.");
}
void neopixel::run(){
    initializeNeopixels();
    while(ros::ok()){
        _neopixel_can_msg.sendCAN(_state);
        ros::spinOnce();
        _loop_rate.sleep();
    }  
}