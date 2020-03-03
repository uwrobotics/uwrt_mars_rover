#include "uwrt_mars_rover_neopixel/uwrt_mars_rover_neopixel_control.h"

constexpr uint16_t NEOPIXEL_CAN_ID_OUTGOING = 0x784;
constexpr uint8_t  FRAME_PAYLOAD_LENGTH = 1;
constexpr uint8_t  LOOP_RATE = 1;

neopixel::neopixel(int argc, char **argv, char* can_interface) : _arg_count(argc), _arg_list(argv),
                   _can_interface(can_interface),
                   _neopixel_can_msg(NEOPIXEL_CAN_ID_OUTGOING, FRAME_PAYLOAD_LENGTH, can_interface){
        ros::init(argc, argv, "neopixel_set_server");   
}
bool neopixel::setState(uwrt_mars_rover_msgs::set_state::Request &req,
                         uwrt_mars_rover_msgs::set_state::Response &res){
    ROS_INFO("Neopixel callback function triggered.");
    _state = req.requested_mode.value;
    _neopixel_can_msg.sendCAN(_state);
    if(_neopixel_can_msg.waitforAck()){
        ROS_INFO("Received acknowledgement from gimbtonomy board.");
        res.success = true;
        return res.success;
    }
    // As currently implemented, this will never run. See waitforAck() for more details
    else{
        ROS_ERROR("Unexpected acknowledgement message received.");
        res.success = false;
        return false;
    }
}
void neopixel::run(){
    // Create handle for process
    ros::NodeHandle _neopixel_node;
    // Loop rate controls speed of while loop
    ros::Rate _loop_rate = LOOP_RATE;
    // object used for advertising service
    neopixel obj(_arg_count, _arg_list, _can_interface);
    _neopixel_service = _neopixel_node.advertiseService("neopixel_set", &neopixel::setState, &obj);
    ROS_INFO("Ready to update neopixel state.");
    while(ros::ok()){
        ros::spinOnce();
        _loop_rate.sleep();
    }  
}