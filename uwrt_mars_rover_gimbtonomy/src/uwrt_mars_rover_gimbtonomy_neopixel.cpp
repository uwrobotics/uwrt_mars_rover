#include "uwrt_mars_rover_gimbtonomy/uwrt_mars_rover_gimbtonomy_neopixel.h"

bool set_state(uwrt_mars_rover_msgs::set_state::Request &req,
               uwrt_mars_rover_msgs::set_state::Response &res){
    state = req.requested_mode;
    return true;
}
neopixel::neopixel(int argc, char **argv) : loop_rate(1), arg_count(argc), arg_list(argv){
        ros::init(argc, argv, "neopixel_set_server");   
}
void neopixel::initialize_neopixels(){
    neopixel obj(arg_count, arg_list);
    neopixel_service = neopixel_node.advertiseService("neopixel_set", &neopixel::set_state, &obj);
    ROS_INFO("Ready to update neopixel state.");
}
void neopixel::run(){
    initialize_neopixels();
    while(ros::ok()){
        neopixel_can_msg.sendCAN(state);
        ros::spinOnce();
        loop_rate.sleep();
    }  
}