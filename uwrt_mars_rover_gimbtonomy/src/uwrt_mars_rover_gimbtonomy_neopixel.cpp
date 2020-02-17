#include "uwrt_mars_rover_gimbtonomy/uwrt_mars_rover_gimbtonomy_neopixel.h"

void neopixel::set_state(const int& s){
    state = s;
}
neopixel::neopixel(int argc, char **argv) : loop_rate(1){
        ros::init(argc, argv, "neopixel_set_server");
        neopixel_service = neopixel_node.advertiseService("neopixel_set", set_state);
        ROS_INFO("Ready to update neopixel state.");
}
void neopixel::run(){
    while(ros::ok()){
        neopixel_can_msg.sendCAN(state);
        ros::spinOnce();
        loop_rate.sleep();
    }  
}