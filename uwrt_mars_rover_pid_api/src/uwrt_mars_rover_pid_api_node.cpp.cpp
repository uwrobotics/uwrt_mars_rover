#include <cstdlib>
#include "uwrt_mars_rover_pid_api.h"
#define LOOP_RATE 1

int main(int argc, char **argv) {
    ros::init(argc, argv, "pid_api_server");
    ros::NodeHandle nh;
    pidApi pidApi_obj(nh, LOOP_RATE);
    pidApi_obj.run();
    return EXIT_SUCCESS;
}