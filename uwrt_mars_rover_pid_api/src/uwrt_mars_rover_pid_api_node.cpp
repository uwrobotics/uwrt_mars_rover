#include <cstdlib>
#include "uwrt_mars_rover_pid_api.h"
constexpr uint8_t loop_rate = 1;
int main(int argc, char **argv) {
    ros::init(argc, argv, "pid_api_server");
    ros::NodeHandle nh;
    pidApi pidApi_obj(nh, loop_rate);
    pidApi_obj.run();
    return EXIT_SUCCESS;
}