#include <cstdlib>
#include "uwrt_mars_rover_pid_tuning_api.h"

int main(int argc, char **argv) {
  ros::init(argc, argv, "pid_tuning_api_server");
  ros::NodeHandle nh;
  constexpr uint8_t loop_rate = 1;
  pidApi interface(nh, loop_rate);
  interface.run();
  return EXIT_SUCCESS;
}