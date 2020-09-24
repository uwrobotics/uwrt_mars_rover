#include <cstdlib>
#include "uwrt_mars_rover_pid_tuning_api.h"

int main(int argc, char **argv) {
  ros::init(argc, argv, "pid_tuning_api_server");
  ros::NodeHandle nh;
  constexpr uint8_t LOOP_RATE = 1;
  PidTuningApi interface(nh, LOOP_RATE);
  interface.run();
  return EXIT_SUCCESS;
}