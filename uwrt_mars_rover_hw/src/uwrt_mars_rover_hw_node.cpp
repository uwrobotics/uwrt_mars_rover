#include <cstdlib>

#include "uwrt_mars_rover_hw/uwrt_mars_rover_hw_control_loop_real.h"

namespace uwrt_mars_rover_hw {}  // namespace uwrt_mars_rover_hw

int main(int argc, char **argv) {
  ros::init(argc, argv, "uwrt_mars_rover_base");
  ros::NodeHandle nh;

  // Startup enough spinners to automatically match number of CPU cores
  ros::AsyncSpinner async_spinner(0);
  async_spinner.start();

  // TODO: realtime page lock

  uwrt_mars_rover_hw::MarsRoverHWControlLoopReal rover_hw_control_loop_real(nh);
  rover_hw_control_loop_real.runForeverBlocking();

  async_spinner.stop();

  return EXIT_SUCCESS;
}
