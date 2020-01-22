#include "uwrt_mars_rover_hw/uwrt_mars_rover_hw_node.h"

#include <cstdlib>

#include "uwrt_mars_rover_hw/uwrt_mars_rover_hw_control_loop_real.h"

namespace uwrt_mars_rover_hw {}  // namespace uwrt_mars_rover_hw

int main(int argc, char **argv) {
  ros::init(argc, argv, "uwrt_mars_rover_base");
  ros::NodeHandle nh;

  ros::CallbackQueue cb_queue;
  nh.setCallbackQueue(&cb_queue);

  ros::AsyncSpinner spinner(0, &cb_queue);
  spinner.start();

  // TODO: realtime page lock

  try {
    uwrt_mars_rover_hw::MarsRoverHWControlLoopReal rover_hw_control_loop_real(nh);
    rover_hw_control_loop_real.runForeverBlocking();
  } catch (const std::runtime_error &e) {
    ROS_FATAL(e.what());
  }

  return EXIT_SUCCESS;
}
