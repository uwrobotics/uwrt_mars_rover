#include <uwrt_mars_rover_control/uwrt_mars_rover_hw_control_loop_real.h>
#include <uwrt_mars_rover_utils/uwrt_params.h>

namespace uwrt_mars_rover_control {
// static constexpr class members must have definitions outside of their class to compile. This can be removed in C++17
constexpr double MarsRoverHWControlLoop::DEFAULT_CONTROL_FREQUENCY;
constexpr double MarsRoverHWControlLoop::DEFAULT_CONTROLLER_WATCHDOG_TIMEOUT;

MarsRoverHWControlLoopReal::MarsRoverHWControlLoopReal(const ros::NodeHandle& nh)
    : MarsRoverHWControlLoop("rover_control_loop_real", nh) {
  control_freq_ = uwrt_mars_rover_utils::getParam(private_nh_, name_, "control_frequency", DEFAULT_CONTROL_FREQUENCY);
  controller_watchdog_timeout_ = uwrt_mars_rover_utils::getParam(private_nh_, name_, "controllers_watchdog_timeout",
                                                                 DEFAULT_CONTROLLER_WATCHDOG_TIMEOUT);
  if (!this->init()) {
    ROS_ERROR_NAMED(name_, "Failed to initialize RobotHW");
    throw std::runtime_error("Failed to initialize RobotHW");
  }
}

void MarsRoverHWControlLoopReal::runForeverBlocking() {
  ros::Rate rate(control_freq_);

  while (ros::ok()) {
    this->update();
    rate.sleep();
  }
}
}  // namespace uwrt_mars_rover_control
