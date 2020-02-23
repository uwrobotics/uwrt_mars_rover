#include "uwrt_mars_rover_hw/uwrt_mars_rover_hw_control_loop_real.h"
namespace uwrt_mars_rover_hw {

MarsRoverHWControlLoopReal::MarsRoverHWControlLoopReal(const ros::NodeHandle& nh)
    : MarsRoverHWControlLoop("robot_hw_control_loop_real", nh) {
  if (!this->init()) {
    throw std::runtime_error("Failed to initialize RobotHW");
  }

  // Get current time for initial update
  ros::ros_steadytime(last_control_loop_time_.sec, last_control_loop_time_.nsec);
}

void MarsRoverHWControlLoopReal::runForeverBlocking() {
  ros::Time current_time;
  ros::Rate rate(control_freq_);

  while (ros::ok()) {
    // Use Monotonic Time
    ros::ros_steadytime(current_time.sec, current_time.nsec);
    this->update(current_time);
    rate.sleep();
  }
}
}  // namespace uwrt_mars_rover_hw
