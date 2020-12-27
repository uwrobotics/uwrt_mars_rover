#include "uwrt_mars_rover_control/uwrt_mars_rover_hw_control_loop_real.h"
namespace uwrt_mars_rover_control {

MarsRoverHWControlLoopReal::MarsRoverHWControlLoopReal(const ros::NodeHandle& nh)
    : MarsRoverHWControlLoop("rover_control_loop_real", nh) {
  if (!this->init()) {
    ROS_ERROR_NAMED(name_, "Failed to initialize RobotHW");
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
}  // namespace uwrt_mars_rover_control
