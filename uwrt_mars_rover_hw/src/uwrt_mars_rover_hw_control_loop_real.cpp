#include "uwrt_mars_rover_hw/uwrt_mars_rover_hw_control_loop_real.h"
namespace uwrt_mars_rover_hw {

MarsRoverHWControlLoopReal::MarsRoverHWControlLoopReal(const ros::NodeHandle& nh)
    : MarsRoverHWControlLoop("robot_hw_control_loop_real", nh) {
  this->rover_hw_ = std::make_unique<UWRTRoverHWDrivetrainReal>();
  if (!this->init()) {
    throw std::runtime_error("Failed to initialize RobotHW");
  }

  // Get current time for initial update
  ros::ros_steadytime(last_update_time_.sec, last_update_time_.nsec);
}

void MarsRoverHWControlLoopReal::runForeverBlocking() {
  ros::Time time_now;
  ros::Rate rate(control_freq_);

  while (ros::ok()) {
    // Use Monotonic Time
    ros::ros_steadytime(time_now.sec, time_now.nsec);
    this->update(time_now);
    rate.sleep();
  }
}
}  // namespace uwrt_mars_rover_hw
