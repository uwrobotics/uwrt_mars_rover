#include "uwrt_mars_rover_hw/uwrt_mars_rover_hw_control_loop.h"

namespace uwrt_mars_rover_hw {

MarsRoverHWControlLoop::MarsRoverHWControlLoop(std::string name, const ros::NodeHandle& nh)
    : name_(std::move(name)), nh_(nh) {}

bool MarsRoverHWControlLoop::init() {
  ros::NodeHandle loop_nh(nh_, name_);
  loop_nh.param("control_frequency", control_freq_, 50.0);
  loop_nh.param("controllers_watchdog_timeout", controller_watchdog_timeout_, 0.5);

  if (!rover_hw_) {
    ROS_WARN_NAMED(name_, "No Derived RoverHW initialized, falling back on default base class");
    rover_hw_ = std::make_unique<UWRTRoverHWDrivetrain>();
  }

  ros::NodeHandle rover_nh(nh_, rover_hw_->getName());
  if (!rover_hw_->init(nh_, rover_nh)) {
    ROS_FATAL_STREAM_NAMED(name_, "Failed to initialize " << rover_hw_->getName());
    return false;
  }

  controller_manager_ = std::make_unique<controller_manager::ControllerManager>(rover_hw_.get(), nh_);
  return true;
}

void MarsRoverHWControlLoop::update(const ros::Time& time_now, bool update_controllers) {
  ros::Duration rw_period = time_now - last_rw_time_;
  last_rw_time_ = time_now;

  rover_hw_->read(time_now, rw_period);

  if (update_controllers) {
    ros::Duration update_period = time_now - last_update_time_;
    last_update_time_ = time_now;

    bool reset_controllers = (update_period.toSec() > controller_watchdog_timeout_);
    controller_manager_->update(time_now, update_period, reset_controllers);
  }

  rover_hw_->write(time_now, rw_period);
}

}  // namespace uwrt_mars_rover_hw
