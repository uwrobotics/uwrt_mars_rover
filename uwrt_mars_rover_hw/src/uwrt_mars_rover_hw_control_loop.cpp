#include "uwrt_mars_rover_hw/uwrt_mars_rover_hw_control_loop.h"

namespace uwrt_mars_rover_hw {

MarsRoverHWControlLoop::MarsRoverHWControlLoop(std::string name, const ros::NodeHandle& nh)
    : name_(std::move(name)), nh_(nh) {}

bool MarsRoverHWControlLoop::init() {
  ros::NodeHandle loop_nh(nh_, name_);
  // TODO: utils package with rosparam errors on not finding param_name
  bool param_retrieved = loop_nh.param<double>("control_frequency", control_freq_, 50.0);
  ROS_ERROR_STREAM_COND_NAMED(!param_retrieved, name_,
                              loop_nh.getNamespace()
                                  << "/control_frequency could not be found and loaded from parameter server.");
  param_retrieved = loop_nh.param<double>("controllers_watchdog_timeout", controller_watchdog_timeout_, 5.0);
  ROS_ERROR_STREAM_COND_NAMED(
      !param_retrieved, name_,
      loop_nh.getNamespace() << "/controllers_watchdog_timeout could not be found and loaded from parameter server.");

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

void MarsRoverHWControlLoop::update(const ros::Time& time_now) {
  ros::Duration control_loop_period = time_now - last_control_loop_time_;
  last_control_loop_time_ = time_now;

  rover_hw_->read(time_now, control_loop_period);

  bool reset_controllers = (last_control_loop_time_.toSec() > controller_watchdog_timeout_);
  controller_manager_->update(time_now, control_loop_period, reset_controllers);

  rover_hw_->write(time_now, control_loop_period);
}

}  // namespace uwrt_mars_rover_hw
