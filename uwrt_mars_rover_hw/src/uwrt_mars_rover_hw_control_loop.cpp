#include "uwrt_mars_rover_hw/uwrt_mars_rover_hw_control_loop.h"

namespace uwrt_mars_rover_hw {
// static constexpr class members must have definitions outside of their class to compile. This can be removed in C++17
constexpr double MarsRoverHWControlLoop::DEFAULT_CONTROL_FREQUENCY;
constexpr double MarsRoverHWControlLoop::DEFAULT_CONTROLLER_WATCHDOG_TIMEOUT;

MarsRoverHWControlLoop::MarsRoverHWControlLoop(std::string name, const ros::NodeHandle& nh)
    : name_(std::move(name)), nh_(nh) {}

bool MarsRoverHWControlLoop::init() {
  ros::NodeHandle loop_nh(nh_, name_);
  // TODO: utils package with rosparam errors on not finding param_name
  bool param_retrieved = loop_nh.param<double>("control_frequency", control_freq_, DEFAULT_CONTROL_FREQUENCY);
  ROS_ERROR_STREAM_COND_NAMED(!param_retrieved, name_,
                              loop_nh.getNamespace()
                                  << "/control_frequency could not be found and loaded from parameter server.");
  param_retrieved = loop_nh.param<double>("controllers_watchdog_timeout", controller_watchdog_timeout_,
                                          DEFAULT_CONTROLLER_WATCHDOG_TIMEOUT);
  ROS_ERROR_STREAM_COND_NAMED(
      !param_retrieved, name_,
      loop_nh.getNamespace() << "/controllers_watchdog_timeout could not be found and loaded from parameter server.");

  ros::NodeHandle rover_hw_nh(nh_, "combined_robot_hw");
  rover_hw_ = std::make_unique<combined_robot_hw::CombinedRobotHW>();
  if (!rover_hw_->init(nh_, rover_hw_nh)) {
    ROS_FATAL_STREAM_NAMED(name_, "Failed to initialize combined_robot_hw");
    return false;
  }

  controller_manager_ = std::make_unique<controller_manager::ControllerManager>(rover_hw_.get(), nh_);
  return true;
}

void MarsRoverHWControlLoop::update(const ros::Time& time_now) {
  ros::Duration control_loop_period = time_now - last_control_loop_time_;
  last_control_loop_time_ = time_now;

  rover_hw_->read(time_now, control_loop_period);

  bool reset_controllers = (control_loop_period.toSec() > controller_watchdog_timeout_);
  ROS_WARN_STREAM_COND_NAMED(reset_controllers, name_,
                             "Control loop period exceeded watchdog timeout by "
                                 << (control_loop_period.toSec() - controller_watchdog_timeout_)
                                 << " seconds. Restarting controllers.");
  controller_manager_->update(time_now, control_loop_period, reset_controllers);

  rover_hw_->write(time_now, control_loop_period);
}

}  // namespace uwrt_mars_rover_hw
