#include <uwrt_mars_rover_control/uwrt_mars_rover_hw_control_loop.h>

namespace uwrt_mars_rover_control {


bool MarsRoverHWControlLoop::init() {
  ros::NodeHandle rover_hw_nh(nh_, "combined_robot_hw");
  rover_hw_ = std::make_unique<combined_robot_hw::CombinedRobotHW>();
  if (!rover_hw_->init(nh_, rover_hw_nh)) {
    ROS_FATAL_STREAM_NAMED(name_, "Failed to initialize combined_robot_hw");
    return false;
  }

  controller_manager_ = std::make_unique<controller_manager::ControllerManager>(rover_hw_.get(), nh_);

  // Set last_control_loop_time_ for first update call
  ros::ros_steadytime(last_control_loop_time_.sec, last_control_loop_time_.nsec);

  return true;
}

void MarsRoverHWControlLoop::update() {
  // Update current time using UNIX time
  ros::Time time_now = ros::Time::now();

  // Calculate control loop period using Monotonic Time
  ros::ros_steadytime(current_control_loop_time_.sec, current_control_loop_time_.nsec);
  ros::Duration control_loop_period = current_control_loop_time_ - last_control_loop_time_;
  last_control_loop_time_ = current_control_loop_time_;

  //  rover_hw_->read(time_now, control_loop_period);

  bool reset_controllers = (control_loop_period.toSec() > controller_watchdog_timeout_);
  ROS_WARN_STREAM_COND_NAMED(reset_controllers, name_,
                             "Control loop period exceeded watchdog timeout by "
                                 << (control_loop_period.toSec() - controller_watchdog_timeout_)
                                 << " seconds. Resetting controllers.");
  controller_manager_->update(time_now, control_loop_period, reset_controllers);

  //  rover_hw_->write(time_now, control_loop_period);
}

}  // namespace uwrt_mars_rover_control
