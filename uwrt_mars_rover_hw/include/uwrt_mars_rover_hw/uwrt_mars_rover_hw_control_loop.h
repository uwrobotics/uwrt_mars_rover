#pragma once

#include <combined_robot_hw/combined_robot_hw.h>
#include <controller_manager/controller_manager.h>
#include <ros/ros.h>

#include <memory>
#include <string>

#include "uwrt_mars_rover_hw_drivetrain.h"

namespace uwrt_mars_rover_hw {

class MarsRoverHWControlLoop {
 public:
  MarsRoverHWControlLoop(std::string name, const ros::NodeHandle& nh = ros::NodeHandle());
  MarsRoverHWControlLoop(MarsRoverHWControlLoop&&) = delete;
  MarsRoverHWControlLoop(const MarsRoverHWControlLoop&) = delete;
  MarsRoverHWControlLoop& operator=(const MarsRoverHWControlLoop&&) = delete;
  MarsRoverHWControlLoop& operator=(const MarsRoverHWControlLoop&) = delete;
  virtual ~MarsRoverHWControlLoop() = default;

  virtual bool init();

  void update(const ros::Time& time_now);

 protected:
  const std::string name_;

  ros::NodeHandle nh_;

  std::unique_ptr<combined_robot_hw::CombinedRobotHW> rover_hw_;

  std::unique_ptr<controller_manager::ControllerManager> controller_manager_;

  ros::Time last_control_loop_time_;
  double controller_watchdog_timeout_{0};
  double control_freq_{0};
  static constexpr double DEFAULT_CONTROL_FREQUENCY{50.0};
  static constexpr double DEFAULT_CONTROLLER_WATCHDOG_TIMEOUT{5.0};
};
}  // namespace uwrt_mars_rover_hw
