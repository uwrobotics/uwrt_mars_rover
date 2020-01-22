#pragma once

#include <controller_manager/controller_manager.h>
#include <ros/ros.h>

#include <memory>
#include <string>

#include "uwrt_mars_rover_hw_drivetrain.h"

namespace uwrt_mars_rover_hw {

class MarsRoverHWControlLoop {
 public:
  MarsRoverHWControlLoop(std::string name, const ros::NodeHandle& nh = ros::NodeHandle());

  virtual ~MarsRoverHWControlLoop() = default;

  virtual bool init();

  void update(const ros::Time& time_now, bool update_controllers = true);

 protected:
  const std::string name_;

  ros::NodeHandle nh_;

  std::unique_ptr<UWRTRoverHWDrivetrain> rover_hw_;

  std::unique_ptr<controller_manager::ControllerManager> controller_manager_;

  ros::Time last_update_time_;
  ros::Time last_rw_time_;
  double controller_watchdog_timeout_;
  double control_freq_;
};

}  // namespace uwrt_mars_rover_hw
