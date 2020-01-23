#pragma once

#include <combined_robot_hw/combined_robot_hw.h>
#include <controller_manager/controller_manager.h>
#include <ros/ros.h>
#include <uwrt_mars_rover_drivetrain_hw/uwrt_mars_rover_drivetrain_hw.h>
#include <uwrt_mars_rover_utils/uwrt_params.h>

#include <memory>
#include <string>

<<<<<<< HEAD:uwrt_mars_rover_control/include/uwrt_mars_rover_control/uwrt_mars_rover_hw_control_loop.h
namespace uwrt_mars_rover_control {
=======
namespace uwrt_mars_rover_hw {
>>>>>>> 4e2498a... initial science_hw base class implementation:uwrt_mars_rover_hw/include/uwrt_mars_rover_hw/uwrt_mars_rover_hw_control_loop.h

class MarsRoverHWControlLoop {
 public:
  MarsRoverHWControlLoop(std::string name, const ros::NodeHandle& nh = ros::NodeHandle())
      : name_(std::move(name)), nh_(nh), private_nh_(ros::NodeHandle(nh, name_)) {}

  MarsRoverHWControlLoop(MarsRoverHWControlLoop&&) = delete;
  MarsRoverHWControlLoop(const MarsRoverHWControlLoop&) = delete;
  MarsRoverHWControlLoop& operator=(const MarsRoverHWControlLoop&&) = delete;
  MarsRoverHWControlLoop& operator=(const MarsRoverHWControlLoop&) = delete;
  virtual ~MarsRoverHWControlLoop() = default;

  virtual bool init();

  void update();

 protected:
  static constexpr double DEFAULT_CONTROL_FREQUENCY{50.0};
  static constexpr double DEFAULT_CONTROLLER_WATCHDOG_TIMEOUT{5.0};

  const std::string name_;

  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;

  std::unique_ptr<combined_robot_hw::CombinedRobotHW> rover_hw_;
  std::unique_ptr<controller_manager::ControllerManager> controller_manager_;

  ros::Time current_control_loop_time_;
  ros::Time last_control_loop_time_;

  double control_freq_{0.0};
  double controller_watchdog_timeout_{0.0};
};
}  // namespace uwrt_mars_rover_control
