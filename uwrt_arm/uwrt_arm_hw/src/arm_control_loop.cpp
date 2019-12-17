/**
Copyright (c) 2020 Somesh Daga <s2daga@uwaterloo.ca>

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#include "uwrt_arm_hw/arm_control_loop.h"

#include <controller_manager/controller_manager.h>
#include <ros/ros.h>

#include <string>

namespace uwrt
{
namespace arm
{

ArmControlLoop::ArmControlLoop(std::string name,
                               const ros::NodeHandle& nh)
  : name_(std::move(name)),
    nh_(nh)
{}

bool ArmControlLoop::init()
{
  ros::NodeHandle loop_nh(nh_, name_);
  loop_nh.param("controller_frequency", control_freq_, 50.0);
  loop_nh.param("controller_watchdog_timeout", controller_watchdog_timeout_, 0.5);

  // Ensure the arm_hw_ exists
  if (!arm_hw_)
  {
    ROS_FATAL_NAMED("arm_control_loop",
                    "[ArmControlLoop] No ArmHW initialized!");
    return false;
  }

  // Initialize the ArmHW
  ros::NodeHandle arm_nh(nh_, arm_hw_->getName());
  if (!arm_hw_->init(nh_, arm_nh))
  {
    ROS_FATAL_NAMED(name_,
                    "[ArmControlLoop] Failed to initialize the ArmHW!");
    return false;
  }

  // Initialize the controller manager
  controller_manager_ =
    std::make_unique<controller_manager::ControllerManager>(arm_hw_.get(), nh_);

  return true;
}

void ArmControlLoop::update(const ros::Time& time_now,
                            bool update_controllers)
{
  ros::Duration rw_period = time_now - last_rw_time_;
  last_rw_time_ =           time_now;

  // Read from hardware
  arm_hw_->read(time_now, rw_period);

  if (update_controllers)
  {
    ros::Duration update_period = time_now - last_update_time_;
    last_update_time_           = time_now;

    bool timeout = (update_period.toSec() > controller_watchdog_timeout_);

    // Update the controllers
    controller_manager_->update(time_now, update_period, timeout);
  }

  // Ensure limits specified in URDF are obeyed
  arm_hw_->enforceLimits(rw_period);

  // Write to hardware
  arm_hw_->write(time_now, rw_period);
}
}  // namespace arm
}  // namespace uwrt
