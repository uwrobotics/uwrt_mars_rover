/**
Copyright (c) 2020 Somesh Daga <s2daga@uwaterloo.ca>

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/
#pragma once

#include "uwrt_arm_hw/arm_hw.h"

#include <controller_manager/controller_manager.h>
#include <ros/ros.h>

#include <string>

namespace uwrt
{
namespace arm
{
class ArmControlLoop
{
public:
  ArmControlLoop(std::string name,
                 const ros::NodeHandle& nh = ros::NodeHandle());
  virtual ~ArmControlLoop() = default;

  virtual bool init();

  void update(const ros::Time& time_now, bool update_controllers = true);

protected:
  // Short name of this class
  const std::string name_;

  // Handle at root of robot's namespace
  ros::NodeHandle nh_;

  // ArmHW instance
  std::unique_ptr<ArmHW> arm_hw_;

  // Controller Manager
  std::unique_ptr<controller_manager::ControllerManager> controller_manager_;

  // Timing
  ros::Time last_update_time_;
  ros::Time last_rw_time_;
  double    controller_watchdog_timeout_;
  double    control_freq_;
};  // class ArmControlLoop
}  // namespace arm
}  // namespace uwrt
