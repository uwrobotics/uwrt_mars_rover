/**
Copyright (c) 2020 Somesh Daga <s2daga@uwaterloo.ca>

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/
#include "uwrt_arm_hw/arm_hw_real.h"
#include "uwrt_arm_hw/real_arm_control.h"

#include <ros/ros.h>

#include <string>

namespace uwrt
{
namespace arm
{
RealArmControl::RealArmControl()
  : ArmControlLoop("real_arm_control")
{
  // Get rate of spinning
  nh_.param<int>("loop_hz", loop_hz_, 50.0);

  // Get the robot description from the parameter server
  std::string robot_description;
  if (!nh_.getParam("robot_description", robot_description))
  {
    ROS_FATAL_NAMED(name_,
                    "[RealArmControl] ROS Parameter 'robot_description' not found!");
    return;
  }

  // Setup the ArmHW
  arm_hw_ = std::make_unique<ArmHWReal>(robot_description);

  bool initialized = init();
  if (!initialized)
  {
    ROS_FATAL_NAMED(name_,
                    "[RealArmControl] Failed to initialize the ArmControlLoop!");
    return;
  }

  control_period_ = ros::Duration(1.0 / control_freq_);

  ROS_INFO_NAMED(name_,
                 "[ArmRealControl] Successfully initialized!");
}

RealArmControl::~RealArmControl()
{}

void RealArmControl::spin()
{
  ros::Rate loop_rate(loop_hz_);
  while (ros::ok())
  {
    spinOnce();
    loop_rate.sleep();
  }
}

void RealArmControl::spinOnce()
{
  // TODO(someshdaga): Consider using wallclock/steadyclock
  const ros::Time time_now = ros::Time::now();
  const bool& update_controllers = (time_now - last_update_time_) > control_period_;
  update(time_now, update_controllers);
}

}  // namespace arm
}  // namespace uwrt
