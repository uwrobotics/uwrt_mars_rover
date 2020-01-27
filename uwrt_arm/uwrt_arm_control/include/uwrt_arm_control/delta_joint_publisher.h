/**
Copyright (c) 2020 Somesh Daga <s2daga@uwaterloo.ca>

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/
#pragma once

#include "uwrt_arm_msgs/UWRTArmCommand.h"

#include <control_msgs/JointJog.h>
#include <ros/ros.h>

namespace uwrt
{
namespace arm
{
class DeltaJointPublisher
{
enum ControlMode
{
  POSITION,
  VELOCITY
};

public:
  DeltaJointPublisher(ros::NodeHandle& nh,
                      ros::NodeHandle& priv_nh);
  ~DeltaJointPublisher();

  void spin();

protected:
  ros::NodeHandle nh_;
  ros::NodeHandle priv_nh_;

  // Subscriber Callback(s)
  void armCommandCallback(const uwrt_arm_msgs::UWRTArmCommandConstPtr& arm_command);

  void armCommandToJointJog(const uwrt_arm_msgs::UWRTArmCommand& arm_command,
                            control_msgs::JointJog& jog_command);

private:
  ControlMode control_mode_;

  control_msgs::JointJog jog_msg_;

  ros::Subscriber arm_command_sub_;
  ros::Publisher jog_command_pub_;
};  // class DeltaJointPublisher
}  // namespace arm
}  // namespace uwrt
