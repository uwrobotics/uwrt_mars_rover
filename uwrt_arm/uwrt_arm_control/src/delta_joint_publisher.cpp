/**
Copyright (c) 2020 Somesh Daga <s2daga@uwaterloo.ca>

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/
#include "uwrt_arm_control/delta_joint_publisher.h"
#include "uwrt_arm_msgs/UWRTArmCommand.h"

#include <control_msgs/JointJog.h>
#include <ros/ros.h>

namespace uwrt
{
namespace arm
{
DeltaJointPublisher::DeltaJointPublisher(ros::NodeHandle& nh,
                                         ros::NodeHandle& priv_nh)
  : nh_(nh),
    priv_nh_(priv_nh),
    control_mode_(ControlMode::VELOCITY),
    jog_msg_()
{
  // Not sure if there's a better way than hard coding this right now
  jog_msg_.joint_names =
  {
    "arm_base_turntable_joint",
    "arm_shoulder_joint",
    "arm_elbow_joint",
    "arm_wrist_roll_joint",
    "arm_wrist_pitch_joint"
  };
  jog_command_pub_ = nh_.advertise<control_msgs::JointJog>("joint_delta_jog_cmds",
                                                           5,
                                                           false);
  arm_command_sub_ = nh_.subscribe<uwrt_arm_msgs::UWRTArmCommand>("delta_arm_cmds",
                                                                  1,
                                                                  &DeltaJointPublisher::armCommandCallback,
                                                                  this);
}

DeltaJointPublisher::~DeltaJointPublisher()
{}

void DeltaJointPublisher::spin()
{
  while (ros::ok())
    ros::spinOnce();
}

void DeltaJointPublisher::armCommandCallback(const uwrt_arm_msgs::UWRTArmCommandConstPtr& arm_command)
{
  armCommandToJointJog(*arm_command, jog_msg_);
  jog_command_pub_.publish(jog_msg_);
}

void DeltaJointPublisher::armCommandToJointJog(const uwrt_arm_msgs::UWRTArmCommand& arm_command_msg,
                                               control_msgs::JointJog& jog_msg)
{
  jog_msg.header = arm_command_msg.header;
  switch (control_mode_)
  {
    case ControlMode::POSITION:
      jog_msg.displacements =
      {
        arm_command_msg.turntable_command,
        arm_command_msg.shoulder_command,
        arm_command_msg.elbow_command,
        arm_command_msg.wrist_roll_command,
        arm_command_msg.wrist_pitch_command
      };
      break;
    case ControlMode::VELOCITY:
      jog_msg.velocities =
      {
        arm_command_msg.turntable_command,
        arm_command_msg.shoulder_command,
        arm_command_msg.elbow_command,
        arm_command_msg.wrist_roll_command,
        arm_command_msg.wrist_pitch_command
      };
      break;
  }
}
}  // namespace arm
}  // namespace uwrt
