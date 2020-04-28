/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, Willow Garage, Inc.
 *  Copyright (c) 2012, hiDOF, Inc.
 *  Copyright (c) 2013, PAL Robotics, S.L.
 *  Copyright (c) 2014, Fraunhofer IPA
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Willow Garage nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

#pragma once

#include <vector>
#include <string>

#include <ros/node_handle.h>
#include <hardware_interface/joint_command_interface.h>
#include <controller_interface/controller.h>
#include <geometry_msgs/TwistStamped.h>
#include <realtime_tools/realtime_buffer.h>

#include "uwrt_mars_rover_msgs/UWRTOpenLoopPosCmd.h"

namespace position_controllers
{
/**
 * \brief Forward command controller for a set of joints.
 *
 * This class forwards the command signal down to a set of joints.
 * Command signal and joint hardware interface are of the same type, e.g. effort commands for an effort-controlled
 * joint.
 *
 * \section ROS interface
 *
 * \param type hardware interface type.
 * \param joints Names of the joints to control.
 *
 * Subscribes to:
 * - \b command (std_msgs::Float64MultiArray) : The joint commands to apply.
 */

// typedef forward_command_controller::UWRTOpenLoopPositionController<hardware_interface::PositionJointInterface>
        // UWRTOpenLoopPositionController;

class UWRTOpenLoopPositionController: public controller_interface::Controller<hardware_interface::PositionJointInterface>
{
public:
  UWRTOpenLoopPositionController() {}
  ~UWRTOpenLoopPositionController() {sub_command_.shutdown();}
  
  bool init(hardware_interface::PositionJointInterface* hw, ros::NodeHandle &nh);

  void starting(const ros::Time& time);
  void stopping(const ros::Time& time);

  void update(const ros::Time&, const ros::Duration&);

  bool getWheelNames(ros::NodeHandle& controller_nh,
                       const std::string& wheel_param,
                       std::vector<std::string>& wheel_names);

  std::vector<hardware_interface::JointHandle> left_wheel_joints_;
  std::vector<hardware_interface::JointHandle> right_wheel_joints_;
  unsigned int wheel_joints_size_;

private:
  
  /// position command related:
  struct Commands
  {
  double lin;
  double ang;
  ros::Time stamp;
  Commands() : lin(0.0), ang(0.0), stamp(0.0) {}
  };

  realtime_tools::RealtimeBuffer<Commands> command_;
  Commands command_struct_;

  ros::Subscriber sub_command_;
  void cmdPosCallback(const uwrt_mars_rover_msgs::UWRTOpenLoopPosCmd& command);

  double cmd_pos_timeout_;
  double wheel_separation_;
  double wheel_radius_;
};
} // position_controllers
