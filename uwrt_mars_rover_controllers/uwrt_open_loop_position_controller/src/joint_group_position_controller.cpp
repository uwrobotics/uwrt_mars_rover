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

#include <position_controllers/joint_group_position_controller.h>
#include <pluginlib/class_list_macros.hpp>

namespace position_controllers
{
bool UWRTOpenLoopPositionController::init(hardware_interface::PositionJointInterface* hw, ros::NodeHandle &nh)
{
  nh.getParam("cmd_pos_timeout_", cmd_pos_timeout_);
  nh.getParam("wheel_separation", wheel_separation_);
  nh.getParam("wheel_radius", wheel_radius_);
  
  // Get left / right wheels from the parameter server
  std::vector<std::string> left_wheel_names, right_wheel_names;
  if (!getWheelNames(nh, "left_wheel", left_wheel_names) or
      !getWheelNames(nh, "right_wheel", right_wheel_names))
  {
    return false;
  }

  if (left_wheel_names.size() != right_wheel_names.size())
  {
    ROS_ERROR_STREAM(
        "#left wheels (" << left_wheel_names.size() << ") != " <<
        "#right wheels (" << right_wheel_names.size() << ").");
    return false;
  }
  else
  {
    wheel_joints_size_ = left_wheel_names.size();

    left_wheel_joints_.resize(wheel_joints_size_);
    right_wheel_joints_.resize(wheel_joints_size_);
  }

  sub_command_ = nh.subscribe("cmd_pos", 1, &UWRTOpenLoopPositionController::cmdPosCallback, this);

  return true;
}

bool UWRTOpenLoopPositionController::getWheelNames(ros::NodeHandle& controller_nh,
                              const std::string& wheel_param,
                              std::vector<std::string>& wheel_names)
{
  XmlRpc::XmlRpcValue wheel_list;
  if (!controller_nh.getParam(wheel_param, wheel_list))
  {
    ROS_ERROR_STREAM(
        "Couldn't retrieve wheel param '" << wheel_param << "'.");
    return false;
  }

  if (wheel_list.getType() == XmlRpc::XmlRpcValue::TypeArray)
  {
    if (wheel_list.size() == 0)
    {
      ROS_ERROR_STREAM(
          "Wheel param '" << wheel_param << "' is an empty list");
      return false;
    }

    for (int i = 0; i < wheel_list.size(); ++i)
    {
      if (wheel_list[i].getType() != XmlRpc::XmlRpcValue::TypeString)
      {
        ROS_ERROR_STREAM(
            "Wheel param '" << wheel_param << "' #" << i <<
            " isn't a string.");
        return false;
      }
    }

    wheel_names.resize(wheel_list.size());
    for (int i = 0; i < wheel_list.size(); ++i)
    {
      wheel_names[i] = static_cast<std::string>(wheel_list[i]);
    }
  }
  else if (wheel_list.getType() == XmlRpc::XmlRpcValue::TypeString)
  {
    wheel_names.push_back(wheel_list);
  }
  else
  {
    ROS_ERROR_STREAM(
        "Wheel param '" << wheel_param <<
        "' is neither a list of strings nor a string.");
    return false;
  }

  return true;
}

void UWRTOpenLoopPositionController::starting(const ros::Time& time)
{
  // Start controller with current joint positions
  for (unsigned int i = 0; i < wheel_joints_size_; ++i)
  {
    left_wheel_joints_[i].getPosition();
    right_wheel_joints_[i].getPosition();
  }
}

void UWRTOpenLoopPositionController::cmdPosCallback(const uwrt_mars_rover_msgs::UWRTOpenLoopPosCmd& command)
{
  command_struct_.ang   = command.angular.z;
  command_struct_.lin   = command.linear.x;
  command_struct_.stamp = ros::Time::now();
  command_.writeFromNonRT (command_struct_);
}

void UWRTOpenLoopPositionController::update(const ros::Time&, const ros::Duration&)
{
  // Retreive current velocity command and time step:
  Commands curr_cmd = *(command_.readFromRT());

  // const double dt = (time - curr_cmd.stamp).toSec(); // TODO (akeaveny): overload operator for -
  // // reset to zero position has timeout:
  // if (dt > cmd_pos_timeout_)
  // {
  //   curr_cmd.lin = 0.0;
  //   curr_cmd.ang = 0.0;
  // }

  // -------------------------------------------------------------
  // GET RIGHT / LEFT POSITION FROM LINEAR / ANGULAR COMMANDS 
  // -------------------------------------------------------------
  // Compute wheels posistions:
  const double pos_left  = (curr_cmd.lin - curr_cmd.ang * wheel_separation_ / 2.0)/wheel_radius_;
  const double pos_right = (curr_cmd.lin + curr_cmd.ang * wheel_separation_ / 2.0)/wheel_radius_;

  // Set wheels posistions:
  for (unsigned int i = 0; i < wheel_joints_size_; ++i)
  {
    left_wheel_joints_[i].setCommand(pos_left);
    right_wheel_joints_[i].setCommand(pos_right);
  }
}
} // position_controllers
PLUGINLIB_EXPORT_CLASS(position_controllers::UWRTOpenLoopPositionController,controller_interface::ControllerBase)
