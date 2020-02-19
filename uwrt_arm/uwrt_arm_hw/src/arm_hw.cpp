/**
Copyright (c) 2019 Somesh Daga <s2daga@uwaterloo.ca>

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/
#include "uwrt_arm_hw/arm_hw.h"
#include "uwrt_arm_hw/voltage_joint_interface.h"

#include <joint_limits_interface/joint_limits_urdf.h>
#include <ros/ros.h>
#include <urdf_model/joint.h>

#include <limits>
#include <list>
#include <string>

namespace uwrt
{
namespace arm
{
ArmHW::ArmHW(const std::string& urdf_str)
  : ArmHW("arm_hw", urdf_str)
{}

ArmHW::ArmHW(const std::string& name,
             const std::string& urdf_str)
  : name_(std::move(name)),
    urdf_string_(std::move(urdf_str)),
    num_joints_(5)
{
  joint_types_.resize(num_joints_);
  joint_control_method_.resize(num_joints_);
  joint_position_.resize(num_joints_, 0.0);
  joint_position_command_.resize(num_joints_, 0.0);
  joint_velocity_.resize(num_joints_, 0.0);
  joint_velocity_command_.resize(num_joints_, 0.0);
  joint_effort_.resize(num_joints_, 0.0);
  joint_effort_command_.resize(num_joints_, 0.0);
  joint_voltage_command_.resize(num_joints_, 0.0);
  joint_lower_limits_.resize(num_joints_, 0.0);
  joint_upper_limits_.resize(num_joints_, 0.0);
  joint_effort_limits_.resize(num_joints_, 0.0);
}

bool ArmHW::init(ros::NodeHandle& nh,
                 ros::NodeHandle& arm_hw_nh)
{
  nh_ = nh;
  arm_hw_nh_ = arm_hw_nh;

  // Attempt to initialize the URDF model
  if (!urdf_model_.initString(urdf_string_))
  {
    ROS_FATAL_NAMED("arm_hw",
                    "[ArmHW] Unable to parse URDF Model. "
                    "Failed to initialize Arm Interface!");
    return false;
  }

  // Get joint names from the parameter server
  if (!nh_.getParam("joint_names", joint_names_))
  {
    ROS_FATAL_NAMED("arm_hw",
                    "[ArmHW] Failed to get 'joint_names' from "
                    "the parameter server. Failed to initialize Arm Interface!");
    return false;
  }

  if (joint_names_.size() != num_joints_)
  {
    ROS_FATAL_NAMED("arm_hw",
                    "[ArmHW] Incorrect number of joints specified. "
                    "Failed to Initialize the Arm Interface!");
    return false;
  }

  // Ensure the URDF contains the joint names that were specified
  for (std::string joint : joint_names_)
  {
    if (!urdf_model_.getJoint(joint))
    {
      ROS_FATAL_NAMED("arm_hw",
                      "Joint '%s' not found in URDF. "
                      "Failed to initialize the Arm Interface", joint.c_str());
      return false;
    }
  }

  registerInterfaces(urdf_model_);

  // TODO(someshdaga): Consider registering this class as a unique interface using 'registerInterface(this)'
  //                   This will allow us to introduce custom functions for custom controllers if we need
  //                   any in the future.
  ROS_INFO_NAMED("arm_hw",
                 "[ArmHW] Successfully initialized the base Arm Interface!");
  return true;
}

void ArmHW::doSwitch(const std::list<hardware_interface::ControllerInfo>& start_list,
                     const std::list<hardware_interface::ControllerInfo>& stop_list)
{
  // Reset command type for joints claimed by stopping controllers
  for (const auto& controller : stop_list)
  {
    for (const auto& claimed : controller.claimed_resources)
    {
      for (const auto& resource : claimed.resources)
      {
        uint8_t joint_index = joint_index_map_[resource];
        joint_control_method_[joint_index] = ControlMethod::NONE;
        joint_position_command_[joint_index] = joint_position_[joint_index];
        joint_velocity_command_[joint_index] = 0.0;
        joint_effort_command_[joint_index] = 0.0;
        joint_voltage_command_[joint_index] = 0.0;
      }
    }
  }

  for (const auto& controller : start_list)
  {
    for (const auto& claimed : controller.claimed_resources)
    {
      for (const auto& resource : claimed.resources)
      {
        uint8_t joint_index = joint_index_map_[resource];
        if (claimed.hardware_interface == "hardware_interface::PositionJointInterface")
        {
          joint_control_method_[joint_index] = ControlMethod::POSITION;
        }
        else if (claimed.hardware_interface == "hardware_interface::VelocityJointInterface")
        {
          joint_control_method_[joint_index] = ControlMethod::VELOCITY;
        }
        else if (claimed.hardware_interface == "hardware_interface::EffortJointInterface")
        {
          joint_control_method_[joint_index] = ControlMethod::EFFORT;
        }
        else if (claimed.hardware_interface == "hardware_interface::VoltageJointInterface")
        {
          joint_control_method_[joint_index] = ControlMethod::VOLTAGE;
        }
        else
        {
          ROS_ERROR_NAMED("arm_hw",
                          "[ArmHW] Invalid hardware interface '%s' specified"
                          "for resource '%s'",
                          claimed.hardware_interface.c_str(),
                          resource.c_str());
          joint_control_method_[joint_index] = ControlMethod::NONE;
        }
      }
    }
  }
}

void ArmHW::registerInterfaces(const urdf::Model& urdf_model)
{
  // Create and register hardware interface handles for each joint
  for (std::size_t i=0; i < num_joints_; i++)
  {
    // Create a name to index mapping
    joint_index_map_[joint_names_[i]] = i;

    // Get the type of the joint
    joint_types_[i] = urdf_model.getJoint(joint_names_[i])->type;

    // Joint state interface
    joint_state_interface_.registerHandle(
      hardware_interface::JointStateHandle(joint_names_[i],
                                           &joint_position_[i],
                                           &joint_velocity_[i],
                                           &joint_effort_[i]));

    // Create Joint Handles for position, velocity and effort for the given joint
    hardware_interface::JointHandle joint_handle_position,
                                    joint_handle_velocity,
                                    joint_handle_effort,
                                    joint_handle_voltage;
    joint_handle_position = hardware_interface::JointHandle(joint_state_interface_.getHandle(joint_names_[i]),
                                                            &joint_position_command_[i]);
    joint_handle_velocity = hardware_interface::JointHandle(joint_state_interface_.getHandle(joint_names_[i]),
                                                            &joint_velocity_command_[i]);
    joint_handle_effort = hardware_interface::JointHandle(joint_state_interface_.getHandle(joint_names_[i]),
                                                          &joint_effort_command_[i]);
    joint_handle_voltage = hardware_interface::JointHandle(joint_state_interface_.getHandle(joint_names_[i]),
                                                           &joint_voltage_command_[i]);

    // Register the handles for the given joint with the required interfaces
    joint_position_interface_.registerHandle(joint_handle_position);
    joint_velocity_interface_.registerHandle(joint_handle_velocity);
    joint_effort_interface_.registerHandle(joint_handle_effort);
    joint_voltage_interface_.registerHandle(joint_handle_voltage);

    // Register the Joint Limit Handles for the given joint
    registerJointLimits(joint_names_[i],
                        urdf_model_,
                        joint_handle_position,
                        joint_handle_velocity,
                        joint_handle_effort,
                        &joint_lower_limits_[i],
                        &joint_upper_limits_[i],
                        &joint_effort_limits_[i]);
  }

  // Register the interfaces
  registerInterface(&joint_state_interface_);
  registerInterface(&joint_position_interface_);
  registerInterface(&joint_velocity_interface_);
  registerInterface(&joint_effort_interface_);
  registerInterface(&joint_voltage_interface_);
}

void ArmHW::registerJointLimits(const std::string& joint_name,
                                const urdf::Model& urdf_model,
                                const hardware_interface::JointHandle& joint_handle_position,
                                const hardware_interface::JointHandle& joint_handle_velocity,
                                const hardware_interface::JointHandle& joint_handle_effort,
                                double* const joint_lower_limit,
                                double* const joint_upper_limit,
                                double* const joint_effort_limit)
{
  *joint_lower_limit = -std::numeric_limits<double>::max();
  *joint_upper_limit = std::numeric_limits<double>::max();
  *joint_effort_limit = std::numeric_limits<double>::max();

  joint_limits_interface::JointLimits limits;
  joint_limits_interface::SoftJointLimits soft_limits;
  bool has_limits = false;
  bool has_soft_limits = false;

  urdf::JointConstSharedPtr joint = urdf_model.getJoint(joint_name);
  if (joint_limits_interface::getJointLimits(joint, limits))
    has_limits = true;
  if (joint_limits_interface::getSoftJointLimits(joint, soft_limits))
    has_soft_limits = true;

  if (!has_limits)
    return;

  if (limits.has_position_limits)
  {
    *joint_lower_limit = limits.min_position;
    *joint_upper_limit = limits.max_position;
  }

  if (limits.has_effort_limits)
  {
    *joint_effort_limit = limits.max_effort;
  }

  if (has_soft_limits)
  {
    const joint_limits_interface::PositionJointSoftLimitsHandle
      limits_handle_position(joint_handle_position, limits, soft_limits);
    const joint_limits_interface::VelocityJointSoftLimitsHandle
      limits_handle_velocity(joint_handle_velocity, limits, soft_limits);
    const joint_limits_interface::EffortJointSoftLimitsHandle
      limits_handle_effort(joint_handle_effort, limits, soft_limits);
    position_limits_interface_.registerHandle(limits_handle_position);
    velocity_limits_interface_.registerHandle(limits_handle_velocity);
    effort_limits_interface_.registerHandle(limits_handle_effort);
  }
  else
  {
    const joint_limits_interface::PositionJointSaturationHandle
      sat_handle_position(joint_handle_position, limits);
    const joint_limits_interface::VelocityJointSaturationHandle
      sat_handle_velocity(joint_handle_velocity, limits);
    const joint_limits_interface::EffortJointSaturationHandle
      sat_handle_effort(joint_handle_effort, limits);
    position_sat_interface_.registerHandle(sat_handle_position);
    velocity_sat_interface_.registerHandle(sat_handle_velocity);
    effort_sat_interface_.registerHandle(sat_handle_effort);
  }
}

void ArmHW::enforceLimits(ros::Duration period)
{
  position_sat_interface_.enforceLimits(period);
  velocity_sat_interface_.enforceLimits(period);
  effort_sat_interface_.enforceLimits(period);
  position_limits_interface_.enforceLimits(period);
  velocity_limits_interface_.enforceLimits(period);
  effort_limits_interface_.enforceLimits(period);
}

std::string ArmHW::getName() const
{
  return name_;
}
}  // namespace arm
}  // namespace uwrt
