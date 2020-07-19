/**
Copyright (c) 2019 Somesh Daga <s2daga@uwaterloo.ca>

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/
#include "uwrt_arm_hw/arm_hw_sim.h"

#include <angles/angles.h>
#include <ros/ros.h>

#include <string>
#include <vector>

namespace uwrt
{
namespace arm
{
ArmHWSim::ArmHWSim(gazebo::physics::ModelPtr model,
                   const std::string& urdf_str)
  : ArmHWSim("arm_hw_sim", model, urdf_str)
{}

ArmHWSim::ArmHWSim(const std::string& name,
                   gazebo::physics::ModelPtr model,
                   const std::string& urdf_str)
  : ArmHW(name, urdf_str),
    gz_model_(model)
{}

bool ArmHWSim::init(ros::NodeHandle& nh,
                    ros::NodeHandle& arm_hw_nh)
{
  if (!ArmHW::init(nh, arm_hw_nh))
    return false;

  // Ensure the gazebo model has all the required joints
  sim_joints_.clear();
  for (std::size_t i=0; i < num_joints_; i++)
  {
    gazebo::physics::JointPtr joint = gz_model_->GetJoint(joint_names_[i]);
    if (!joint)
    {
      ROS_FATAL_NAMED("arm_hw_sim",
                      "[ArmHWSim] This robot has a joint named '%s' "
                      "which is not in the gazebo model.",
                      joint_names_[i].c_str());
      return false;
    }

    // TODO(someshdaga): Seems like this is needed for the simulation
    //                   to behave properly. Verified that is at least
    //                   needed for direct position control (not using efforts)
    //                   in Gazebo. Not sure if this settings causes issues
    //                   with working in other control modes e.g. velocity/effort
    joint->SetParam("fmax", 0, joint_effort_limits_[i]);
    sim_joints_.push_back(joint);
  }

  // Get the physics engine type for gazebo
  gz_physics_type_ = gazebo::physics::get_world()->Physics()->GetType();

  ROS_INFO_NAMED("arm_hw_sim",
                 "[ArmHWSim] Successfully initialized the simulated Arm Interface!");

  return true;
}

void ArmHWSim::read(const ros::Time& time, const ros::Duration& period)
{
  for (std::size_t i=0; i < num_joints_; i++)
  {
    double position = sim_joints_[i]->Position(0);

    if (joint_types_[i] == urdf::Joint::PRISMATIC)
      joint_position_[i] = position;
    else
      joint_position_[i] +=
        angles::shortest_angular_distance(joint_position_[i],
                                          position);

    joint_velocity_[i] = sim_joints_[i]->GetVelocity(0);
    joint_effort_[i] = sim_joints_[i]->GetForce((unsigned int)(0));
  }
}

void ArmHWSim::write(const ros::Time& time, const ros::Duration& period)
{
  // Depending on the type of control method for the joint,
  // set the appropriate variables (e.g. position, velocity, effort)
  for (std::size_t i=0; i < num_joints_; i++)
  {
    switch (joint_control_method_[i])
    {
      case ControlMethod::POSITION:
        sim_joints_[i]->SetPosition(0, joint_position_command_[i], true);
        break;
      case ControlMethod::VELOCITY:
        if (gz_physics_type_.compare("ode") == 0)
        {
          sim_joints_[i]->SetParam("vel", 0, joint_velocity_command_[i]);
          sim_joints_[i]->SetVelocity(0, joint_velocity_command_[i]);
        }
        else
        {
          sim_joints_[i]->SetVelocity(0, joint_velocity_command_[i]);
        }
        break;
      case ControlMethod::EFFORT:
        sim_joints_[i]->SetForce(0, joint_effort_command_[i]);
        break;
    }
  }
}

}  // namespace arm
}  // namespace uwrt
