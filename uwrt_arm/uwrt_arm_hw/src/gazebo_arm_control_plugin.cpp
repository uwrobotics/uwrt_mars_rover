/**
Copyright (c) 2020 Somesh Daga <s2daga@uwaterloo.ca>

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/
#include "uwrt_arm_hw/gazebo_arm_control_plugin.h"

#include "uwrt_arm_hw/arm_hw_sim.h"

#include <ros/ros.h>

#include <string>

namespace uwrt
{
namespace arm
{

GazeboArmControlPlugin::GazeboArmControlPlugin()
  : ArmControlLoop("gazebo_arm_control_plugin")
{}

void GazeboArmControlPlugin::Load(gazebo::physics::ModelPtr model,
                                  sdf::ElementPtr sdf)
{
  parent_model_ = model;
  sdf_ = sdf;

  // Ensure we have a parent model
  if (!parent_model_)
  {
    ROS_FATAL_NAMED(name_,
                    "[GazeboArmControlPlugin] No parent model found!");
    return;
  }

  // Ensure the ROS plugin is loaded
  if (!ros::isInitialized())
  {
    ROS_FATAL_NAMED(name_,
                    "[GazeboArmControlPlugin] ROS API plugin not loaded!");
    return;
  }

  // Setup the node handle
  if (sdf_->HasElement("robotNamespace"))
  {
    robot_namespace_ = sdf_->Get<std::string>("robotNamespace");
  }
  else
  {
    robot_namespace_ = parent_model_->GetName();  // Default namespace
  }

  nh_ = ros::NodeHandle(robot_namespace_);

  // Get the robot description from the parameter server
  std::string robot_description;
  if (!nh_.getParam("robot_description", robot_description))
  {
    ROS_FATAL_NAMED(name_,
                    "[GazeboArmControlPlugin] ROS Parameter 'robot_description' "
                    "not found in namespace '%s'", robot_namespace_.c_str());
    return;
  }

  // Setup the ArmHW
  arm_hw_ = std::make_unique<ArmHWSim>(parent_model_, robot_description);

  bool initialized = init();
  if (!initialized)
  {
    ROS_FATAL_NAMED(name_,
                    "[GazeboArmControlPlugin] Failed to initialize the ArmControlLoop!");
    return;
  }

  control_period_ = ros::Duration(1.0 / control_freq_);

  // Listen to the world update event. This event is broadcast every simulation step
  update_connection_ = gazebo::event::Events::ConnectWorldUpdateBegin(
    std::bind(&GazeboArmControlPlugin::worldUpdate, this, std::placeholders::_1));

  ROS_INFO_NAMED(name_,
                 "[GazeboArmControlPlugin] Successfully loaded plugin!");
}

void GazeboArmControlPlugin::Reset()
{
  // Reset timing variables on world reset
  last_rw_time_ = ros::Time();
  last_update_time_ = ros::Time();
}

void GazeboArmControlPlugin::worldUpdate(const gazebo::common::UpdateInfo& info)
{
  const ros::Time now(info.simTime.sec, info.simTime.nsec);
  const bool& update_controllers = (now - last_update_time_) > control_period_;
  update(now, update_controllers);
}

// Register the plugin with Gazebo
GZ_REGISTER_MODEL_PLUGIN(GazeboArmControlPlugin);
}  // namespace arm
}  // namespace uwrt
