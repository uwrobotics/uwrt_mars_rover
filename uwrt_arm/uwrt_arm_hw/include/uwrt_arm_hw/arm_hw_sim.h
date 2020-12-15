/**
Copyright (c) 2019 Somesh Daga <s2daga@uwaterloo.ca>

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/
#pragma once

#include "uwrt_arm_hw/arm_hw.h"

#include <gazebo/physics/physics.hh>
#include <ros/ros.h>

#include <boost/shared_ptr.hpp>
#include <string>
#include <vector>

namespace uwrt
{
namespace arm
{
class ArmHWSim : public ArmHW
{
public:
  ArmHWSim(gazebo::physics::ModelPtr model,
           const std::string& urdf_str);

  ArmHWSim(const std::string& name,
           gazebo::physics::ModelPtr model,
           const std::string& urdf_str);
  // NOLINTNEXTLINE(modernize-use-override, cppcoreguidelines-explicit-virtual-functions)   
  virtual bool init(ros::NodeHandle& nh,
                    ros::NodeHandle& arm_hw_nh);
  // NOLINTNEXTLINE(modernize-use-override, cppcoreguidelines-explicit-virtual-functions)   
  virtual void read(const ros::Time& time, const ros::Duration& period);
  // NOLINTNEXTLINE(modernize-use-override, cppcoreguidelines-explicit-virtual-functions)   
  virtual void write(const ros::Time& time, const ros::Duration& period);

private:
  std::string gz_physics_type_;
  gazebo::physics::ModelPtr gz_model_;
  std::vector<gazebo::physics::JointPtr> sim_joints_;
};  // class ArmHWSim

// NOLINTNEXTLINE(modernize-use-using)   
typedef boost::shared_ptr<ArmHWSim> ArmHWSimPtr;

}  // namespace arm
}  // namespace uwrt
