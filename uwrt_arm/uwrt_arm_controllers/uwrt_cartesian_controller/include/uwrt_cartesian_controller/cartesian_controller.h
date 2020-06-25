/**
Copyright (c) 2020 Somesh Daga <s2daga@uwaterloo.ca>

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#pragma once

#include "uwrt_arm_msgs/UWRTArmTwist.h"
#include <geometry_msgs/TwistStamped.h>

#include <ros/ros.h>

#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>

#include <boost/scoped_ptr.hpp>
#include <kdl/chain.hpp>
#include <kdl/tree.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/frames.hpp>
#include <kdl/jacobian.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/chainiksolver.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/chainfksolver.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>

namespace velocity_controllers
{
class CartesianController : public controller_interface::Controller<hardware_interface::VelocityJointInterface>
{
public:

  bool init(hardware_interface::VelocityJointInterface* hw, ros::NodeHandle& nh);
  
  void starting(const ros::Time& time);
  void stopping(const ros::Time& time);

  void update(const ros::Time& time, const ros::Duration& period);

private:

  std::string robot_description_, root_name_, tip_name_;
   
  hardware_interface::JointHandle joint_;
  std::vector<hardware_interface::JointHandle> joint_handles_;

  KDL::Tree kdl_tree_;
  KDL::Chain kdl_chain_;

  KDL::Twist cmd_linear_twist_;
  KDL::Twist cmd_angular_twist_;

  boost::scoped_ptr<KDL::ChainIkSolverVel> chain_ik_solver_vel_;
  boost::scoped_ptr<KDL::ChainFkSolverPos> chain_fk_solver_;

  bool got_msg_;
  ros::Time last_msg_;

  ros::Duration dead_man_timeout_;

  ros::Subscriber arm_command_sub_;

  void armCommandCallback(const uwrt_arm_msgs::UWRTArmTwistConstPtr& arm_command);

};  // class CartesianController
}  // namespace velocity_controllers
