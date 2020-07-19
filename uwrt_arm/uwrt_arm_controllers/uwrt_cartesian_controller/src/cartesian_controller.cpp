/**
Copyright (c) 2020 Somesh Daga <s2daga@uwaterloo.ca>

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#include "uwrt_cartesian_controller/cartesian_controller.h"

#include <ros/ros.h>

#include <pluginlib/class_list_macros.h>

#include <kdl_parser/kdl_parser.hpp>

namespace velocity_controllers
{
bool UWRTCartesianController::init(hardware_interface::VelocityJointInterface* hw, ros::NodeHandle& nh)
  { 
    if(!nh.searchParam("/robot_description", robot_description_)){
        ROS_ERROR("Failed to get robot_description parameter");
        return false;
      }
    
    if (!kdl_parser::treeFromParam(robot_description_, kdl_tree_)){
      ROS_ERROR("Failed to construct kdl tree from robot description parameter");
      return false;
    }

    if (!nh.getParam("/arm_cartesian_controller/root_name", root_name_)){
      ROS_ERROR("Failed to get root_name parameter");
      return false;
    }


    if (!nh.getParam("/arm_cartesian_controller/tip_name", tip_name_)){
      ROS_ERROR("Failed to get tip_name parameter");
      return false;
    }

    // Populate the KDL chain
    if(!kdl_tree_.getChain(root_name_, tip_name_, kdl_chain_)){

      ROS_ERROR_STREAM("Failed to get KDL chain from tree: ");
      ROS_ERROR_STREAM("  "<<root_name_<<" --> "<<tip_name_);
      
      ROS_ERROR_STREAM("  Tree has "<<kdl_tree_.getNrOfJoints()<<" joints");
      ROS_ERROR_STREAM("  The joint names are:");
      for(std::vector<KDL::Segment>::const_iterator it = kdl_chain_.segments.begin(); it != kdl_chain_.segments.end(); ++it) {
        ROS_ERROR_STREAM("    "<<it->getJoint().getName());
      }

      ROS_ERROR_STREAM("  Tree has "<<kdl_tree_.getNrOfSegments()<<" segments");
      ROS_ERROR_STREAM("  The segments are:");

      KDL::SegmentMap segment_map = kdl_tree_.getSegments();
      KDL::SegmentMap::iterator it;

      for( it=segment_map.begin(); it != segment_map.end(); it++ ){
        ROS_ERROR_STREAM( "    "<<(*it).first);
      }

      return false;
    }
    
    // Get joint handles for all of the joints in the chain
    for(std::vector<KDL::Segment>::const_iterator it = kdl_chain_.segments.begin(); it != kdl_chain_.segments.end(); ++it)
    {
      // WE DON'T WANT AN INTERFACE FOR "arm_shoulder_bicep_joint"
      if(it->getJoint().getName().compare("arm_shoulder_bicep_joint")){
        joint_handles_.push_back(hw->getHandle(it->getJoint().getName()));
      //   ROS_INFO_STREAM(it->getJoint().getName());
      }
    }

    // KDL solvers
    chain_ik_solver_vel_.reset(new KDL::ChainIkSolverVel_pinv(kdl_chain_));
    chain_fk_solver_.reset(new KDL::ChainFkSolverPos_recursive(kdl_chain_));

    // Subscribe to twist msg 
    arm_command_sub_ = nh.subscribe<uwrt_arm_msgs::UWRTArmTwist>("twist_arm_cmds",
                                                                  1,
                                                                  &UWRTCartesianController::armCommandCallback,
                                                                  this);

    double dead_man_timeout;
    nh.param<double>("dead_man_timeout", dead_man_timeout, 0.2);
    dead_man_timeout_ = ros::Duration(dead_man_timeout);

    return true;
  }

 void UWRTCartesianController::armCommandCallback(const uwrt_arm_msgs::UWRTArmTwistConstPtr& arm_command)
  {
    cmd_linear_twist_.vel(0) = arm_command->twist.linear.x;
    cmd_linear_twist_.vel(1) = arm_command->twist.linear.y;
    cmd_linear_twist_.vel(2) = arm_command->twist.linear.z;
    cmd_linear_twist_.rot(0) = arm_command->twist.angular.x;
    cmd_linear_twist_.rot(1) = arm_command->twist.angular.y;
    cmd_linear_twist_.rot(2) = arm_command->twist.angular.z;

    // cmd_linear_twist_.vel(0) = arm_command->twist.linear.x;
    // cmd_linear_twist_.vel(1) = arm_command->twist.linear.y;
    // cmd_linear_twist_.vel(2) = arm_command->twist.linear.z;
    // cmd_linear_twist_.rot(0) = 0.0;
    // cmd_linear_twist_.rot(1) = 0.0;
    // cmd_linear_twist_.rot(2) = 0.0;

    // cmd_angular_twist_.vel(0) = 0.0;
    // cmd_angular_twist_.vel(1) = 0.0;
    // cmd_angular_twist_.vel(2) = 0.0;
    // cmd_angular_twist_.rot(0) = arm_command->twist.angular.x;
    // cmd_angular_twist_.rot(1) = arm_command->twist.angular.y;
    // cmd_angular_twist_.rot(2) = arm_command->twist.angular.z;

    last_msg_ = ros::Time::now();
    got_msg_ = false;
  }

  void UWRTCartesianController::update(const ros::Time& time, const ros::Duration& period)
  {
    ROS_INFO_STREAM("Calling Update!");
    if((time - last_msg_) >= dead_man_timeout_) 
    {
      cmd_linear_twist_[0] = 0.0;
      cmd_linear_twist_[1] = 0.0;
      cmd_linear_twist_[2] = 0.0;
      cmd_linear_twist_[3] = 0.0;
      cmd_linear_twist_[4] = 0.0;
      cmd_linear_twist_[5] = 0.0;

      // cmd_angular_twist_[0] = 0.0;
      // cmd_angular_twist_[1] = 0.0;
      // cmd_angular_twist_[2] = 0.0;
      // cmd_angular_twist_[3] = 0.0;
      // cmd_angular_twist_[4] = 0.0;
      // cmd_angular_twist_[5] = 0.0;

      got_msg_ = false;

      for (unsigned int i = 0; i < kdl_chain_.getNrOfJoints(); i++)
      {
          joint_handles_[i].setCommand(0.0);
      }

      return;
    }

    // get joint positions
    KDL::JntArray  q(joint_handles_.size());
    for(size_t i=0; i<joint_handles_.size(); i++)
    {
      q(i) = joint_handles_[i].getPosition();
    }

    KDL::Frame frame_tip_pose;
    if(chain_fk_solver_->JntToCart(q, frame_tip_pose) < 0)
    {
      ROS_ERROR("Unable to compute forward kinematic.");
      for (unsigned int i = 0; i < kdl_chain_.getNrOfJoints(); i++)
      {
          joint_handles_[i].setCommand(0.0);
      }

      return;
    }

    KDL::Frame frame_tip_pose_inv = frame_tip_pose.Inverse();

    KDL::Twist linear_twist = frame_tip_pose_inv * cmd_linear_twist_;
    // KDL::Twist angular_twist = frame_tip_pose_inv.M * cmd_angular_twist_;
    
    KDL::Twist twist(linear_twist.vel, linear_twist.rot);

    KDL::JntArray joint_vel(joint_handles_.size());
    if(chain_ik_solver_vel_->CartToJnt(q, twist, joint_vel) < 0)
    {
      ROS_ERROR("Unable to compute cartesian to joint velocity.");
      for (unsigned int i = 0; i < kdl_chain_.getNrOfJoints(); i++)
      {
          joint_handles_[i].setCommand(0.0);
      }

      return;
    }

    // Convert the wrench into joint efforts
    for (unsigned int i = 0; i < kdl_chain_.getNrOfJoints(); i++)
    {
      joint_handles_[i].setCommand(joint_vel(i));
    }
  }

  void UWRTCartesianController::starting(const ros::Time& time)
  {
    // set twists to zero
    cmd_linear_twist_[0] = 0.0;
    cmd_linear_twist_[1] = 0.0;
    cmd_linear_twist_[2] = 0.0;
    cmd_linear_twist_[3] = 0.0;
    cmd_linear_twist_[4] = 0.0;
    cmd_linear_twist_[5] = 0.0;

    // cmd_angular_twist_[0] = 0.0;
    // cmd_angular_twist_[1] = 0.0;
    // cmd_angular_twist_[2] = 0.0;
    // cmd_angular_twist_[3] = 0.0;
    // cmd_angular_twist_[4] = 0.0;
    // cmd_angular_twist_[5] = 0.0;

    last_msg_ = ros::Time::now();
    got_msg_ = false;
  }

  void UWRTCartesianController::stopping(const ros::Time& time)
  {
  }
}  // namespace velocity_controllers

PLUGINLIB_EXPORT_CLASS(velocity_controllers::UWRTCartesianController, controller_interface::ControllerBase)
