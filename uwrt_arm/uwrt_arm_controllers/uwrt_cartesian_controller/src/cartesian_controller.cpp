/**
Copyright (c) 2020 Aidan Keaveny <akeaveny@uwaterloo.ca>

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

  if(!nh.searchParam("/robot_description", _robot_description)){
      ROS_ERROR("Failed to get robot_description parameter");
      return false;
    }
  
  if (!kdl_parser::treeFromParam(_robot_description, kdl_tree_)){
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
    ROS_ERROR_STREAM("\t"<<root_name_<<" --> "<<tip_name_);
    
    ROS_ERROR_STREAM("\tTree has "<<kdl_tree_.getNrOfJoints()<<" joints");
    ROS_ERROR_STREAM("\tThe joint names are:");
    for(std::vector<KDL::Segment>::const_iterator it = kdl_chain_.segments.begin(); it != kdl_chain_.segments.end(); ++it) {
      ROS_ERROR_STREAM("\t\t"<<it->getJoint().getName());
    }

    ROS_ERROR_STREAM("\tTree has "<<kdl_tree_.getNrOfSegments()<<" segments");
    ROS_ERROR_STREAM("\tThe segments are:");

    KDL::SegmentMap segment_map = kdl_tree_.getSegments();
    KDL::SegmentMap::iterator it;

    for( it=segment_map.begin(); it != segment_map.end(); it++ ){
      ROS_ERROR_STREAM("\t\t"<<(*it).first);
    }

    return false;
  }

  ROS_INFO_STREAM("*** Listing KDL chain from tree: ***");
  ROS_INFO_STREAM("\t"<<root_name_<<" --> "<<tip_name_);
  
  ROS_INFO_STREAM("\tTree has "<<kdl_tree_.getNrOfJoints()<<" joints");
  ROS_INFO_STREAM("\tThe joint names are:");
  for(std::vector<KDL::Segment>::const_iterator it = kdl_chain_.segments.begin(); it != kdl_chain_.segments.end(); ++it) {
    ROS_INFO_STREAM("\t\t"<<it->getJoint().getName());
  }

  ROS_INFO_STREAM("\tTree has "<<kdl_tree_.getNrOfSegments()<<" segments");
  ROS_INFO_STREAM("\tThe segments are:");

  KDL::SegmentMap segment_map = kdl_tree_.getSegments();
  KDL::SegmentMap::iterator it;

  for( it=segment_map.begin(); it != segment_map.end(); it++ ){
    ROS_INFO_STREAM("\t\t"<<(*it).first);
  }

  ROS_INFO_STREAM("***Listing HW Interfaces: ***");    
  // Get joint handles for all of the joints in the chain
  for(std::vector<KDL::Segment>::const_iterator it = kdl_chain_.segments.begin(); it != kdl_chain_.segments.end(); ++it)
  {
    // WE DON'T WANT AN INTERFACE FOR THE FIXED JOINT
    if(it->getJoint().getName() != "arm_shoulder_bicep_joint") {
      joint_handles_.push_back(hw->getHandle(it->getJoint().getName()));
      joint_names_.push_back(it->getJoint().getName());
      ROS_INFO_STREAM("\t\t"<<it->getJoint().getName());
    }
  }
  ROS_INFO_STREAM("***KDL Chain has "<< kdl_chain_.getNrOfJoints() <<" joints ***");    

  // KDL solvers
  // NOLINTNEXTLINE(cppcoreguidelines-owning-memory)
  cart_to_joint_solver_vel_.reset(new KDL::ChainIkSolverVel_pinv(kdl_chain_));

  // Subscribe to twist msg 
  arm_command_sub_ = nh.subscribe<uwrt_arm_msgs::UWRTArmTwist>("twist_arm_cmds",
                                                                1,
                                                                &UWRTCartesianController::armCommandCallback,
                                                                this);

  constexpr double DEFAULT_DEAD_MAN_TIMEOUT{0.2}; // TODO: Default value to be configurable in a cfg file.  
  dead_man_timeout_ = ros::Duration(DEFAULT_DEAD_MAN_TIMEOUT);

  return true;
}


 void UWRTCartesianController::armCommandCallback(const uwrt_arm_msgs::UWRTArmTwistConstPtr& arm_command)
  {
    qdot_cart_.vel(0) = arm_command->twist.linear.x;
    qdot_cart_.vel(1) = arm_command->twist.linear.y;
    qdot_cart_.vel(2) = arm_command->twist.linear.z;
    qdot_cart_.rot(0) = arm_command->twist.angular.x;
    qdot_cart_.rot(1) = arm_command->twist.angular.y;
    qdot_cart_.rot(2) = arm_command->twist.angular.z;

    last_msg_time_ = ros::Time::now();
  }

  void UWRTCartesianController::starting(const ros::Time& /*time*/)
  {
    KDL::SetToZero(qdot_cart_);

    last_msg_time_ = ros::Time::now();
  }
  
  void UWRTCartesianController::stopping(const ros::Time& /*time*/)
  {
  }

  void UWRTCartesianController::update(const ros::Time& time, const ros::Duration& /*period*/)
  {
    if((time - last_msg_time_) >= dead_man_timeout_) 
    {
      KDL::SetToZero(qdot_cart_);

      for (unsigned int i = 0; i < kdl_chain_.getNrOfJoints(); i++)
      {
          joint_handles_[i].setCommand(0.0);
      }

      return;
    }

    // DEBUG_STREAM_NAMED("Calling Update!");

    // get joint positions
    KDL::JntArray  q(joint_handles_.size());            
    for(size_t i=0; i<joint_handles_.size(); i++)
    {
      q(i) = joint_handles_[i].getPosition();
    }

    // get joint velocities from cartesian velocities and current joint positions 
    KDL::JntArray qdot_joint(joint_handles_.size());
    if(cart_to_joint_solver_vel_->CartToJnt(q, qdot_cart_, qdot_joint) < 0)
    {
      ROS_ERROR("Unable to compute cartesian to joint velocity.");
      for (unsigned int i = 0; i < kdl_chain_.getNrOfJoints(); i++)
      {
        joint_handles_[i].setCommand(0.0);
      }

      return;
    }

    // set joint velocities
    for (unsigned int i = 0; i < kdl_chain_.getNrOfJoints(); i++)
    {
      joint_handles_[i].setCommand(qdot_joint(i));
      ROS_INFO_STREAM(i << "\t" << joint_names_[i] << ": " << qdot_joint(i));
    }
    ROS_INFO_STREAM("");
  }
}  // namespace velocity_controllers

PLUGINLIB_EXPORT_CLASS(velocity_controllers::UWRTCartesianController, controller_interface::ControllerBase)
