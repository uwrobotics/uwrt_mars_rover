#include "uwrt_mars_rover_drivetrain_description/execute_trajectory.hpp"

using std::placeholders::_1;
using std::placeholders::_2;

ExecuteTrajectory::ExecuteTrajectory() : Node("execute_trajectory")
{
  pointSubscriber_ = this->create_subscription<geometry_msgs::msg::PointStamped>("clicked_point", 20, 
      std::bind(&ExecuteTrajectory::handleReceivedPoint, this, _1));

  pathClient_ = rclcpp_action::create_client<nav2_msgs::action::ComputePathToPose>(this, "/compute_path_to_pose");
  followClient_ = rclcpp_action::create_client<nav2_msgs::action::FollowPath>(this, "/follow_path");

  // pathPublisher_ = this->create_publisher<nav_msgs::msg::Path>("/planned_path", 20);
}

void ExecuteTrajectory::handleReceivedPoint(const geometry_msgs::msg::PointStamped point)
{
  RCLCPP_INFO(this->get_logger(), "*** GOT POINT %f, %f, %f", point.point.x, point.point.y, point.point.z);

  // SEND GOAL TO GET PATH

  // if action server not available, abort
  if (!this->pathClient_->wait_for_action_server()) {
    RCLCPP_ERROR(this->get_logger(), "ComputePathToPose action server not available after waiting");
    rclcpp::shutdown();
  }

  auto computePathMsg = nav2_msgs::action::ComputePathToPose::Goal();

  computePathMsg.goal.pose.position.x = point.point.x;
  computePathMsg.goal.pose.position.y = point.point.y;
  computePathMsg.goal.pose.position.z = 0;
  // [TODO] Specify Orientation - using default orientation for now
  computePathMsg.planner_id = "GridBased";
  computePathMsg.goal.header.frame_id = "odom";
  computePathMsg.start.header.frame_id = "odom";

  // send goal and attach callbacks
  auto send_goal_options = rclcpp_action::Client<nav2_msgs::action::ComputePathToPose>::SendGoalOptions();
  send_goal_options.result_callback =
    std::bind(&ExecuteTrajectory::pathGoalReceivedCallback, this, _1);
  this->pathClient_->async_send_goal(computePathMsg, send_goal_options);
}

void ExecuteTrajectory::pathGoalReceivedCallback(const rclcpp_action::ClientGoalHandle<nav2_msgs::action::ComputePathToPose>::WrappedResult & result)
{
  RCLCPP_INFO(this->get_logger(), "path goal received");
  switch (result.code) {
    case rclcpp_action::ResultCode::SUCCEEDED:
      break;
    case rclcpp_action::ResultCode::ABORTED:
      RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
      return;
    case rclcpp_action::ResultCode::CANCELED:
      RCLCPP_ERROR(this->get_logger(), "Goal was canceled");
      return;
    default:
      RCLCPP_ERROR(this->get_logger(), "Unknown result code");
      return;
  }

  // publish resulting path for visualizing
  path_ = result.result->path;
  // pathPublisher_->publish(path_); // publish path to visualize
  
  // SEND GOAL TO FOLLOW PATH

  if (!this->followClient_->wait_for_action_server()) {
    RCLCPP_ERROR(this->get_logger(), "FollowPath action server not available after waiting");
    rclcpp::shutdown();
  }

  auto followPathMsg = nav2_msgs::action::FollowPath::Goal();
  followPathMsg.path = path_;
  followPathMsg.controller_id = "FollowPath";
  followPathMsg.goal_checker_id = "simple_goal_checker";

  auto send_goal_options = rclcpp_action::Client<nav2_msgs::action::FollowPath>::SendGoalOptions();
  send_goal_options.result_callback =
    std::bind(&ExecuteTrajectory::followGoalReceivedCallback, this, _1);
  send_goal_options.feedback_callback = 
    std::bind(&ExecuteTrajectory::followGoalFeedbackCallback, this, _1, _2);
  this->followClient_->async_send_goal(followPathMsg, send_goal_options);
}

void ExecuteTrajectory::followGoalFeedbackCallback(rclcpp_action::ClientGoalHandle<nav2_msgs::action::FollowPath>::SharedPtr,
    const std::shared_ptr<const nav2_msgs::action::FollowPath::Feedback> feedback)
{
  RCLCPP_INFO(this->get_logger(), "got feedback speed: %f, dist to goal: %f", feedback->speed, feedback->distance_to_goal);
}

void ExecuteTrajectory::followGoalReceivedCallback(const rclcpp_action::ClientGoalHandle<nav2_msgs::action::FollowPath>::WrappedResult & result)
{
  RCLCPP_INFO(this->get_logger(), "follow goal received");
  switch (result.code) {
    case rclcpp_action::ResultCode::SUCCEEDED:
      break;
    case rclcpp_action::ResultCode::ABORTED:
      RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
      return;
    case rclcpp_action::ResultCode::CANCELED:
      RCLCPP_ERROR(this->get_logger(), "Goal was canceled");
      return;
    default:
      RCLCPP_ERROR(this->get_logger(), "Unknown result code");
      return;
  }
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ExecuteTrajectory>());
  rclcpp::shutdown();
  return 0;
}
