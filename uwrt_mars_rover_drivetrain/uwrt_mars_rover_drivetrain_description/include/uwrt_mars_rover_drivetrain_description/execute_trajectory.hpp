#pragma once

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "nav2_msgs/action/compute_path_to_pose.hpp"
#include "nav2_msgs/action/follow_path.hpp"
#include "nav_msgs/msg/path.hpp"

#include "geometry_msgs/msg/point_stamped.hpp"

class ExecuteTrajectory : public rclcpp::Node
{
public:
    ExecuteTrajectory();

private:
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr pointSubscriber_;
    rclcpp_action::Client<nav2_msgs::action::ComputePathToPose>::SharedPtr pathClient_;
    rclcpp_action::Client<nav2_msgs::action::FollowPath>::SharedPtr followClient_;

    nav_msgs::msg::Path path_;

    void handleReceivedPoint(const geometry_msgs::msg::PointStamped point);
    void pathGoalReceivedCallback(const rclcpp_action::ClientGoalHandle<nav2_msgs::action::ComputePathToPose>::WrappedResult & result);
    void followGoalReceivedCallback(const rclcpp_action::ClientGoalHandle<nav2_msgs::action::FollowPath>::WrappedResult & result);
    void followGoalFeedbackCallback(rclcpp_action::ClientGoalHandle<nav2_msgs::action::FollowPath>::SharedPtr,
        const std::shared_ptr<const nav2_msgs::action::FollowPath::Feedback> feedback);
};
