
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <string>
#include <uwrt_mars_rover_vision/aruco_target_tracker.hpp>

#include "cv_bridge/cv_bridge.h"

namespace uwrt_autonomy
{
TargetTracker::TargetTracker(const rclcpp::NodeOptions & options) : Node("target_tracker", options)
{
  // get parameters set in the launch file
  this->declare_parameter("aruco_marker_len", 0.2);
  this->declare_parameter("aruco_dict", "");
  this->declare_parameter("display_marker_pose", true);
  aruco_marker_len_ = this->get_parameter("aruco_marker_len").get_parameter_value().get<float>();
  display_marker_pose_ =
    this->get_parameter("display_marker_pose").get_parameter_value().get<bool>();
  std::string aruco_dict =
    this->get_parameter("aruco_dict").get_parameter_value().get<std::string>();
  RCLCPP_DEBUG_STREAM(this->get_logger(), "Marker Len: " << aruco_marker_len_);

  camera_info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
    "/zed2/zed_node/left/camera_info", 10,
    std::bind(&TargetTracker::camInfoCallback, this, std::placeholders::_1));

  rmw_qos_profile_t custom_qos = rmw_qos_profile_default;
  camera_sub_ = image_transport::create_subscription(
    this, "/zed2/zed_node/left/image_rect_color",
    std::bind(&TargetTracker::imageCallback, this, std::placeholders::_1), "raw", custom_qos);

  aruco_pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>("/aruco_pose", 10);

  // do aruco setup stuff
  obj_points_.ptr<cv::Vec3f>(0)[0] =
    cv::Vec3f(-aruco_marker_len_ / 2.f, aruco_marker_len_ / 2.f, 0);
  obj_points_.ptr<cv::Vec3f>(0)[1] = cv::Vec3f(aruco_marker_len_ / 2.f, aruco_marker_len_ / 2.f, 0);
  obj_points_.ptr<cv::Vec3f>(0)[2] =
    cv::Vec3f(aruco_marker_len_ / 2.f, -aruco_marker_len_ / 2.f, 0);
  obj_points_.ptr<cv::Vec3f>(0)[3] =
    cv::Vec3f(-aruco_marker_len_ / 2.f, -aruco_marker_len_ / 2.f, 0);

  params_ = cv::aruco::DetectorParameters::create();
  if (aruco_dict == "") {
    RCLCPP_ERROR_STREAM(this->get_logger(), "Please set an ARUCO dictionary to use");
  } else if (aruco_dict == "4x4_50") {
    // use this dictionary for URC
    dictionary_ = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);
  } else if (aruco_dict == "4x4_100") {
    // use this dictionary for CIRC
    dictionary_ = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_100);
  } else {
    RCLCPP_ERROR_STREAM(this->get_logger(), "Please choose a defined ARUCO dictionary");
  }
}

void TargetTracker::toPoseMsg(cv::Vec3d rvec, cv::Vec3d tvec, geometry_msgs::msg::Pose & pose_msg)
{
  // convert the compressed rodrigues rotation vector into a rotation matrix
  cv::Mat rot_matrix(3, 3, CV_64FC1);
  cv::Rodrigues(rvec, rot_matrix);
  tf2::Matrix3x3 tf2_rot_matrix{
    rot_matrix.at<double>(0, 0), rot_matrix.at<double>(0, 1), rot_matrix.at<double>(0, 2),
    rot_matrix.at<double>(1, 0), rot_matrix.at<double>(1, 1), rot_matrix.at<double>(1, 2),
    rot_matrix.at<double>(2, 0), rot_matrix.at<double>(2, 1), rot_matrix.at<double>(2, 2)};
  tf2::Vector3 tf2_tvec{tvec[0], tvec[1], tvec[2]};
  // Create a transform and convert to a Pose
  tf2::Transform tf2_transform{tf2_rot_matrix, tf2_tvec};
  tf2::toMsg(tf2_transform, pose_msg);
}

void TargetTracker::imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr & image_msg)
{
  cv_bridge::CvImagePtr cv_ptr;
  try {
    cv_ptr = cv_bridge::toCvCopy(image_msg, image_msg->encoding);
  } catch (cv_bridge::Exception & e) {
    RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
    return;
  }
  // receive a bgra8 encoded image, must convert to bgr
  cv::Mat image_rgb;
  cv::cvtColor(cv_ptr->image, image_rgb, cv::COLOR_BGRA2BGR);

  // detect ARUCO markers in the image
  std::vector<int> marker_ids;
  std::vector<std::vector<cv::Point2f>> marker_corners, rejection_candidates;
  cv::aruco::detectMarkers(
    image_rgb, dictionary_, marker_corners, marker_ids, params_, rejection_candidates);
  int num_markers_detected = marker_corners.size();

  // identify visible aruco marker poses
  if (num_markers_detected > 0) {
    // Calculate pose for each marker
    for (int i = 0; i < num_markers_detected; i++) {
      cv::solvePnP(
        obj_points_, marker_corners.at(i), intrinsic_calib_matrix_, dist_coefficients_,
        rvecs_.at(i), tvecs_.at(i));
      // send a pose message stamped of the aruco tag pose
      geometry_msgs::msg::PoseStamped aruco_pose_msg;
      aruco_pose_msg.header.frame_id = std::to_string(marker_ids[i]);
      toPoseMsg(rvecs_[i], tvecs_[i], aruco_pose_msg.pose);
      aruco_pose_pub_->publish(aruco_pose_msg);
      if (display_marker_pose_) {
        // draw the detected poses with coordinate axis of 0.2m in length
        cv::drawFrameAxes(
          image_rgb, intrinsic_calib_matrix_, dist_coefficients_, rvecs_.at(i), tvecs_.at(i), 0.2);
      }
    }
    RCLCPP_DEBUG_STREAM(
      this->get_logger(), "ARUCO at tvec: " << tvecs_.at(0) << " | rvec: " << rvecs_.at(0));
    if (display_marker_pose_) {
      cv::aruco::drawDetectedMarkers(image_rgb, marker_corners, marker_ids);
    }
  }

  if (display_marker_pose_) {
    cv::imshow("Image window", image_rgb);
    cv::waitKey(5);
  }
}

void TargetTracker::camInfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr info)
{
  // hacky work around to only read cam info once
  if (intrinsic_calib_matrix_.size().height == 0) {
    RCLCPP_DEBUG_STREAM(this->get_logger(), info->distortion_model);
    RCLCPP_DEBUG_STREAM(
      this->get_logger(), "fx: " << info->k[0] << " | cx: " << info->k[2] << " | fy: " << info->k[4]
                                 << " | cy: " << info->k[5]);
    int param_num = 0;
    for (const double & disto_param : info->d) {
      dist_coefficients_.at<double>(0, param_num) = disto_param;
      param_num++;
    }
    intrinsic_calib_matrix_ =
      (cv::Mat_<float>(3, 3) << info->k[0], 0, info->k[2], 0, info->k[4], info->k[5], 0, 0, 1);
  }
}

}  // namespace uwrt_autonomy

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(uwrt_autonomy::TargetTracker)
