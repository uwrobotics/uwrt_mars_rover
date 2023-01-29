
#include <uwrt_mars_rover_vision/aruco_target_tracker.hpp>
#include <string>
#include "cv_bridge/cv_bridge.h"

#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

namespace uwrt_autonomy {

TargetTracker::TargetTracker(const rclcpp::NodeOptions& options) : Node("target_tracker", options) 
{
  // todo: figure out how to only call subscription once to not constantly read these messages
  // maybe link the camera info and camera subscription messages together? Only do if necessary
  camera_info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
      "/zed2/zed_node/left/camera_info", 10, std::bind(&TargetTracker::camInfoCallback, this, std::placeholders::_1));

  rmw_qos_profile_t custom_qos = rmw_qos_profile_default;
  camera_sub_ = image_transport::create_subscription(
      this, "/zed2/zed_node/left/image_rect_color",
      std::bind(&TargetTracker::imageCallback, this, std::placeholders::_1), "raw", custom_qos);
  
  aruco_pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>("/aruco_pose", 10);
  
  // do aruco setup stuff
  obj_points_.ptr<cv::Vec3f>(0)[0] = cv::Vec3f(-ARUCO_MARKER_LEN/2.f, ARUCO_MARKER_LEN/2.f, 0);
  obj_points_.ptr<cv::Vec3f>(0)[1] = cv::Vec3f(ARUCO_MARKER_LEN/2.f, ARUCO_MARKER_LEN/2.f, 0);
  obj_points_.ptr<cv::Vec3f>(0)[2] = cv::Vec3f(ARUCO_MARKER_LEN/2.f, -ARUCO_MARKER_LEN/2.f, 0);
  obj_points_.ptr<cv::Vec3f>(0)[3] = cv::Vec3f(-ARUCO_MARKER_LEN/2.f, -ARUCO_MARKER_LEN/2.f, 0);

  // TODO: potentially play with detector params
  params_ = cv::aruco::DetectorParameters::create();
  // TODO: ensure the below predefined dictionary is the correct one
  dictionary_ = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);
}

void TargetTracker::toPoseMsg(cv::Vec3d rvec, cv::Vec3d tvec, geometry_msgs::msg::Pose& pose_msg) 
{
  // convert the compressed rodrigues rotation vector into a rotation matrix
  cv::Mat rot_matrix(3, 3, CV_64FC1);
  cv::Rodrigues(rvec, rot_matrix);
  tf2::Matrix3x3 tf2_rot_matrix{rot_matrix.at<double>(0, 0), rot_matrix.at<double>(0, 1), rot_matrix.at<double>(0, 2),
                                rot_matrix.at<double>(1, 0), rot_matrix.at<double>(1, 1), rot_matrix.at<double>(1, 2),
                                rot_matrix.at<double>(2, 0), rot_matrix.at<double>(2, 1), rot_matrix.at<double>(2, 2)};
  tf2::Vector3 tf2_tvec{ tvec[0], tvec[1], tvec[2] };
  // Create a transform and convert to a Pose
  tf2::Transform tf2_transform {tf2_rot_matrix, tf2_tvec };
  tf2::toMsg(tf2_transform, pose_msg);
}

void TargetTracker::imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr& msg) 
{
  cv_bridge::CvImagePtr cv_ptr;
  try {
    cv_ptr = cv_bridge::toCvCopy(msg, msg->encoding);
  } catch (cv_bridge::Exception& e) {
    RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
    return;
  }
  // receive a bgra8 encoded image, must convert to bgr
  cv::Mat image_rgb;
  cv::cvtColor(cv_ptr->image, image_rgb, cv::COLOR_BGRA2BGR);
  std::vector<int> marker_ids;
  std::vector<std::vector<cv::Point2f>> marker_corners, rejection_candidates;
  cv::aruco::detectMarkers(image_rgb, dictionary_, marker_corners, marker_ids, params_, rejection_candidates);
  int num_markers_detected = marker_corners.size();
  // identify aruco codes
  if (num_markers_detected > 0) 
  {
    // Calculate pose for each marker
    for (int i = 0; i < num_markers_detected; i++) {
      cv::solvePnP(obj_points_, marker_corners.at(i), intrinsic_calib_matrix_, dist_coefficients_, rvecs_.at(i),
                   tvecs_.at(i));
      // send a pose message stamped of the aruco tag pose
      geometry_msgs::msg::PoseStamped aruco_pose_msg;
      aruco_pose_msg.header.frame_id = std::to_string(marker_ids[i]);
      toPoseMsg(rvecs_[i], tvecs_[i], aruco_pose_msg.pose);
      aruco_pose_pub_->publish(aruco_pose_msg);
      // draw the detected poses with coordinate axis of 0.2m in length
      cv::drawFrameAxes(image_rgb, intrinsic_calib_matrix_, dist_coefficients_, rvecs_.at(i), tvecs_.at(i), 0.2);
    }

    // log the points just to check that stuff is making sense
    RCLCPP_DEBUG_STREAM(this->get_logger(), "ARUCO at tvec: " << tvecs_.at(0) << " | rvec: " << rvecs_.at(0));
    cv::aruco::drawDetectedMarkers(image_rgb, marker_corners, marker_ids);
  } 

  // TODO: make optional to show the position on an aruco tag
  cv::imshow("Image window", image_rgb);
  // handle key event -> make cv2 show a new image on the window every 5ms
  cv::waitKey(5);
}

void TargetTracker::camInfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr info) 
{
  // temporary hacky work around to only read cam info once
  if (intrinsic_calib_matrix_.size().height == 0)
  {
  RCLCPP_INFO_STREAM(this->get_logger(), info->distortion_model);
  RCLCPP_INFO_STREAM(this->get_logger(), "fx: " << info->k[0] << " | cx: " << info->k[2] << " | fy: " << info->k[4] << " | cy: " << info->k[5]);
  (void)info;
  int param_num = 0; 
  for (const double &disto_param: info->d)
  {
      dist_coefficients_.at<double>(0, param_num) = disto_param;
      param_num++;
  }
  intrinsic_calib_matrix_ = (cv::Mat_<float>(3,3) <<
      info->k[0], 0, info->k[2],
      0, info->k[4], info->k[5],
      0, 0, 1);
  }
}

}  // namespace uwrt_autonomy

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(uwrt_autonomy::TargetTracker)
