
#include <uwrt_mars_rover_vision/aruco_target_tracker.hpp>
// ZED includes
// #include <sl/Camera.hpp>
#include "cv_bridge/cv_bridge.h"

namespace uwrt_autonomy {

TargetTracker::TargetTracker(const rclcpp::NodeOptions& options) : Node("target_tracker", options) 
{
  rmw_qos_profile_t custom_qos = rmw_qos_profile_default;
  camera_sub_ = image_transport::create_subscription(
      this, "/zed2/zed_node/left/image_rect_color",
      std::bind(&TargetTracker::imageCallback, this, std::placeholders::_1), "raw", custom_qos);

  // camera_sub_ = create_subscription<sensor_msgs::Image>("/zed2/zed_node/left/image_rect_color", 10,
  // std::bind(&TargetTracker::imageCallback, this , std::placeholders::_1)); 
  
  aruco_pose_pub_ = create_publisher<geometry_msgs::msg::Pose>("/aruco_pose", 10);
  // todo: figure out how to only call subscription once to not constantly read these messages
  camera_info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
      "/zed2/zed_node/left/camera_info", 10, std::bind(&TargetTracker::camInfoCallback, this, std::placeholders::_1));
  

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

void TargetTracker::imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr& msg) {
  cv_bridge::CvImagePtr cv_ptr;
  try {
    cv_ptr = cv_bridge::toCvCopy(msg, msg->encoding);
  } catch (cv_bridge::Exception& e) {
    RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
    return;
  }

  // assume we are receiving a bgra8 encoded image, must reduce to bgr
  cv::Mat image_rgb;
  cv::cvtColor(cv_ptr->image, image_rgb, cv::COLOR_BGRA2BGR);

  std::vector<int> marker_ids;
  std::vector<std::vector<cv::Point2f>> marker_corners, rejection_candidates;
  cv::aruco::detectMarkers(image_rgb, dictionary_, marker_corners, marker_ids, params_, rejection_candidates);
  int num_markers_detected = marker_corners.size();
  // identify aruco codes
  if (num_markers_detected > 0) {
    // Calculate pose for each marker
    for (int i = 0; i < num_markers_detected; i++) {
      cv::solvePnP(obj_points_, marker_corners.at(i), intrinsic_calib_matrix_, dist_coefficients_, rvecs_.at(i),
                   tvecs_.at(i));
    }

    // log the points just to check that stuff is making sense
    RCLCPP_INFO_STREAM(this->get_logger(), "ARUCO at tvec: " << tvecs_.at(0) << " | rvec: " << rvecs_.at(0));

    // TODO add in functionality for validating detected markers and axes frames (raw c++ code below)
    // for(int i = 0; i < marker_ids.size(); i++)
    // {
    //     // draw the detected poses with coordinate axis of 0.2m in length
    //     cv::drawFrameAxes(image_ocv_c3, intrinsicCalibMatrix, distortionCoefficients, rvecs[i], tvecs[i], 0.2);
    // }
    // cv::aruco::drawDetectedMarkers(image_ocv_c3, marker_corners, marker_ids);
  } else {
    // std::cout << "No ARUCO code detected" << std::endl;
  }

  // transform rvecs and tvecs to quaternions

  // publish message to arcuo pose publisher
  // investigate translation and rotation vector to point (translation vector) and quaternion
}

void TargetTracker::camInfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr info) 
{
  // (void)info;
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
  RCLCPP_DEBUG_STREAM(this->get_logger(), "fx: " << info->k[0] << " | cx: " << info->k[2] << " | fy: " << info->k[4] << " | cy: " << info->k[5]);
}

}  // namespace uwrt_autonomy

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(uwrt_autonomy::TargetTracker)
