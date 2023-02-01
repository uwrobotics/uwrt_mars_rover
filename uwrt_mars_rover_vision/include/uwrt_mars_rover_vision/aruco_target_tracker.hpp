#ifndef ARUCO_TARGET_TRACKER_H
#define ARUCO_TARGET_TRACKER_H

#include <rclcpp/rclcpp.hpp>
#include <uwrt_mars_rover_vision/visibility.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <image_transport/image_transport.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/aruco.hpp>
#include <vector>

namespace uwrt_autonomy {

class TargetTracker: public rclcpp::Node {

public:
    TargetTracker(const rclcpp::NodeOptions &options);

private:
    /**
     * @brief Reads images from a zed camera subscriber topic and publishes poses for detected ARUCO tags
     * @param image_msg Image data and metadata from the zed2 camera 
    */
    void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr& image_msg);

    /**
     * @brief Reads intrinstic calibration data from the camera info topic
     * @param info Camera info message
    */
    void camInfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr info);

    /**
     * @brief Converts a rotation vector and translation vector to pose message 
     * @param rvec Compressed rodrigues rotation vector (angle-axis form) which represents a rotation
     * @param tvec Translation vector in 3D
     * @param pose_msg Reference to a pose message 
    */
    void toPoseMsg(cv::Vec3d rvec, cv::Vec3d tvec, geometry_msgs::msg::Pose& pose_msg);

    // pose publisher for aruco tags
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr aruco_pose_pub_;
    // zed2 image topic subscriber
    image_transport::Subscriber camera_sub_;
    // camera info subscriber to get intrinstic calibration params
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;

    // zed calibration stuff
    cv::Mat dist_coefficients_ = cv::Mat::zeros(1, 5, CV_32FC1);
    cv::Mat intrinsic_calib_matrix_;

    // vectors for ARUCO tag rotations and translations (max of 4 aruco codes identified at a time)
    std::vector<cv::Vec3d> rvecs_{std::vector<cv::Vec3d>(4, 0.0)}, tvecs_{std::vector<cv::Vec3d>(4, 0.0)};
    // opencv aruco detector variables
    cv::Ptr<cv::aruco::DetectorParameters> params_;
    cv::Ptr<cv::aruco::Dictionary> dictionary_;
    // location of corners in object's reference frame 
    cv::Mat obj_points_{cv::Mat(4, 1, CV_32FC3)};

    // marker length (both length and width are the same) in meters for an ARUCO marker
    float aruco_marker_len_;
    // whether or not to show a visual output of the codes
    bool display_marker_pose_;
};
 
}

#endif // ARUCO_TARGET_TRACKER_H