#ifndef ARUCO_TARGET_TRACKER_H
#define ARUCO_TARGET_TRACKER_H

#include <rclcpp/rclcpp.hpp>
#include <uwrt_mars_rover_vision/visibility.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include "image_transport/image_transport.hpp"
#include "opencv2/highgui/highgui.hpp"

#include <opencv2/aruco.hpp>
#include "opencv2/imgproc.hpp"
#include <opencv2/cvconfig.h>

#include <vector>


namespace uwrt_autonomy {

class TargetTracker: public rclcpp::Node {

public:
    TargetTracker(const rclcpp::NodeOptions &options);

private:
    // zed2 image topic subscriber
    image_transport::Subscriber camera_sub_;
    // camera subsciber image callback
    void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr& msg);
    
    // pose publisher for aruco tags
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr aruco_pose_pub_;

    // make subscriber to get the camera info
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;
    void camInfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr info);

    // convert translation and rotation vector to pose message
    void toPoseMsg(cv::Vec3d rvec, cv::Vec3d tvec, geometry_msgs::msg::Pose& pose_msg);

    // zed calibration stuff
    cv::Mat dist_coefficients_ = cv::Mat::zeros(1, 5, CV_32FC1);
    cv::Mat intrinsic_calib_matrix_;

    // vectors for ARUCO tag poses (max of 4 aruco codes identified at a time)
    std::vector<cv::Vec3d> rvecs_{std::vector<cv::Vec3d>(4, 0.0)}, tvecs_{std::vector<cv::Vec3d>(4, 0.0)};

    // aruco detector things
    cv::Ptr<cv::aruco::DetectorParameters> params_;
    cv::Ptr<cv::aruco::Dictionary> dictionary_;
    // location of corners in object's reference frame 
    // NOTE: below assumes fixed size for aruco tags
    cv::Mat obj_points_{cv::Mat(4, 1, CV_32FC3)};

    // marker length in meters for ARUCO code. Be sure to change it if the marker length changes
    // TODO: change this into a configurable parameter
    float ARUCO_MARKER_LEN = 18.4 / 100;
};
 
}

#endif // ARUCO_TARGET_TRACKER_H