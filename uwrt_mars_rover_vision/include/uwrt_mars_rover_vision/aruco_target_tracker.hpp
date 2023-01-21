#ifndef ARUCO_TARGET_TRACKER_H
#define ARUCO_TARGET_TRACKER_H

#include <rclcpp/rclcpp.hpp>
#include <uwrt_mars_rover_vision/visibility.h>
#include <geometry_msgs/msg/Pose.h>
#include <sensor_msgs/msg/Image.h>
#include "image_transport/image_transport.hpp"
#include "opencv2/highgui/highgui.hpp"

#include <opencv2/aruco.hpp>
#include "opencv2/imgproc.hpp"
#include <opencv2/cvconfig.h>

#include <vector>


namespace uwrt_autonomy {

class TargetTracker {

public:
    TargetTracker(const rclcpp::NodeOptions &options);

private:
    // use image transport for camera subscriber
    image_transport::ImageTransport it_;
    // zed2 image topic subscriber
    image_transport::Subscriber camera_sub_;
    // camera subsciber image callback
    void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr& msg);
    
    // pose publisher for aruco tags
    rclcpp::Publisher<geometry_msgs::Pose>::SharedPtr aruco_pose_pub_;

    // zed calibration stuff
    cv::Mat dist_coefficients_;
    cv::Mat intrinsic_calib_matrix_;

    // vectors for ARUCO tag poses (max of 4 aruco codes identified at a time)
    std::vector<cv::Vec3d> rvecs_(4, 0.0), t_vecs_(4, 0.0);

    // aruco detector things
    cv::Ptr<cv::aruco::DetectorParameters> params_;
    cv::Ptr<cv::aruco::Dictionary> dictionary_;
    // location of corners in object's reference frame 
    // NOTE: below assumes fixed size for aruco tags
    cv::Mat obj_points_(4, 1, CV_32FC3);

    // marker length in meters for ARUCO code. Be sure to change it if the marker length changes
    // TODO: change this into a configurable parameter
    float ARUCO_MARKER_LEN = 18.4 / 100;


}
 
}

#endif // ARUCO_TARGET_TRACKER_H