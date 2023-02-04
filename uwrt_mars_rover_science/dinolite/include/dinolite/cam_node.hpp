#pragma once

#include "dinolite/camera_context.hpp"

#include <string>
#include <rclcpp/rclcpp.hpp>

#include "opencv2/highgui/highgui.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/image.hpp"

#include "camera_calibration_parsers/parse.hpp"

namespace dinolite {

class CamNode : public rclcpp::Node {

public:
    explicit CamNode(const rclcpp::NodeOptions &options);

private:
    void validate_parameters();
    void frame();

    void printer();

    CameraContext cxt_;
    std::shared_ptr<cv::VideoCapture> capture_;

    sensor_msgs::msg::CameraInfo camera_info_msg_;

    int publish_fps_;
    rclcpp::Time next_stamp_;

    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_pub_;

    rclcpp::TimerBase::SharedPtr timer_;
};

} // namespace dinolite