pragma once

#include <rclcpp/rclcpp.hpp>
#include <rcpputils/asserts.hpp>
#include <string>

#include "dinolite/camera_context.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/image.hpp"

namespace dinolite {
class CamNode : public rclcpp::Node {
 public:
  explicit CamNode(const rclcpp::NodeOptions& options);

 private:
  void frame();

  CameraContext cxt_;
  std::shared_ptr<cv::VideoCapture> capture_;

  sensor_msgs::msg::CameraInfo camera_info_msg_;

  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_pub_;

  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace dinolite
