#pragma once

#include <chrono>
#include <functional>
#include <memory>
#include <string>

// ROS2
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"

// VectorNav libvncxx
#include "vn/ezasyncdata.h"

namespace vectornav {

class VectornavNode : public rclcpp::Node {
 public:
  VectornavNode(const rclcpp::NodeOptions& options);
  ~VectornavNode() override;
 private:
  // function
  void publish();

  rclcpp::Time getTimeStamp(vn::sensors::CompositeData& data);

  void publishImu(vn::sensors::CompositeData &data);
  void publishGps(vn::sensors::CompositeData &data);

  // variables
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr publisher_imu_;
  rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr publisher_gps_;

  rclcpp::TimerBase::SharedPtr timer_publish_;

  vn::sensors::EzAsyncData* ez;
  const std::string SensorPort = "/dev/ttyUSB0";    // Linux format for virtual (USB) serial port.
  // const string SensorPort = "/dev/ttyS1";   // Linux format for physical serial port.
  const uint32_t SensorBaudrate = 115200;
  const std::string frame_id = "VN300";
  const float rate = 400.0f;
  std::chrono::milliseconds delay{static_cast<long int>(1000 / rate)};
};

geometry_msgs::msg::Quaternion toMsg(const vn::math::vec4f &rhs);
geometry_msgs::msg::Vector3 toMsg(const vn::math::vec3f & rhs);

} // namespace vectornav
