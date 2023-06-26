#include "vectornav/vectornav_node.hpp"

using namespace std::chrono_literals;

namespace vectornav {

  VectornavNode::VectornavNode(const rclcpp::NodeOptions &options): Node{"vectornav_node", options} {
    ez = vn::sensors::EzAsyncData::connect(SensorPort, SensorBaudrate);
    this->publisher_imu_ = this->create_publisher<sensor_msgs::msg::Imu>
        ("vectornav/imu", 10);
    this->publisher_gps_ = this->create_publisher<sensor_msgs::msg::NavSatFix>
        ("vectornav/navSatFix", 10);
    
    this->timer_publish_ = this->create_wall_timer(std::chrono::duration<int, std::chrono::milliseconds::period>(3), 
                                                   std::bind(&VectornavNode::publish, this));
  }

  VectornavNode:: ~VectornavNode() {
    ez->disconnect();
    delete ez;
  }

  void VectornavNode:: publish() {
    vn::sensors::CompositeData current_data = ez->currentData();
    publishImu(current_data);
    publishGps(current_data);
    
    // std::cout<<getTimeStamp(current_data)<<std::endl;
  }

  rclcpp::Time VectornavNode:: getTimeStamp(vn::sensors::CompositeData &data) {
    const rclcpp::Time t = now();
    // const uint64_t sensorTime = data.timeStartup();
    // const double dt = t.seconds() - sensorTime * 1e-9;
    //
    // // compute exponential moving average
    // const double alpha = 0.001;  // average over rougly 1000 samples
    // averageTimeDifference_ = averageTimeDifference_ * (1.0 - alpha) + alpha * dt;
    //
    // // adjust sensor time by average difference to ROS time
    // const rclcpp::Time adjustedTime = rclcpp::Time(sensorTime, RCL_SYSTEM_TIME) +
    //                                   rclcpp::Duration::from_seconds(averageTimeDifference_);
    // return (adjustedTime);
    
    // TODO simplified
    return t;
  }

  // Convert from vn::math::vec4f to geometry_msgs::msgs::Quaternion
  geometry_msgs::msg::Quaternion VectornavNode::toMsg(const vn::math::vec4f & rhs) {
    geometry_msgs::msg::Quaternion lhs;
    lhs.x = rhs[0];
    lhs.y = rhs[1];
    lhs.z = rhs[2];
    lhs.w = rhs[3];
    return lhs;
  }

  // Convert from vn::math::vec3f to geometry_msgs::msgs::Vector3
  geometry_msgs::msg::Vector3 VectornavNode::toMsg(const vn::math::vec3f & rhs) {
    geometry_msgs::msg::Vector3 lhs;
    lhs.x = rhs[0];
    lhs.y = rhs[1];
    lhs.z = rhs[2];
    return lhs;
  }

  void VectornavNode::publishImu(vn::sensors::CompositeData &data) {
    auto message = std::make_unique<sensor_msgs::msg::Imu>();
    message->header.stamp = this->getTimeStamp(data);
    message->header.frame_id = this->frame_id;
    
    // set async output to 8 - Quaternion, Magnetic, Acceleration, Angular Rate
    try {
      ez->sensor()->writeAsyncDataOutputType(vn::protocol::uart::VNQMR);
    } catch (...) {
      std::cout << "Error setting async data output type: (VNQMR)" << std::endl;
    }

    // QUATERNION
    if (data.hasQuaternion()) {
      message->orientation = this->toMsg(data.quaternion());
    }

    // ANGULAR RATE
    if (data.hasAngularRate()) {
      message->angular_velocity = this->toMsg(data.angularRate());
    }

    // ACCELERATION
    if (data.hasAcceleration()) {
      message->linear_acceleration = this->toMsg(data.acceleration());
    }
    this->publisher_imu_->publish(std::move(message));
  }

  void VectornavNode:: publishGps(vn::sensors::CompositeData &data) {
    auto message = std::make_unique<sensor_msgs::msg::NavSatFix>();
    message->header.stamp = this->getTimeStamp(data);
    message->header.frame_id = this->frame_id; 

    // set async output to 29 0 gnss, lla format (also includes uncertainty)
    try {
      ez->sensor()->writeAsyncDataOutputType(vn::protocol::uart::VNGPS);
    } catch (...) {
      std::cout << "Error setting async data output type: (VNGPS)" << std::endl;
    }

    vn::math::vec3d pos;
    bool flag = false;
    if (data.hasPositionGpsLla()) {
      pos = data.positionGpsLla(); 
      flag = true;
    } else if (data.hasPositionEstimatedLla()) {
      pos = data.positionEstimatedLla(); 
      flag = true;
    }

    if (flag) {
      message->latitude = pos[0];
      message->longitude = pos[1];
      message->altitude = pos[2];
    }

    if (data.hasPositionUncertaintyGpsNed()) {
      geometry_msgs::msg::Vector3 uncertainty = this->toMsg(data.positionUncertaintyGpsNed());
      
      message->position_covariance = {uncertainty.y, 0.0000, 0.0000, 0.0000, uncertainty.x, 0.0000, 0.0000, 0.0000, uncertainty.z};
      message->position_covariance_type = sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_DIAGONAL_KNOWN;
    } else {
      message->position_covariance_type = sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_UNKNOWN;
    }

    publisher_gps_->publish(std::move(message));
  }


} // namespace vectornav

#include <rclcpp_components/register_node_macro.hpp>

RCLCPP_COMPONENTS_REGISTER_NODE(vectornav::VectornavNode)
