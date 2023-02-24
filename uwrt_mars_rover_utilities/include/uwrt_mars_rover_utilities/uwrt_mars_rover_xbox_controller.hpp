#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <uwrt_msgs/msg/xbox_controller.hpp>
#include <geometry_msgs/msg/twist.hpp>

const float XBOX_DEADZONE_X = 0.1;
const float XBOX_DEADZONE_Y = 0.1;

namespace uwrt_xbox {
    
using joy_msg = sensor_msgs::msg::Joy;
using xbox_msg = uwrt_msgs::msg::XboxController;
using twist_msg = geometry_msgs::msg::Twist;

class UWRTXboxController: public rclcpp::Node {
public:
    explicit UWRTXboxController();

private:
    rclcpp::Subscription<joy_msg>::SharedPtr joy_node_sub_;
    rclcpp::Publisher<xbox_msg>::SharedPtr xbox_node_pub_;
    rclcpp::Publisher<twist_msg>::SharedPtr joystick_vel_pub_;
    // publish to a generic 'manipulated xbox controller' topic
    rclcpp::TimerBase::SharedPtr pub_timer;
    

    struct JsData {
        float drivetrain_js_x;
        float drivetrain_js_y;
        float gimble_js_x;
        float gimble_js_y;
    };

    JsData joystick_data;

    const int joystick_vel_scale_ = 5;


    void getXboxData(const joy_msg::SharedPtr msg);
    void publishStructuredXboxData();
    void publishJoyVelData();

};
}