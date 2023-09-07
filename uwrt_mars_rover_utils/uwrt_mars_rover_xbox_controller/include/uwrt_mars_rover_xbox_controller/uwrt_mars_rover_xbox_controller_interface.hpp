#ifndef XBOX_CONTROLLER_H
#define XBOX_CONTROLLER_H

#include <rclcpp/rclcpp.hpp>
#include <uwrt_mars_rover_xbox_controller/visibility.h>
#include <sensor_msgs/msg/joy.hpp>
#include <uwrt_mars_rover_xbox_controller/msg/xbox_controller.hpp>


namespace uwrt_xbox {
    using joy_msg = sensor_msgs::msg::Joy;
    using xbox_msg = uwrt_mars_rover_xbox_controller::msg::XboxController;
    class UWRTXboxController: public rclcpp::Node {
    public:
        UWRT_MARS_ROVER_XBOX_CONTROLLER_PUBLIC
        explicit UWRTXboxController(const rclcpp::NodeOptions &options);
    
    private:
        rclcpp::Subscription<joy_msg>::SharedPtr joy_node_sub;
        rclcpp::Publisher<xbox_msg>::SharedPtr xbox_node_pub;
        // publish to a generic 'manipulated xbox controller' topic
        rclcpp::TimerBase::SharedPtr pub_timer;
        

        struct JsData {
            float drivetrain_js_x;
            float drivetrain_js_y;
            float gimble_js_x;
            float gimble_js_y;
        };

        JsData joystick_data;

        UWRT_MARS_ROVER_XBOX_CONTROLLER_LOCAL
        void getXboxData(const joy_msg::SharedPtr msg);
        UWRT_MARS_ROVER_XBOX_CONTROLLER_LOCAL
        void publishStructuredXboxData();


    };
}

#endif // XBOX_CONTROLLER_H