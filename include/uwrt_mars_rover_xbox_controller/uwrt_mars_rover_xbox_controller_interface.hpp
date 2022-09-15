#ifndef XBOX_CONTROLLER_H
#define XBOX_CONTROLLER_H

#include <rclcpp/rclcpp.hpp>
#include <uwrt_mars_rover_xbox_controller/visibility.h>
#include <sensor_msgs/msg/joy.hpp>

// TODO: create or add to a launch file, where a joy_node is launch prior to an xbox_controller component being loaded

namespace uwrt_xbox {
    using joy_msg = sensor_msgs::msg::Joy;
    class UWRTXboxController: public rclcpp::Node {
    public:
        UWRT_MARS_ROVER_XBOX_CONTROLLER_PUBLIC
        explicit UWRTXboxController(const rclcpp::NodeOptions &options);
    
    private:
        rclcpp::Subscription<joy_msg>::SharedPtr joy_node_sub;
        // TODO: create a specialized message type to publish joystick data
        // rclcpp::Publisher<>::SharedPtr joy_node_pub;
        // TODO: create a timer to publish to either (JS_topic or drivetrain_topic/gimble_topic/etc)

        struct JsData {
            float drivetrain_js_x;
            float drivetrain_js_y;
            float gimble_js_x;
            float gimble_js_y;
        };

        JsData joystick_data;

        UWRT_MARS_ROVER_XBOX_CONTROLLER_LOCAL
        void getXboxData(const joy_msg::SharedPtr msg);
        // TODO: create a publsher callback func


    };
}

#endif // XBOX_CONTROLLER_H