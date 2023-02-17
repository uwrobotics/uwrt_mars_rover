#ifndef COORDINATENODE_HPP_
#define COORDINATENODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <uwrt_mars_rover_xbox_controller/visibility.h>
#include <uwrt_mars_rover_xbox_controller/msg/xbox_controller.hpp>
#include <geometry_msgs/msg/twist.hpp>

namespace drivetraincontrollerComposition {
    class CoordinateNode : public rclcpp::Node {
        public:
            UWRT_MARS_ROVER_XBOX_CONTROLLER_PUBLIC
            explicit CoordinateNode(const rclcpp::NodeOptions &options);

        private:
            rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_;
            rclcpp::Subscription<uwrt_mars_rover_xbox_controller::msg::XboxController>::SharedPtr drvtrain_joy_sub;
    };
}

#endif