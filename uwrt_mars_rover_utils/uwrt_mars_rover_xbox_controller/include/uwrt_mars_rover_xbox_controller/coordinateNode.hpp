#ifndef COORDINATENODE_HPP_
#define COORDINATENODE_HPP_


#include rclcpp/rclcpp.hpp
#include <uwrt_mars_rover_xbox_controller/visibility.h>

namespace drivetraincontrollerComposition {
    class CoordinateNode : public rclcpp::Node {
        public:
            UWRT_MARS_ROVER_COORDINATENODE_PUBLIC
            explicit CoordinateNode(const rclcpp::NodeOptions &options)

        private:
            rclcpp::Subscription<uwrt_mars_rover_xbox_controller::msg::XboxController>::SharedPtr sub_joy_;
    }
}

#endif