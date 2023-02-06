#include <uwrt_mars_rover_xbox_controller/coordinateNode.h>

#include "rclcpp/rclcpp.hpp"

namespace drivetraincontrollerComposition {
    CoordinateNode::CoordinateNode(const rclcpp::NodeOptions &options): Node("coordinateNode", options) {
        auto callback =
            [this][const uwrt_mars_rover_xbox_controller::msg::XboxController::SharedPtr msg_in] -> void {
                //Add scaling value
                const vConstant = 5;

                auto msg = std::make_unique<geometry_msgs::msg::Twist>();
                msg->linear.x = CoordinateNode::coordinates::linear::x*vConstant;
                msg->linear.y = CoordinateNode::coordinates::linear::y*vConstant;

                this->publisher->publish(std::move(msg));
            }
        
        this->publisher = this->create_publisher<geometry_msgs::msg::Twist>("/differential_drivetrain_controller/cmd_vel", 10);
        sub_joy = create_subscription<uwrt_mars_rover_xbox_controller::msg::XboxController>("/xbox_info", 10, callback);
        
    }
}

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(drivetraincontrollerComposition::CoordinateNode)