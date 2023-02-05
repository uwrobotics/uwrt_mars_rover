#include <uwrt_mars_rover_xbox_controller/coordinateNode.h>

#include "rclcpp/rclcpp.hpp"

namespace drivetraincontrollerComposition {
    CoordinateNode::CoordinateNode(const rclcpp::NodeOptions &options): Node("coordinateNode", options) {
        //Publisher
        pub_drivetrain_ = create_publisher<geometry_msgs::msg::Twist>("/differential_drivetrain_controller/cmd_vel", 10);

        auto callback =
            [this][const uwrt_mars_rover_xbox_controller::msg::XboxController::SharedPtr msg_in] -> void {
                //Add scaling value
                const vConstant = 5;

                auto msg = std::make_unique<geometry_msgs::msg::Twist>();
                msg->linear.x = CoordinateNode::linear::x*vConstant;
                msg->linear.y = CoordinateNode::linear::y*vConstant;

                msg->angular.x = CoordinateNode::angular::x*vConstant;
                msg->angular.y = CoordinateNode::angular::y*vConstant;
                

                RCLCPP_INFO(this->get_logger(), msg);
            }

        sub_joy = create_subscription<uwrt_mars_rover_xbox_controller::msg::XboxController>("/xbox_info", 10, callback);
        
    }
}

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(drivetraincontrollerComposition::CoordinateNode)


// class CoordinateNode : public rclcpp::Node
// {
//     public:
//         MyNode() : Node("coordinateNode") 
//         {
//             //Publish to drivetrain
//             pub_drivetrain_ =
//                 this->create_publisher<geometry_msgs::msg::TwistMsg>("/differential_drivetrain_controller/cmd_vel", 10);
//             //Subcribe from xbox_info
//             auto sub_joy_cb = std::bind(&CoordinateNode::nodeCallBack, this, std::placeholders::_1);
//             sub_joy = this->create_subscription<uwrt_mars_rover_xbox_controller::msg::XboxController>(
//               "/xbox_info", 10, sub_joy_cb);
//         }
//     private:
//         //Callback
//         void nodeCallBack(const uwrt_mars_rover_xbox_controller::msg::XboxController::SharedPtr msg_in) const {}

//         rclcpp::Publisher<geometry_msgs::msg::TwistMsg>::SharedPtr pub_drivetrain_;
//         rclcpp:Subscription<uwrt_mars_rover_xbox_controller::msg::XboxController>:: SharedPtr sub_joy_;
// };