#include <uwrt_mars_rover_xbox_controller/coordinateNode.hpp>

namespace drivetraincontrollerComposition
{
CoordinateNode::CoordinateNode(const rclcpp::NodeOptions & options) : Node("coordinateNode", options) {
    auto callback = [this](const uwrt_mars_rover_xbox_controller::msg::XboxController::SharedPtr msg_in) -> void {
        //Add scaling value
        int vConstant = 5;

        auto msg = std::make_unique<geometry_msgs::msg::Twist>();
        msg->linear.x = (*msg_in).drivetrain_joy_y * vConstant;
        msg->angular.z = (*msg_in).drivetrain_joy_x * vConstant;
        msg->linear.x  = (*msg_in).gimble_joy_y*(vConstant++);
        msg->linear.x  = (*msg_in).gimble_joy_x*(vConstant--);

        //To avoid overflow
        if(vConstant > 2147483647 || vConstant < 0)
            vConstant = 5;

        pub_->publish(std::move(msg));
    };

    pub_ = create_publisher<geometry_msgs::msg::Twist>("/differential_drivetrain_controller/cmd_vel_unstamped", 10);
    drvtrain_joy_sub = create_subscription<uwrt_mars_rover_xbox_controller::msg::XboxController>(
        "/xbox_info", 10, callback);
    }
}  // namespace drivetraincontrollerComposition

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(drivetraincontrollerComposition::CoordinateNode)