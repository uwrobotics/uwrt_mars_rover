
#include <uwrt_mars_rover_xbox_controller/uwrt_mars_rover_xbox_controller_interface.hpp>

namespace uwrt_xbox
{
UWRTXboxController::UWRTXboxController(const rclcpp::NodeOptions & options)
: Node("xbox_node", options)
{
  using namespace std::chrono_literals;
  // create publishers and subscribers
  // constantly get data from the sensor messages joy topic
  joy_node_sub = create_subscription<joy_msg>(
    "joy", 10, std::bind(&UWRTXboxController::getXboxData, this, std::placeholders::_1));
  xbox_node_pub = create_publisher<xbox_msg>("/xbox_info", 10);
  pub_timer =
    create_wall_timer(100ms, std::bind(&UWRTXboxController::publishStructuredXboxData, this));
}

void UWRTXboxController::getXboxData(const joy_msg::SharedPtr msg)
{
  auto inJsRange = [](float js_axis) -> bool { return js_axis <= 1 && js_axis >= -1; };
  // check that all values are within the expected range
  if (!(inJsRange(msg->axes[0]) && inJsRange(msg->axes[1]) && inJsRange(msg->axes[3]) &&
        inJsRange(msg->axes[4]))) {
    RCLCPP_WARN(this->get_logger(), "A joystick gave a value outside of the [-1, 1] range.");
  }
  // store all joystick data
  joystick_data.drivetrain_js_x = msg->axes[0];
  joystick_data.drivetrain_js_y = msg->axes[1];
  joystick_data.gimble_js_x = msg->axes[3];
  joystick_data.gimble_js_y = msg->axes[4];
  // RCLCPP_INFO(this->get_logger(), "DT JS_x: %f, DT JS_y: %f", joystick_data.drivetrain_js_x, joystick_data.drivetrain_js_y);
}

void UWRTXboxController::publishStructuredXboxData()
{
  auto data = xbox_msg();
  data.drivetrain_joy_x = joystick_data.drivetrain_js_x;
  data.drivetrain_joy_y = joystick_data.drivetrain_js_y;
  data.gimble_joy_x = joystick_data.gimble_js_x;
  data.gimble_joy_y = joystick_data.gimble_js_y;
  xbox_node_pub->publish(data);
}

}  // namespace uwrt_xbox

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(uwrt_xbox::UWRTXboxController)
