#include <uwrt_mars_rover_utilities/uwrt_mars_rover_xbox_controller.hpp>
#include <stdlib.h>

namespace uwrt_xbox {
    
UWRTXboxController::UWRTXboxController(): Node("xbox_node") {
    using namespace std::chrono_literals;
    // create publishers and subscribers
    // constantly get data from the sensor messages joy topic
    joy_node_sub_ = create_subscription<joy_msg>("joy", 10, std::bind(&UWRTXboxController::getXboxData, this , std::placeholders::_1));
    // xbox_node_pub_ = create_publisher<xbox_msg>("/xbox_info", 10);
    // joystick_vel_pub_ = create_publisher<twist_msg>("/tank_drivetrain_controller/cmd_vel", 10);
    vel_pub_ = create_publisher<uwrt_msgs::msg::SetVel>("/can_send_vel", 10);

    // pub_timer = create_wall_timer(100ms, std::bind(&UWRTXboxController::publishStructuredXboxData, this));
    pub_timer = create_wall_timer(100ms, std::bind(&UWRTXboxController::publishJoyVelData, this));
}

void UWRTXboxController::getXboxData(const joy_msg::SharedPtr msg) {
    auto inJsRange = [](float js_axis) -> bool {
        return js_axis <= 1 && js_axis >= -1;
    };
    // check that all values are within the expected range
    if (!(inJsRange(msg->axes[0]) && inJsRange(msg->axes[1]) && inJsRange(msg->axes[3]) && inJsRange(msg->axes[4]))) {
        RCLCPP_WARN(this->get_logger(), "A joystick gave a value outside of the [-1, 1] range.");
    }

    joystick_data.drivetrain_js_x = msg->axes[0];
    joystick_data.drivetrain_js_y = msg->axes[1];
    joystick_data.gimble_js_x = msg->axes[3];
    joystick_data.gimble_js_y = msg->axes[4];
    // RCLCPP_INFO(this->get_logger(), "DT JS_x: %f, DT JS_y: %f", joystick_data.drivetrain_js_x, joystick_data.drivetrain_js_y);
}

void UWRTXboxController::publishJoyVelData()
{      
    // store all joystick data 
    uint8_t IDs[6] = {uwrt_msgs::msg::SetVel::MOTOR_1_ID,uwrt_msgs::msg::SetVel::MOTOR_2_ID,uwrt_msgs::msg::SetVel::MOTOR_3_ID,
                    uwrt_msgs::msg::SetVel::MOTOR_4_ID,uwrt_msgs::msg::SetVel::MOTOR_5_ID,uwrt_msgs::msg::SetVel::MOTOR_6_ID};

    // assuming CAN IDs 0-2 are the left side motors, and CAN IDs 3-5 are the right side motors;
    auto motor_msg = uwrt_msgs::msg::SetVel();
    for(int i = 0; i < 3; i++) {
        motor_msg.can_id = IDs[i];
        // only publish to running motors
        if (motor_msg.can_id % 2 == 0)
        {
            if (abs(joystick_data.drivetrain_js_y) >= XBOX_DEADZONE_Y) {
                motor_msg.vel = joystick_data.drivetrain_js_y * joystick_vel_scale_;
            }
            else {
                motor_msg.vel = 0;
            }
            vel_pub_->publish(motor_msg);
        }
    }
    // Send message to right side motors; make negative to account for motor direction
    for(int i = 3; i < 6; i++) {
        motor_msg.can_id = IDs[i];
        if (motor_msg.can_id % 2 != 0)
        {
            if (abs(joystick_data.gimble_js_y) >= XBOX_DEADZONE_Y) {
                motor_msg.vel = -joystick_data.gimble_js_y * joystick_vel_scale_;
            }
            else {
                motor_msg.vel = 0;
            }
            vel_pub_->publish(motor_msg);
        }
    }
}

// TODO: might be able to remove or refactor this method to do something more useful (like what the above method is doing)
void UWRTXboxController::publishStructuredXboxData() {
    auto data = xbox_msg();
    data.drivetrain_joy_x = joystick_data.drivetrain_js_x;
    data.drivetrain_joy_y = joystick_data.drivetrain_js_y;
    data.gimble_joy_x = joystick_data.gimble_js_x;
    data.gimble_joy_y = joystick_data.gimble_js_y;
    xbox_node_pub_->publish(data);
}

}

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<uwrt_xbox::UWRTXboxController>());
    rclcpp::shutdown();
    return 0;
}
