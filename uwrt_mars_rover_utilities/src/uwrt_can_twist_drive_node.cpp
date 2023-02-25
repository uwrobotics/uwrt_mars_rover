#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/u_int64.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <uwrt_msgs/msg/set_vel.hpp>

// ros constants
static constexpr int TOPIC_BUFFER_SIZE = 10;
static constexpr int LOOP_RATE = 10;

using namespace std::chrono_literals;

class CANTwistDriveNode : public rclcpp::Node {

    // Subscriptions
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr twist_sub;
    
    // Publishers
    rclcpp::Publisher<uwrt_msgs::msg::SetVel>::SharedPtr vel_pub;

    // Timer to periodically read from CAN bus
    rclcpp::TimerBase::SharedPtr timer;


    void sendTwistToMotors(const geometry_msgs::msg::Twist data) {
        
        uint8_t IDs[6] = {uwrt_msgs::msg::SetVel::MOTOR_1_ID,uwrt_msgs::msg::SetVel::MOTOR_2_ID,uwrt_msgs::msg::SetVel::MOTOR_3_ID,
                    uwrt_msgs::msg::SetVel::MOTOR_4_ID,uwrt_msgs::msg::SetVel::MOTOR_5_ID,uwrt_msgs::msg::SetVel::MOTOR_6_ID};
        
        // assuming CAN IDs 1-3 are the left side motors, and CAN IDs 4-6 are the right side motors;
        uwrt_msgs::msg::SetVel motor_msg = uwrt_msgs::msg::SetVel();
        for(int i = 0; i < 3; i++) {
            motor_msg.can_id = IDs[i];
            motor_msg.vel = data.linear.x - data.linear.y;
            this->vel_pub->publish(motor_msg);
        }
        // Send message to right side motors
        for(int i = 3; i < 6; i++) {
            motor_msg.can_id = IDs[i];
            motor_msg.vel = -data.linear.x + data.linear.y;
            this->vel_pub->publish(motor_msg);
        }
    }


public:
    CANTwistDriveNode() : Node("CAN_twist_drive_node") {


        // create subscribers
        twist_sub = this->create_subscription<geometry_msgs::msg::Twist>(
            "/tank_drivetrain_controller/cmd_vel", TOPIC_BUFFER_SIZE, std::bind(&CANTwistDriveNode::sendTwistToMotors, this, std::placeholders::_1));
        vel_pub = this->create_publisher<uwrt_msgs::msg::SetVel>(
            "can_send_vel", TOPIC_BUFFER_SIZE);
    }
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CANTwistDriveNode>());
    rclcpp::shutdown();
    return 0;
}
