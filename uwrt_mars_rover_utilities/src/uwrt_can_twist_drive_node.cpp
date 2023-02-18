#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/u_int64.hpp>
#include <uwrt_msgs/msg/set_vel.hpp>


static constexpr uint32_t[3] DRIVETRAIN_LEFT_ODRIVE_IDs = {0x001,0x002,0x003};
static constexpr uint32_t[3] DRIVETRAIN_RIGHT_ODRIVE_IDs = {0x004,0x005,0x006};

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
        uwrt_msgs::msg::SetVel motor_msg = uwrt_msgs::msg::SetVel();
        motor_msg.vel = 
        auto IDs = {}
        // assuming CAN IDs 1-3 are the left side motors, and CAN IDs 4-6 are the right side motors;
        for(int i = 0; i < 3; i++) {

        }
    }


public:
    CANTwistDriveNode() : Node("CAN_twist_drive_node") {


        // create subscribers
        subscription_uint = this->create_subscription<std_msgs::msg::UInt64>(
            "can_test_uint", TOPIC_BUFFER_SIZE, std::bind(&CanTestNode::sendCanUIntCallback, this, std::placeholders::_1));
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
