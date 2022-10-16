#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/u_int32.hpp>
#include <uwrt_mars_rover_utilities/uwrt_can.h>

/**
 * Tests the uwrt_can class.
 * This node is a subscriber that receives messages from a topic and writes them onto the CAN bus.
 * It also has a timer that will periodically read and log messages from the CAN bus.
 */

// ids
static constexpr uint32_t FLOAT_READ_ID1 = 0x001;
static constexpr uint32_t FLOAT_READ_ID2 = 0x002;
static constexpr uint32_t UINT_READ_ID1 = 0x003;
static constexpr uint32_t UINT_READ_ID2 = 0x004;
static constexpr uint32_t FLOAT_WRITE_ID = 0x005;
static constexpr uint32_t UINT_WRITE_ID = 0x006;

// ros constants
static constexpr int TOPIC_BUFFER_SIZE = 10;
static constexpr int LOOP_RATE = 10;

using namespace std::chrono_literals;

class CanTestNode : public rclcpp::Node {

    // two CAN_wrappers to make sure we can create multiple
    uwrt_mars_rover_utilities::UWRTCANWrapper can_wrapper_int;    // NOLINT(readability-identifier-naming)
    uwrt_mars_rover_utilities::UWRTCANWrapper can_wrapper_float;  // NOLINT(readability-identifier-naming)

    // Subscriptions
    rclcpp::Subscription<std_msgs::msg::UInt32>::SharedPtr subscription_uint;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr subscription_float;

    // Vectors that hold the float ids and uint ids for reading
    std::vector<uint32_t> float_ids {FLOAT_READ_ID1, FLOAT_READ_ID2};
    std::vector<uint32_t> uint_ids {UINT_READ_ID1, UINT_READ_ID2};

    // Timer to periodically read from CAN bus
    rclcpp::TimerBase::SharedPtr timer;

    // callback to get new float from topic, and send it over CAN
    void sendCanFloatCallback(const std_msgs::msg::Float32::SharedPtr data) {
        auto msg = (float)data->data;
        if (can_wrapper_float.writeToID<float>(msg, FLOAT_WRITE_ID)) {
            RCLCPP_INFO(this->get_logger(), "Successfully sent float msg '%f' to id 0x05", msg);
        } else {
            RCLCPP_INFO(this->get_logger(), "Failed to send float msg '%f' to id 0x05", msg);
        }
    }

    // callback to get new int from topic, and send it over CAN
    void sendCanUIntCallback(const std_msgs::msg::UInt32::SharedPtr data) {
        auto msg = (uint32_t)data->data;
        if (can_wrapper_int.writeToID<uint32_t>(msg, UINT_WRITE_ID)) {
            RCLCPP_INFO(this->get_logger(), "Successfully sent uint32_t msg '%d' to id 0x06", msg);
        } else {
            RCLCPP_INFO(this->get_logger(), "Failed to send uint32_t msg '%d' to id 0x06", msg);
        }
    }

    // Read can messages from the CAN bus, this function will be called periodically
    void readCanMessages() {
        // try and read float ids
        for (const auto& id : float_ids) {
            float data;
            if (can_wrapper_float.getLatestFromID<float>(data, id)) {
                RCLCPP_INFO(this->get_logger(), "Successfully read float msg '%f' from id '%d'", data, id);
            }
        }

        // try and read uint ids
        for (const auto& id : uint_ids) {
            uint32_t data;
            if (can_wrapper_int.getLatestFromID<uint32_t>(data, id)) {
                RCLCPP_INFO(this->get_logger(), "Successfully read uint32_t msg '%d' from id '%d'", data, id);
            }
        }
    }

public:
    CanTestNode() : Node("can_test_node") {
        // Get the can interface name from the launch parameters
        this->declare_parameter<std::string>("CAN_interface");
        rclcpp::Parameter interface_param = this->get_parameter("CAN_interface");
        std::string can_interface;
        can_interface = interface_param.as_string();

        // Note: you can set launch parameters as command line arguments:
        // ros2 run uwrt_mars_rover_utilities uwrt_can_test_node --ros-args -p CAN_interface:="can0"
        
        // create the two CAN wrappers
        // "can_test_int" and "can_test_float" are the topic names
        can_wrapper_int = uwrt_mars_rover_utilities::UWRTCANWrapper("can_test_int", can_interface, true);
        can_wrapper_float = uwrt_mars_rover_utilities::UWRTCANWrapper("can_test_float", can_interface, true);

        // initialize the CAN wrappers with the proper IDs
        can_wrapper_int.init(uint_ids);
        can_wrapper_float.init(float_ids);

        // create subscribers
        subscription_uint = this->create_subscription<std_msgs::msg::UInt32>(
            "can_test_uint", TOPIC_BUFFER_SIZE, std::bind(&CanTestNode::sendCanUIntCallback, this, std::placeholders::_1));
        subscription_float = this->create_subscription<std_msgs::msg::Float32>(
            "can_test_float", TOPIC_BUFFER_SIZE, std::bind(&CanTestNode::sendCanFloatCallback, this, std::placeholders::_1));

        // Now, we need to create a timer to periodically read from the CAN bus
        // This timer will call the readCanMessages function

        // create and run the timer
        timer = this->create_wall_timer(10ms, std::bind(&CanTestNode::readCanMessages, this));
    }
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CanTestNode>());
    rclcpp::shutdown();
    return 0;
}
