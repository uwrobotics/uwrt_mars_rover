#include <uwrt_mars_rover_estop/estop.hpp>

namespace composition{
    estop::estop(const rclcpp::NodeOptions &options) : Node("estop_button", options){
        RCLCPP_INFO(this->get_logger(),"HELLO");

    }
}

#include <rclcpp_components/register_node_macro.hpp>

RCLCPP_COMPONENTS_REGISTER_NODE(composition::estop)
