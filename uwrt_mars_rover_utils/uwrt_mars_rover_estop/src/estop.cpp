#include <uwrt_mars_rover_estop/estop.hpp>

namespace composition{
    estop::estop(const rclcpp::NodeOptions &options) : Node("estop_button", options){
        RCLCPP_INFO(this->get_logger(),"HELLO");

        //create publisher for middle man topic
        cmd_vel_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("estop_vel_interceptor",10);
        
        //callback for checking if the enter key has been pressed
        auto estop_callback = [this](const std_msgs::msg::String::SharedPtr msg) -> void {
            
            //if enter is presed then the estop toggle will be enabled
            if(strlen(msg->data.c_str()) == 0){
                RCLCPP_INFO(this->get_logger(),"ESTOP WORKS");
                isEstop = true;
            }

            
                                
        };


        //callback for the subscriber will read the values for the drivetrain topic
        //we check the state of the estop boolean here
        auto cmd_subscriber_callback = [this](const geometry_msgs::msg::Twist::SharedPtr msg) -> void {
            // RCLCPP_INFO(this->get_logger(),"in callback WORKS");
            
            auto twist_msg = geometry_msgs::msg::Twist();
            
            if(isEstop){

                RCLCPP_INFO(this->get_logger(),"inside the stopper");
                //populate all values of Twist to be 0
                twist_msg.angular.x = 0;
                twist_msg.angular.y = 0;
                twist_msg.angular.z = 0;

                twist_msg.linear.x = 0;
                twist_msg.linear.y = 0;
                twist_msg.linear.z = 0;
                
                //publish values to interceptor topic
                this->cmd_vel_publisher_->publish(twist_msg);

            }

            else{
                RCLCPP_INFO(this->get_logger(),"FORWARDIN THE VALUES");
                twist_msg.angular.x = msg->angular.x;
                twist_msg.angular.y = msg->angular.y;
                twist_msg.angular.z = msg->angular.z;

                twist_msg.linear.x = msg->angular.x;
                twist_msg.linear.y = msg->angular.y;
                twist_msg.linear.z = msg->angular.z;
                this->cmd_vel_publisher_->publish(twist_msg);
                
            }
        };
        
        


        //subscriber for bool value
        estop_bool_subscriber = this->create_subscription<std_msgs::msg::String>("/estop", 10, estop_callback);

        //subscirber to the drivetrain veclocity topic
        cmd_subscriber_ = this->create_subscription<geometry_msgs::msg::Twist>("/differential_drivetrain_controller/cmd_vel", 10, cmd_subscriber_callback);


    }
}

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(composition::estop)
