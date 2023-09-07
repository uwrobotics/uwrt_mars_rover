#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>

namespace uwrt_mars_rover_estop{

    class Estop : public rclcpp::Node{
        
    public:
        explicit Estop(const rclcpp::NodeOptions &options);
    
    
    
    private:

        //differential_drivetrain_controller/cmd_vel, which takes in geometry_msgs/Twist messages

        // subscriber - topic where the velocities are put
        rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_subscriber_;
        rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr estop_bool_subscriber;



        //publisher - this is the topic where i will be writting the velocities too so the arm can read it
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher_;


        //timer, this is to check for the boolean status and update it
        rclcpp::TimerBase::SharedPtr timer;

        //store the current estop boolean value here
        bool isEstop = false;
    
    
    };


}