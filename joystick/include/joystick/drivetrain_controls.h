#include "ros.h"

class drivetrain_controls{
    public:
        enum class {
            VOLTAGE,
            VELOCITY,
            POSITION,
        } control_mode_t;

        explicit drivetrain_controls(ros::nodeHandle& nh);
        
        void scaleAndPublish(bool isLowGear, float left_right_axis, float up_down_axis);


    private: 

    control_mode_t control_mode_;
    
    int voltage_high_gear_scalar;
    int voltage_low_gear_scalar;
    int velocity_high_gear_scalar;
    int velocity_low_gear_scalar;
    int position_high_gear_scalar;
    int position_low_gear_scalar;

    ros::publisher drivetrain_voltage_publisher_;
    ros::publisher drivetrain_velocity_publisher_;
    ros::publisher drivetrain_position_publisher_;

    
}