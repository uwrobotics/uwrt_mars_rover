
drivetrain_controls::drivetrain_controls(ros::nodeHandle& nh) {
    // load scaling parameters into member variables ex. nh.param("up_down_axis_low_speed_scale", some_member_variable)
    drivetrain_velocity_publisher_ = nh_.advertise<geometry_msgs::Twist>("/uwrt_mars_rover/drivetrain_velocity_controller/cmd_vel", 1);
    drivetrain_voltage_publisher_  = nh_.advertise<>("", 1);
    drivetrain_position_publisher_ = nh_.advertise<>("", 1);

    voltage_high_gear_scalar  =     1;
    voltage_low_gear_scalar   =     1;
    velocity_high_gear_scalar =     1;
    velocity_low_gear_scalar  =     1;
    position_high_gear_scalar =     1;
    position_low_gear_scalar  =     1;
    
    // initialize publishers
}

drivetrain_controls::void drivetrain_controls::scaleAndPublish(bool isLowGear, float left_right_axis, float up_down_axis) {
    switch (control_mode_){
        case control_mode_t::VOLTAGE:
            //create a Voltage message
            _________________ drivetrain_voltage_______;
            if (isLowGear){
                // scale floats by low gear values and publish voltage values
                _________________ drivetrain_voltage_______ =   voltage_low_gear_scalar * left_right_axis;
                _________________ drivetrain_voltage_______  =   voltage_low_gear_scalar * up_down_axis;
            } else {
                //same thing but with high gear scaling
                _________________ drivetrain_voltage_______ =   voltage_high_gear_scalar * left_right_axis;
                _________________ drivetrain_voltage_______  =   voltage_high_gear_scalar * up_down_axis;
            }
            drivetrain_voltage_publisher_.publish(_________________ drivetrain_voltage_______);
        break;

        case control_mode_t::VELOCITY:
            //create a velocity message
            geometry_msgs::Twist drivetrain_velocity_twist;
            if (isLowGear){
                // scale floats by low gear values and publish voltage values
                drivetrain_velocity_twist.angular.z =   velocity_low_gear_scalar * left_right_axis;
                drivetrain_velocity_twist.linear.x  =   velocity_low_gear_scalar * up_down_axis;
            } else {
                //same thing but with high gear scaling
                drivetrain_velocity_twist.angular.z =   velocity_high_gear_scalar * left_right_axis;
                drivetrain_velocity_twist.linear.x  =   velocity_high_gear_scalar * up_down_axis;
            }
            drivetrain_velocity_publisher_.publish(drivetrain_velocity_twist);
        break;
        
        case control_mode_t::POSITION:
            //create a Voltage message
            _________________drivetrain_position_______;
            if (isLowGear){
                // scale floats by low gear values and publish voltage values
                _________________drivetrain_position_______=   position_low_gear_scalar * left_right_axis;
                _________________drivetrain_position_______=   position_low_gear_scalar * up_down_axis;
            } else {
                //same thing but with high gear scaling
                _________________drivetrain_position_______ =   position_high_gear_scalar * left_right_axis;
                _________________drivetrain_position_______ =  position_high_gear_scalar * up_down_axis;
            }
            drivetrain_voltage_publisher_.publish(_________________drivetrain_position_______);
        break;
    }
}
control_mode_t& operator++(control_mode_t &c){
    using IntType = typename std::underlying_type<control_mode_t>::type
    c = static_cast<control_mode_t> (static_cast <IntType>(c) + 1);

    if(static_cast <IntType>(c) > 3){
        c = static_cast<control_mode_t>(0);
    }
}
control_mode_t operator++( control_mode_t &c, int ) {
  control_mode_t result = c;
  ++c;
  return result;
}
void drivetrain_controls::incrementAndSetControlMode(control_mode_t control_mode){
    control_mode_ = ++control_mode;
}