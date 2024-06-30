// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from uwrt_mars_rover_xbox_controller:msg/XboxController.idl
// generated code does not contain a copyright notice

#ifndef UWRT_MARS_ROVER_XBOX_CONTROLLER__MSG__DETAIL__XBOX_CONTROLLER__BUILDER_HPP_
#define UWRT_MARS_ROVER_XBOX_CONTROLLER__MSG__DETAIL__XBOX_CONTROLLER__BUILDER_HPP_

#include "uwrt_mars_rover_xbox_controller/msg/detail/xbox_controller__struct.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <utility>


namespace uwrt_mars_rover_xbox_controller
{

namespace msg
{

namespace builder
{

class Init_XboxController_rt
{
public:
  explicit Init_XboxController_rt(::uwrt_mars_rover_xbox_controller::msg::XboxController & msg)
  : msg_(msg)
  {}
  ::uwrt_mars_rover_xbox_controller::msg::XboxController rt(::uwrt_mars_rover_xbox_controller::msg::XboxController::_rt_type arg)
  {
    msg_.rt = std::move(arg);
    return std::move(msg_);
  }

private:
  ::uwrt_mars_rover_xbox_controller::msg::XboxController msg_;
};

class Init_XboxController_lt
{
public:
  explicit Init_XboxController_lt(::uwrt_mars_rover_xbox_controller::msg::XboxController & msg)
  : msg_(msg)
  {}
  Init_XboxController_rt lt(::uwrt_mars_rover_xbox_controller::msg::XboxController::_lt_type arg)
  {
    msg_.lt = std::move(arg);
    return Init_XboxController_rt(msg_);
  }

private:
  ::uwrt_mars_rover_xbox_controller::msg::XboxController msg_;
};

class Init_XboxController_gimble_joy_y
{
public:
  explicit Init_XboxController_gimble_joy_y(::uwrt_mars_rover_xbox_controller::msg::XboxController & msg)
  : msg_(msg)
  {}
  Init_XboxController_lt gimble_joy_y(::uwrt_mars_rover_xbox_controller::msg::XboxController::_gimble_joy_y_type arg)
  {
    msg_.gimble_joy_y = std::move(arg);
    return Init_XboxController_lt(msg_);
  }

private:
  ::uwrt_mars_rover_xbox_controller::msg::XboxController msg_;
};

class Init_XboxController_gimble_joy_x
{
public:
  explicit Init_XboxController_gimble_joy_x(::uwrt_mars_rover_xbox_controller::msg::XboxController & msg)
  : msg_(msg)
  {}
  Init_XboxController_gimble_joy_y gimble_joy_x(::uwrt_mars_rover_xbox_controller::msg::XboxController::_gimble_joy_x_type arg)
  {
    msg_.gimble_joy_x = std::move(arg);
    return Init_XboxController_gimble_joy_y(msg_);
  }

private:
  ::uwrt_mars_rover_xbox_controller::msg::XboxController msg_;
};

class Init_XboxController_drivetrain_joy_y
{
public:
  explicit Init_XboxController_drivetrain_joy_y(::uwrt_mars_rover_xbox_controller::msg::XboxController & msg)
  : msg_(msg)
  {}
  Init_XboxController_gimble_joy_x drivetrain_joy_y(::uwrt_mars_rover_xbox_controller::msg::XboxController::_drivetrain_joy_y_type arg)
  {
    msg_.drivetrain_joy_y = std::move(arg);
    return Init_XboxController_gimble_joy_x(msg_);
  }

private:
  ::uwrt_mars_rover_xbox_controller::msg::XboxController msg_;
};

class Init_XboxController_drivetrain_joy_x
{
public:
  Init_XboxController_drivetrain_joy_x()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_XboxController_drivetrain_joy_y drivetrain_joy_x(::uwrt_mars_rover_xbox_controller::msg::XboxController::_drivetrain_joy_x_type arg)
  {
    msg_.drivetrain_joy_x = std::move(arg);
    return Init_XboxController_drivetrain_joy_y(msg_);
  }

private:
  ::uwrt_mars_rover_xbox_controller::msg::XboxController msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::uwrt_mars_rover_xbox_controller::msg::XboxController>()
{
  return uwrt_mars_rover_xbox_controller::msg::builder::Init_XboxController_drivetrain_joy_x();
}

}  // namespace uwrt_mars_rover_xbox_controller

#endif  // UWRT_MARS_ROVER_XBOX_CONTROLLER__MSG__DETAIL__XBOX_CONTROLLER__BUILDER_HPP_
