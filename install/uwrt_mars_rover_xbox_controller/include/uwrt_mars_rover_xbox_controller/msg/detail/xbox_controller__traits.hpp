// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from uwrt_mars_rover_xbox_controller:msg/XboxController.idl
// generated code does not contain a copyright notice

#ifndef UWRT_MARS_ROVER_XBOX_CONTROLLER__MSG__DETAIL__XBOX_CONTROLLER__TRAITS_HPP_
#define UWRT_MARS_ROVER_XBOX_CONTROLLER__MSG__DETAIL__XBOX_CONTROLLER__TRAITS_HPP_

#include "uwrt_mars_rover_xbox_controller/msg/detail/xbox_controller__struct.hpp"
#include <stdint.h>
#include <rosidl_runtime_cpp/traits.hpp>
#include <sstream>
#include <string>
#include <type_traits>

namespace rosidl_generator_traits
{

inline void to_yaml(
  const uwrt_mars_rover_xbox_controller::msg::XboxController & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: drivetrain_joy_x
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "drivetrain_joy_x: ";
    value_to_yaml(msg.drivetrain_joy_x, out);
    out << "\n";
  }

  // member: drivetrain_joy_y
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "drivetrain_joy_y: ";
    value_to_yaml(msg.drivetrain_joy_y, out);
    out << "\n";
  }

  // member: gimble_joy_x
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "gimble_joy_x: ";
    value_to_yaml(msg.gimble_joy_x, out);
    out << "\n";
  }

  // member: gimble_joy_y
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "gimble_joy_y: ";
    value_to_yaml(msg.gimble_joy_y, out);
    out << "\n";
  }

  // member: lt
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "lt: ";
    value_to_yaml(msg.lt, out);
    out << "\n";
  }

  // member: rt
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "rt: ";
    value_to_yaml(msg.rt, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const uwrt_mars_rover_xbox_controller::msg::XboxController & msg)
{
  std::ostringstream out;
  to_yaml(msg, out);
  return out.str();
}

template<>
inline const char * data_type<uwrt_mars_rover_xbox_controller::msg::XboxController>()
{
  return "uwrt_mars_rover_xbox_controller::msg::XboxController";
}

template<>
inline const char * name<uwrt_mars_rover_xbox_controller::msg::XboxController>()
{
  return "uwrt_mars_rover_xbox_controller/msg/XboxController";
}

template<>
struct has_fixed_size<uwrt_mars_rover_xbox_controller::msg::XboxController>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<uwrt_mars_rover_xbox_controller::msg::XboxController>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<uwrt_mars_rover_xbox_controller::msg::XboxController>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // UWRT_MARS_ROVER_XBOX_CONTROLLER__MSG__DETAIL__XBOX_CONTROLLER__TRAITS_HPP_
