// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from rcl_interfaces:msg/SetParametersResult.idl
// generated code does not contain a copyright notice

#ifndef RCL_INTERFACES__MSG__DETAIL__SET_PARAMETERS_RESULT__TRAITS_HPP_
#define RCL_INTERFACES__MSG__DETAIL__SET_PARAMETERS_RESULT__TRAITS_HPP_

#include "rcl_interfaces/msg/detail/set_parameters_result__struct.hpp"
#include <stdint.h>
#include <rosidl_runtime_cpp/traits.hpp>
#include <sstream>
#include <string>
#include <type_traits>

namespace rosidl_generator_traits
{

inline void to_yaml(
  const rcl_interfaces::msg::SetParametersResult & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: successful
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "successful: ";
    value_to_yaml(msg.successful, out);
    out << "\n";
  }

  // member: reason
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "reason: ";
    value_to_yaml(msg.reason, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const rcl_interfaces::msg::SetParametersResult & msg)
{
  std::ostringstream out;
  to_yaml(msg, out);
  return out.str();
}

template<>
inline const char * data_type<rcl_interfaces::msg::SetParametersResult>()
{
  return "rcl_interfaces::msg::SetParametersResult";
}

template<>
inline const char * name<rcl_interfaces::msg::SetParametersResult>()
{
  return "rcl_interfaces/msg/SetParametersResult";
}

template<>
struct has_fixed_size<rcl_interfaces::msg::SetParametersResult>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<rcl_interfaces::msg::SetParametersResult>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<rcl_interfaces::msg::SetParametersResult>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // RCL_INTERFACES__MSG__DETAIL__SET_PARAMETERS_RESULT__TRAITS_HPP_
