// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from rcl_interfaces:msg/Parameter.idl
// generated code does not contain a copyright notice

#ifndef RCL_INTERFACES__MSG__DETAIL__PARAMETER__TRAITS_HPP_
#define RCL_INTERFACES__MSG__DETAIL__PARAMETER__TRAITS_HPP_

#include "rcl_interfaces/msg/detail/parameter__struct.hpp"
#include <stdint.h>
#include <rosidl_runtime_cpp/traits.hpp>
#include <sstream>
#include <string>
#include <type_traits>

// Include directives for member types
// Member 'value'
#include "rcl_interfaces/msg/detail/parameter_value__traits.hpp"

namespace rosidl_generator_traits
{

inline void to_yaml(
  const rcl_interfaces::msg::Parameter & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: name
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "name: ";
    value_to_yaml(msg.name, out);
    out << "\n";
  }

  // member: value
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "value:\n";
    to_yaml(msg.value, out, indentation + 2);
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const rcl_interfaces::msg::Parameter & msg)
{
  std::ostringstream out;
  to_yaml(msg, out);
  return out.str();
}

template<>
inline const char * data_type<rcl_interfaces::msg::Parameter>()
{
  return "rcl_interfaces::msg::Parameter";
}

template<>
inline const char * name<rcl_interfaces::msg::Parameter>()
{
  return "rcl_interfaces/msg/Parameter";
}

template<>
struct has_fixed_size<rcl_interfaces::msg::Parameter>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<rcl_interfaces::msg::Parameter>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<rcl_interfaces::msg::Parameter>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // RCL_INTERFACES__MSG__DETAIL__PARAMETER__TRAITS_HPP_
