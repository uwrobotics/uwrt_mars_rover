// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from rcl_interfaces:msg/IntegerRange.idl
// generated code does not contain a copyright notice

#ifndef RCL_INTERFACES__MSG__DETAIL__INTEGER_RANGE__TRAITS_HPP_
#define RCL_INTERFACES__MSG__DETAIL__INTEGER_RANGE__TRAITS_HPP_

#include "rcl_interfaces/msg/detail/integer_range__struct.hpp"
#include <stdint.h>
#include <rosidl_runtime_cpp/traits.hpp>
#include <sstream>
#include <string>
#include <type_traits>

namespace rosidl_generator_traits
{

inline void to_yaml(
  const rcl_interfaces::msg::IntegerRange & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: from_value
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "from_value: ";
    value_to_yaml(msg.from_value, out);
    out << "\n";
  }

  // member: to_value
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "to_value: ";
    value_to_yaml(msg.to_value, out);
    out << "\n";
  }

  // member: step
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "step: ";
    value_to_yaml(msg.step, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const rcl_interfaces::msg::IntegerRange & msg)
{
  std::ostringstream out;
  to_yaml(msg, out);
  return out.str();
}

template<>
inline const char * data_type<rcl_interfaces::msg::IntegerRange>()
{
  return "rcl_interfaces::msg::IntegerRange";
}

template<>
inline const char * name<rcl_interfaces::msg::IntegerRange>()
{
  return "rcl_interfaces/msg/IntegerRange";
}

template<>
struct has_fixed_size<rcl_interfaces::msg::IntegerRange>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<rcl_interfaces::msg::IntegerRange>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<rcl_interfaces::msg::IntegerRange>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // RCL_INTERFACES__MSG__DETAIL__INTEGER_RANGE__TRAITS_HPP_
