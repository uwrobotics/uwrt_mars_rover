// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from rcl_interfaces:msg/ListParametersResult.idl
// generated code does not contain a copyright notice

#ifndef RCL_INTERFACES__MSG__DETAIL__LIST_PARAMETERS_RESULT__TRAITS_HPP_
#define RCL_INTERFACES__MSG__DETAIL__LIST_PARAMETERS_RESULT__TRAITS_HPP_

#include "rcl_interfaces/msg/detail/list_parameters_result__struct.hpp"
#include <stdint.h>
#include <rosidl_runtime_cpp/traits.hpp>
#include <sstream>
#include <string>
#include <type_traits>

namespace rosidl_generator_traits
{

inline void to_yaml(
  const rcl_interfaces::msg::ListParametersResult & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: names
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.names.size() == 0) {
      out << "names: []\n";
    } else {
      out << "names:\n";
      for (auto item : msg.names) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: prefixes
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.prefixes.size() == 0) {
      out << "prefixes: []\n";
    } else {
      out << "prefixes:\n";
      for (auto item : msg.prefixes) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        value_to_yaml(item, out);
        out << "\n";
      }
    }
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const rcl_interfaces::msg::ListParametersResult & msg)
{
  std::ostringstream out;
  to_yaml(msg, out);
  return out.str();
}

template<>
inline const char * data_type<rcl_interfaces::msg::ListParametersResult>()
{
  return "rcl_interfaces::msg::ListParametersResult";
}

template<>
inline const char * name<rcl_interfaces::msg::ListParametersResult>()
{
  return "rcl_interfaces/msg/ListParametersResult";
}

template<>
struct has_fixed_size<rcl_interfaces::msg::ListParametersResult>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<rcl_interfaces::msg::ListParametersResult>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<rcl_interfaces::msg::ListParametersResult>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // RCL_INTERFACES__MSG__DETAIL__LIST_PARAMETERS_RESULT__TRAITS_HPP_
