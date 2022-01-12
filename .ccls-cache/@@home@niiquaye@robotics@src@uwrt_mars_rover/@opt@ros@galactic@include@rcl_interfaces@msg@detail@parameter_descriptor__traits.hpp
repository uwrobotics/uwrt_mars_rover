// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from rcl_interfaces:msg/ParameterDescriptor.idl
// generated code does not contain a copyright notice

#ifndef RCL_INTERFACES__MSG__DETAIL__PARAMETER_DESCRIPTOR__TRAITS_HPP_
#define RCL_INTERFACES__MSG__DETAIL__PARAMETER_DESCRIPTOR__TRAITS_HPP_

#include "rcl_interfaces/msg/detail/parameter_descriptor__struct.hpp"
#include <stdint.h>
#include <rosidl_runtime_cpp/traits.hpp>
#include <sstream>
#include <string>
#include <type_traits>

// Include directives for member types
// Member 'floating_point_range'
#include "rcl_interfaces/msg/detail/floating_point_range__traits.hpp"
// Member 'integer_range'
#include "rcl_interfaces/msg/detail/integer_range__traits.hpp"

namespace rosidl_generator_traits
{

inline void to_yaml(
  const rcl_interfaces::msg::ParameterDescriptor & msg,
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

  // member: type
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "type: ";
    value_to_yaml(msg.type, out);
    out << "\n";
  }

  // member: description
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "description: ";
    value_to_yaml(msg.description, out);
    out << "\n";
  }

  // member: additional_constraints
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "additional_constraints: ";
    value_to_yaml(msg.additional_constraints, out);
    out << "\n";
  }

  // member: read_only
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "read_only: ";
    value_to_yaml(msg.read_only, out);
    out << "\n";
  }

  // member: dynamic_typing
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "dynamic_typing: ";
    value_to_yaml(msg.dynamic_typing, out);
    out << "\n";
  }

  // member: floating_point_range
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.floating_point_range.size() == 0) {
      out << "floating_point_range: []\n";
    } else {
      out << "floating_point_range:\n";
      for (auto item : msg.floating_point_range) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "-\n";
        to_yaml(item, out, indentation + 2);
      }
    }
  }

  // member: integer_range
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.integer_range.size() == 0) {
      out << "integer_range: []\n";
    } else {
      out << "integer_range:\n";
      for (auto item : msg.integer_range) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "-\n";
        to_yaml(item, out, indentation + 2);
      }
    }
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const rcl_interfaces::msg::ParameterDescriptor & msg)
{
  std::ostringstream out;
  to_yaml(msg, out);
  return out.str();
}

template<>
inline const char * data_type<rcl_interfaces::msg::ParameterDescriptor>()
{
  return "rcl_interfaces::msg::ParameterDescriptor";
}

template<>
inline const char * name<rcl_interfaces::msg::ParameterDescriptor>()
{
  return "rcl_interfaces/msg/ParameterDescriptor";
}

template<>
struct has_fixed_size<rcl_interfaces::msg::ParameterDescriptor>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<rcl_interfaces::msg::ParameterDescriptor>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<rcl_interfaces::msg::ParameterDescriptor>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // RCL_INTERFACES__MSG__DETAIL__PARAMETER_DESCRIPTOR__TRAITS_HPP_
