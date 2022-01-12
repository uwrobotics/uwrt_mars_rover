// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from rcl_interfaces:msg/ParameterValue.idl
// generated code does not contain a copyright notice

#ifndef RCL_INTERFACES__MSG__DETAIL__PARAMETER_VALUE__TRAITS_HPP_
#define RCL_INTERFACES__MSG__DETAIL__PARAMETER_VALUE__TRAITS_HPP_

#include "rcl_interfaces/msg/detail/parameter_value__struct.hpp"
#include <stdint.h>
#include <rosidl_runtime_cpp/traits.hpp>
#include <sstream>
#include <string>
#include <type_traits>

namespace rosidl_generator_traits
{

inline void to_yaml(
  const rcl_interfaces::msg::ParameterValue & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: type
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "type: ";
    value_to_yaml(msg.type, out);
    out << "\n";
  }

  // member: bool_value
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "bool_value: ";
    value_to_yaml(msg.bool_value, out);
    out << "\n";
  }

  // member: integer_value
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "integer_value: ";
    value_to_yaml(msg.integer_value, out);
    out << "\n";
  }

  // member: double_value
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "double_value: ";
    value_to_yaml(msg.double_value, out);
    out << "\n";
  }

  // member: string_value
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "string_value: ";
    value_to_yaml(msg.string_value, out);
    out << "\n";
  }

  // member: byte_array_value
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.byte_array_value.size() == 0) {
      out << "byte_array_value: []\n";
    } else {
      out << "byte_array_value:\n";
      for (auto item : msg.byte_array_value) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        character_value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: bool_array_value
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.bool_array_value.size() == 0) {
      out << "bool_array_value: []\n";
    } else {
      out << "bool_array_value:\n";
      for (auto item : msg.bool_array_value) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: integer_array_value
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.integer_array_value.size() == 0) {
      out << "integer_array_value: []\n";
    } else {
      out << "integer_array_value:\n";
      for (auto item : msg.integer_array_value) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: double_array_value
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.double_array_value.size() == 0) {
      out << "double_array_value: []\n";
    } else {
      out << "double_array_value:\n";
      for (auto item : msg.double_array_value) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: string_array_value
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.string_array_value.size() == 0) {
      out << "string_array_value: []\n";
    } else {
      out << "string_array_value:\n";
      for (auto item : msg.string_array_value) {
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

inline std::string to_yaml(const rcl_interfaces::msg::ParameterValue & msg)
{
  std::ostringstream out;
  to_yaml(msg, out);
  return out.str();
}

template<>
inline const char * data_type<rcl_interfaces::msg::ParameterValue>()
{
  return "rcl_interfaces::msg::ParameterValue";
}

template<>
inline const char * name<rcl_interfaces::msg::ParameterValue>()
{
  return "rcl_interfaces/msg/ParameterValue";
}

template<>
struct has_fixed_size<rcl_interfaces::msg::ParameterValue>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<rcl_interfaces::msg::ParameterValue>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<rcl_interfaces::msg::ParameterValue>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // RCL_INTERFACES__MSG__DETAIL__PARAMETER_VALUE__TRAITS_HPP_
