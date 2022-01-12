// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from rcl_interfaces:srv/GetParameterTypes.idl
// generated code does not contain a copyright notice

#ifndef RCL_INTERFACES__SRV__DETAIL__GET_PARAMETER_TYPES__TRAITS_HPP_
#define RCL_INTERFACES__SRV__DETAIL__GET_PARAMETER_TYPES__TRAITS_HPP_

#include "rcl_interfaces/srv/detail/get_parameter_types__struct.hpp"
#include <stdint.h>
#include <rosidl_runtime_cpp/traits.hpp>
#include <sstream>
#include <string>
#include <type_traits>

namespace rosidl_generator_traits
{

inline void to_yaml(
  const rcl_interfaces::srv::GetParameterTypes_Request & msg,
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
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const rcl_interfaces::srv::GetParameterTypes_Request & msg)
{
  std::ostringstream out;
  to_yaml(msg, out);
  return out.str();
}

template<>
inline const char * data_type<rcl_interfaces::srv::GetParameterTypes_Request>()
{
  return "rcl_interfaces::srv::GetParameterTypes_Request";
}

template<>
inline const char * name<rcl_interfaces::srv::GetParameterTypes_Request>()
{
  return "rcl_interfaces/srv/GetParameterTypes_Request";
}

template<>
struct has_fixed_size<rcl_interfaces::srv::GetParameterTypes_Request>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<rcl_interfaces::srv::GetParameterTypes_Request>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<rcl_interfaces::srv::GetParameterTypes_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

inline void to_yaml(
  const rcl_interfaces::srv::GetParameterTypes_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: types
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.types.size() == 0) {
      out << "types: []\n";
    } else {
      out << "types:\n";
      for (auto item : msg.types) {
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

inline std::string to_yaml(const rcl_interfaces::srv::GetParameterTypes_Response & msg)
{
  std::ostringstream out;
  to_yaml(msg, out);
  return out.str();
}

template<>
inline const char * data_type<rcl_interfaces::srv::GetParameterTypes_Response>()
{
  return "rcl_interfaces::srv::GetParameterTypes_Response";
}

template<>
inline const char * name<rcl_interfaces::srv::GetParameterTypes_Response>()
{
  return "rcl_interfaces/srv/GetParameterTypes_Response";
}

template<>
struct has_fixed_size<rcl_interfaces::srv::GetParameterTypes_Response>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<rcl_interfaces::srv::GetParameterTypes_Response>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<rcl_interfaces::srv::GetParameterTypes_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<rcl_interfaces::srv::GetParameterTypes>()
{
  return "rcl_interfaces::srv::GetParameterTypes";
}

template<>
inline const char * name<rcl_interfaces::srv::GetParameterTypes>()
{
  return "rcl_interfaces/srv/GetParameterTypes";
}

template<>
struct has_fixed_size<rcl_interfaces::srv::GetParameterTypes>
  : std::integral_constant<
    bool,
    has_fixed_size<rcl_interfaces::srv::GetParameterTypes_Request>::value &&
    has_fixed_size<rcl_interfaces::srv::GetParameterTypes_Response>::value
  >
{
};

template<>
struct has_bounded_size<rcl_interfaces::srv::GetParameterTypes>
  : std::integral_constant<
    bool,
    has_bounded_size<rcl_interfaces::srv::GetParameterTypes_Request>::value &&
    has_bounded_size<rcl_interfaces::srv::GetParameterTypes_Response>::value
  >
{
};

template<>
struct is_service<rcl_interfaces::srv::GetParameterTypes>
  : std::true_type
{
};

template<>
struct is_service_request<rcl_interfaces::srv::GetParameterTypes_Request>
  : std::true_type
{
};

template<>
struct is_service_response<rcl_interfaces::srv::GetParameterTypes_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // RCL_INTERFACES__SRV__DETAIL__GET_PARAMETER_TYPES__TRAITS_HPP_
