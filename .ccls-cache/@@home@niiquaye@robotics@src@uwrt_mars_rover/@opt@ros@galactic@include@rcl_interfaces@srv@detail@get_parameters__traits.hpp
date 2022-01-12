// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from rcl_interfaces:srv/GetParameters.idl
// generated code does not contain a copyright notice

#ifndef RCL_INTERFACES__SRV__DETAIL__GET_PARAMETERS__TRAITS_HPP_
#define RCL_INTERFACES__SRV__DETAIL__GET_PARAMETERS__TRAITS_HPP_

#include "rcl_interfaces/srv/detail/get_parameters__struct.hpp"
#include <stdint.h>
#include <rosidl_runtime_cpp/traits.hpp>
#include <sstream>
#include <string>
#include <type_traits>

namespace rosidl_generator_traits
{

inline void to_yaml(
  const rcl_interfaces::srv::GetParameters_Request & msg,
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

inline std::string to_yaml(const rcl_interfaces::srv::GetParameters_Request & msg)
{
  std::ostringstream out;
  to_yaml(msg, out);
  return out.str();
}

template<>
inline const char * data_type<rcl_interfaces::srv::GetParameters_Request>()
{
  return "rcl_interfaces::srv::GetParameters_Request";
}

template<>
inline const char * name<rcl_interfaces::srv::GetParameters_Request>()
{
  return "rcl_interfaces/srv/GetParameters_Request";
}

template<>
struct has_fixed_size<rcl_interfaces::srv::GetParameters_Request>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<rcl_interfaces::srv::GetParameters_Request>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<rcl_interfaces::srv::GetParameters_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

// Include directives for member types
// Member 'values'
#include "rcl_interfaces/msg/detail/parameter_value__traits.hpp"

namespace rosidl_generator_traits
{

inline void to_yaml(
  const rcl_interfaces::srv::GetParameters_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: values
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.values.size() == 0) {
      out << "values: []\n";
    } else {
      out << "values:\n";
      for (auto item : msg.values) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "-\n";
        to_yaml(item, out, indentation + 2);
      }
    }
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const rcl_interfaces::srv::GetParameters_Response & msg)
{
  std::ostringstream out;
  to_yaml(msg, out);
  return out.str();
}

template<>
inline const char * data_type<rcl_interfaces::srv::GetParameters_Response>()
{
  return "rcl_interfaces::srv::GetParameters_Response";
}

template<>
inline const char * name<rcl_interfaces::srv::GetParameters_Response>()
{
  return "rcl_interfaces/srv/GetParameters_Response";
}

template<>
struct has_fixed_size<rcl_interfaces::srv::GetParameters_Response>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<rcl_interfaces::srv::GetParameters_Response>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<rcl_interfaces::srv::GetParameters_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<rcl_interfaces::srv::GetParameters>()
{
  return "rcl_interfaces::srv::GetParameters";
}

template<>
inline const char * name<rcl_interfaces::srv::GetParameters>()
{
  return "rcl_interfaces/srv/GetParameters";
}

template<>
struct has_fixed_size<rcl_interfaces::srv::GetParameters>
  : std::integral_constant<
    bool,
    has_fixed_size<rcl_interfaces::srv::GetParameters_Request>::value &&
    has_fixed_size<rcl_interfaces::srv::GetParameters_Response>::value
  >
{
};

template<>
struct has_bounded_size<rcl_interfaces::srv::GetParameters>
  : std::integral_constant<
    bool,
    has_bounded_size<rcl_interfaces::srv::GetParameters_Request>::value &&
    has_bounded_size<rcl_interfaces::srv::GetParameters_Response>::value
  >
{
};

template<>
struct is_service<rcl_interfaces::srv::GetParameters>
  : std::true_type
{
};

template<>
struct is_service_request<rcl_interfaces::srv::GetParameters_Request>
  : std::true_type
{
};

template<>
struct is_service_response<rcl_interfaces::srv::GetParameters_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // RCL_INTERFACES__SRV__DETAIL__GET_PARAMETERS__TRAITS_HPP_
