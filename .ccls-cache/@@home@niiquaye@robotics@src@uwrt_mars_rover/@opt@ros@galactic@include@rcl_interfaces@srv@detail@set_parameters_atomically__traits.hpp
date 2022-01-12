// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from rcl_interfaces:srv/SetParametersAtomically.idl
// generated code does not contain a copyright notice

#ifndef RCL_INTERFACES__SRV__DETAIL__SET_PARAMETERS_ATOMICALLY__TRAITS_HPP_
#define RCL_INTERFACES__SRV__DETAIL__SET_PARAMETERS_ATOMICALLY__TRAITS_HPP_

#include "rcl_interfaces/srv/detail/set_parameters_atomically__struct.hpp"
#include <stdint.h>
#include <rosidl_runtime_cpp/traits.hpp>
#include <sstream>
#include <string>
#include <type_traits>

// Include directives for member types
// Member 'parameters'
#include "rcl_interfaces/msg/detail/parameter__traits.hpp"

namespace rosidl_generator_traits
{

inline void to_yaml(
  const rcl_interfaces::srv::SetParametersAtomically_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: parameters
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.parameters.size() == 0) {
      out << "parameters: []\n";
    } else {
      out << "parameters:\n";
      for (auto item : msg.parameters) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "-\n";
        to_yaml(item, out, indentation + 2);
      }
    }
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const rcl_interfaces::srv::SetParametersAtomically_Request & msg)
{
  std::ostringstream out;
  to_yaml(msg, out);
  return out.str();
}

template<>
inline const char * data_type<rcl_interfaces::srv::SetParametersAtomically_Request>()
{
  return "rcl_interfaces::srv::SetParametersAtomically_Request";
}

template<>
inline const char * name<rcl_interfaces::srv::SetParametersAtomically_Request>()
{
  return "rcl_interfaces/srv/SetParametersAtomically_Request";
}

template<>
struct has_fixed_size<rcl_interfaces::srv::SetParametersAtomically_Request>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<rcl_interfaces::srv::SetParametersAtomically_Request>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<rcl_interfaces::srv::SetParametersAtomically_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

// Include directives for member types
// Member 'result'
#include "rcl_interfaces/msg/detail/set_parameters_result__traits.hpp"

namespace rosidl_generator_traits
{

inline void to_yaml(
  const rcl_interfaces::srv::SetParametersAtomically_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: result
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "result:\n";
    to_yaml(msg.result, out, indentation + 2);
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const rcl_interfaces::srv::SetParametersAtomically_Response & msg)
{
  std::ostringstream out;
  to_yaml(msg, out);
  return out.str();
}

template<>
inline const char * data_type<rcl_interfaces::srv::SetParametersAtomically_Response>()
{
  return "rcl_interfaces::srv::SetParametersAtomically_Response";
}

template<>
inline const char * name<rcl_interfaces::srv::SetParametersAtomically_Response>()
{
  return "rcl_interfaces/srv/SetParametersAtomically_Response";
}

template<>
struct has_fixed_size<rcl_interfaces::srv::SetParametersAtomically_Response>
  : std::integral_constant<bool, has_fixed_size<rcl_interfaces::msg::SetParametersResult>::value> {};

template<>
struct has_bounded_size<rcl_interfaces::srv::SetParametersAtomically_Response>
  : std::integral_constant<bool, has_bounded_size<rcl_interfaces::msg::SetParametersResult>::value> {};

template<>
struct is_message<rcl_interfaces::srv::SetParametersAtomically_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<rcl_interfaces::srv::SetParametersAtomically>()
{
  return "rcl_interfaces::srv::SetParametersAtomically";
}

template<>
inline const char * name<rcl_interfaces::srv::SetParametersAtomically>()
{
  return "rcl_interfaces/srv/SetParametersAtomically";
}

template<>
struct has_fixed_size<rcl_interfaces::srv::SetParametersAtomically>
  : std::integral_constant<
    bool,
    has_fixed_size<rcl_interfaces::srv::SetParametersAtomically_Request>::value &&
    has_fixed_size<rcl_interfaces::srv::SetParametersAtomically_Response>::value
  >
{
};

template<>
struct has_bounded_size<rcl_interfaces::srv::SetParametersAtomically>
  : std::integral_constant<
    bool,
    has_bounded_size<rcl_interfaces::srv::SetParametersAtomically_Request>::value &&
    has_bounded_size<rcl_interfaces::srv::SetParametersAtomically_Response>::value
  >
{
};

template<>
struct is_service<rcl_interfaces::srv::SetParametersAtomically>
  : std::true_type
{
};

template<>
struct is_service_request<rcl_interfaces::srv::SetParametersAtomically_Request>
  : std::true_type
{
};

template<>
struct is_service_response<rcl_interfaces::srv::SetParametersAtomically_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // RCL_INTERFACES__SRV__DETAIL__SET_PARAMETERS_ATOMICALLY__TRAITS_HPP_
