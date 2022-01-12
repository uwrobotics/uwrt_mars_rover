// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from rcl_interfaces:srv/DescribeParameters.idl
// generated code does not contain a copyright notice

#ifndef RCL_INTERFACES__SRV__DETAIL__DESCRIBE_PARAMETERS__TRAITS_HPP_
#define RCL_INTERFACES__SRV__DETAIL__DESCRIBE_PARAMETERS__TRAITS_HPP_

#include "rcl_interfaces/srv/detail/describe_parameters__struct.hpp"
#include <stdint.h>
#include <rosidl_runtime_cpp/traits.hpp>
#include <sstream>
#include <string>
#include <type_traits>

namespace rosidl_generator_traits
{

inline void to_yaml(
  const rcl_interfaces::srv::DescribeParameters_Request & msg,
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

inline std::string to_yaml(const rcl_interfaces::srv::DescribeParameters_Request & msg)
{
  std::ostringstream out;
  to_yaml(msg, out);
  return out.str();
}

template<>
inline const char * data_type<rcl_interfaces::srv::DescribeParameters_Request>()
{
  return "rcl_interfaces::srv::DescribeParameters_Request";
}

template<>
inline const char * name<rcl_interfaces::srv::DescribeParameters_Request>()
{
  return "rcl_interfaces/srv/DescribeParameters_Request";
}

template<>
struct has_fixed_size<rcl_interfaces::srv::DescribeParameters_Request>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<rcl_interfaces::srv::DescribeParameters_Request>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<rcl_interfaces::srv::DescribeParameters_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

// Include directives for member types
// Member 'descriptors'
#include "rcl_interfaces/msg/detail/parameter_descriptor__traits.hpp"

namespace rosidl_generator_traits
{

inline void to_yaml(
  const rcl_interfaces::srv::DescribeParameters_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: descriptors
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.descriptors.size() == 0) {
      out << "descriptors: []\n";
    } else {
      out << "descriptors:\n";
      for (auto item : msg.descriptors) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "-\n";
        to_yaml(item, out, indentation + 2);
      }
    }
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const rcl_interfaces::srv::DescribeParameters_Response & msg)
{
  std::ostringstream out;
  to_yaml(msg, out);
  return out.str();
}

template<>
inline const char * data_type<rcl_interfaces::srv::DescribeParameters_Response>()
{
  return "rcl_interfaces::srv::DescribeParameters_Response";
}

template<>
inline const char * name<rcl_interfaces::srv::DescribeParameters_Response>()
{
  return "rcl_interfaces/srv/DescribeParameters_Response";
}

template<>
struct has_fixed_size<rcl_interfaces::srv::DescribeParameters_Response>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<rcl_interfaces::srv::DescribeParameters_Response>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<rcl_interfaces::srv::DescribeParameters_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<rcl_interfaces::srv::DescribeParameters>()
{
  return "rcl_interfaces::srv::DescribeParameters";
}

template<>
inline const char * name<rcl_interfaces::srv::DescribeParameters>()
{
  return "rcl_interfaces/srv/DescribeParameters";
}

template<>
struct has_fixed_size<rcl_interfaces::srv::DescribeParameters>
  : std::integral_constant<
    bool,
    has_fixed_size<rcl_interfaces::srv::DescribeParameters_Request>::value &&
    has_fixed_size<rcl_interfaces::srv::DescribeParameters_Response>::value
  >
{
};

template<>
struct has_bounded_size<rcl_interfaces::srv::DescribeParameters>
  : std::integral_constant<
    bool,
    has_bounded_size<rcl_interfaces::srv::DescribeParameters_Request>::value &&
    has_bounded_size<rcl_interfaces::srv::DescribeParameters_Response>::value
  >
{
};

template<>
struct is_service<rcl_interfaces::srv::DescribeParameters>
  : std::true_type
{
};

template<>
struct is_service_request<rcl_interfaces::srv::DescribeParameters_Request>
  : std::true_type
{
};

template<>
struct is_service_response<rcl_interfaces::srv::DescribeParameters_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // RCL_INTERFACES__SRV__DETAIL__DESCRIBE_PARAMETERS__TRAITS_HPP_
