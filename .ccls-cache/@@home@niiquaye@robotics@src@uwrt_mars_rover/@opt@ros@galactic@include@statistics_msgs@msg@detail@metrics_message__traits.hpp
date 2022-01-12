// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from statistics_msgs:msg/MetricsMessage.idl
// generated code does not contain a copyright notice

#ifndef STATISTICS_MSGS__MSG__DETAIL__METRICS_MESSAGE__TRAITS_HPP_
#define STATISTICS_MSGS__MSG__DETAIL__METRICS_MESSAGE__TRAITS_HPP_

#include "statistics_msgs/msg/detail/metrics_message__struct.hpp"
#include <stdint.h>
#include <rosidl_runtime_cpp/traits.hpp>
#include <sstream>
#include <string>
#include <type_traits>

// Include directives for member types
// Member 'window_start'
// Member 'window_stop'
#include "builtin_interfaces/msg/detail/time__traits.hpp"
// Member 'statistics'
#include "statistics_msgs/msg/detail/statistic_data_point__traits.hpp"

namespace rosidl_generator_traits
{

inline void to_yaml(
  const statistics_msgs::msg::MetricsMessage & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: measurement_source_name
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "measurement_source_name: ";
    value_to_yaml(msg.measurement_source_name, out);
    out << "\n";
  }

  // member: metrics_source
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "metrics_source: ";
    value_to_yaml(msg.metrics_source, out);
    out << "\n";
  }

  // member: unit
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "unit: ";
    value_to_yaml(msg.unit, out);
    out << "\n";
  }

  // member: window_start
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "window_start:\n";
    to_yaml(msg.window_start, out, indentation + 2);
  }

  // member: window_stop
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "window_stop:\n";
    to_yaml(msg.window_stop, out, indentation + 2);
  }

  // member: statistics
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.statistics.size() == 0) {
      out << "statistics: []\n";
    } else {
      out << "statistics:\n";
      for (auto item : msg.statistics) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "-\n";
        to_yaml(item, out, indentation + 2);
      }
    }
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const statistics_msgs::msg::MetricsMessage & msg)
{
  std::ostringstream out;
  to_yaml(msg, out);
  return out.str();
}

template<>
inline const char * data_type<statistics_msgs::msg::MetricsMessage>()
{
  return "statistics_msgs::msg::MetricsMessage";
}

template<>
inline const char * name<statistics_msgs::msg::MetricsMessage>()
{
  return "statistics_msgs/msg/MetricsMessage";
}

template<>
struct has_fixed_size<statistics_msgs::msg::MetricsMessage>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<statistics_msgs::msg::MetricsMessage>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<statistics_msgs::msg::MetricsMessage>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // STATISTICS_MSGS__MSG__DETAIL__METRICS_MESSAGE__TRAITS_HPP_
