// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from statistics_msgs:msg/StatisticDataPoint.idl
// generated code does not contain a copyright notice

#ifndef STATISTICS_MSGS__MSG__DETAIL__STATISTIC_DATA_POINT__TRAITS_HPP_
#define STATISTICS_MSGS__MSG__DETAIL__STATISTIC_DATA_POINT__TRAITS_HPP_

#include "statistics_msgs/msg/detail/statistic_data_point__struct.hpp"
#include <stdint.h>
#include <rosidl_runtime_cpp/traits.hpp>
#include <sstream>
#include <string>
#include <type_traits>

namespace rosidl_generator_traits
{

inline void to_yaml(
  const statistics_msgs::msg::StatisticDataPoint & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: data_type
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "data_type: ";
    value_to_yaml(msg.data_type, out);
    out << "\n";
  }

  // member: data
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "data: ";
    value_to_yaml(msg.data, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const statistics_msgs::msg::StatisticDataPoint & msg)
{
  std::ostringstream out;
  to_yaml(msg, out);
  return out.str();
}

template<>
inline const char * data_type<statistics_msgs::msg::StatisticDataPoint>()
{
  return "statistics_msgs::msg::StatisticDataPoint";
}

template<>
inline const char * name<statistics_msgs::msg::StatisticDataPoint>()
{
  return "statistics_msgs/msg/StatisticDataPoint";
}

template<>
struct has_fixed_size<statistics_msgs::msg::StatisticDataPoint>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<statistics_msgs::msg::StatisticDataPoint>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<statistics_msgs::msg::StatisticDataPoint>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // STATISTICS_MSGS__MSG__DETAIL__STATISTIC_DATA_POINT__TRAITS_HPP_
