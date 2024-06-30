// generated from rosidl_typesupport_fastrtps_cpp/resource/idl__rosidl_typesupport_fastrtps_cpp.hpp.em
// with input from uwrt_mars_rover_xbox_controller:msg/XboxController.idl
// generated code does not contain a copyright notice

#ifndef UWRT_MARS_ROVER_XBOX_CONTROLLER__MSG__DETAIL__XBOX_CONTROLLER__ROSIDL_TYPESUPPORT_FASTRTPS_CPP_HPP_
#define UWRT_MARS_ROVER_XBOX_CONTROLLER__MSG__DETAIL__XBOX_CONTROLLER__ROSIDL_TYPESUPPORT_FASTRTPS_CPP_HPP_

#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_interface/macros.h"
#include "uwrt_mars_rover_xbox_controller/msg/rosidl_typesupport_fastrtps_cpp__visibility_control.h"
#include "uwrt_mars_rover_xbox_controller/msg/detail/xbox_controller__struct.hpp"

#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-parameter"
# ifdef __clang__
#  pragma clang diagnostic ignored "-Wdeprecated-register"
#  pragma clang diagnostic ignored "-Wreturn-type-c-linkage"
# endif
#endif
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif

#include "fastcdr/Cdr.h"

namespace uwrt_mars_rover_xbox_controller
{

namespace msg
{

namespace typesupport_fastrtps_cpp
{

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_uwrt_mars_rover_xbox_controller
cdr_serialize(
  const uwrt_mars_rover_xbox_controller::msg::XboxController & ros_message,
  eprosima::fastcdr::Cdr & cdr);

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_uwrt_mars_rover_xbox_controller
cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  uwrt_mars_rover_xbox_controller::msg::XboxController & ros_message);

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_uwrt_mars_rover_xbox_controller
get_serialized_size(
  const uwrt_mars_rover_xbox_controller::msg::XboxController & ros_message,
  size_t current_alignment);

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_uwrt_mars_rover_xbox_controller
max_serialized_size_XboxController(
  bool & full_bounded,
  size_t current_alignment);

}  // namespace typesupport_fastrtps_cpp

}  // namespace msg

}  // namespace uwrt_mars_rover_xbox_controller

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_uwrt_mars_rover_xbox_controller
const rosidl_message_type_support_t *
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, uwrt_mars_rover_xbox_controller, msg, XboxController)();

#ifdef __cplusplus
}
#endif

#endif  // UWRT_MARS_ROVER_XBOX_CONTROLLER__MSG__DETAIL__XBOX_CONTROLLER__ROSIDL_TYPESUPPORT_FASTRTPS_CPP_HPP_
