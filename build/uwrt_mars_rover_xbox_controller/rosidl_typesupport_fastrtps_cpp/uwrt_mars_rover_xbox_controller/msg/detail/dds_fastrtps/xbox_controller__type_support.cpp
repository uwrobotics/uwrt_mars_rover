// generated from rosidl_typesupport_fastrtps_cpp/resource/idl__type_support.cpp.em
// with input from uwrt_mars_rover_xbox_controller:msg/XboxController.idl
// generated code does not contain a copyright notice
#include "uwrt_mars_rover_xbox_controller/msg/detail/xbox_controller__rosidl_typesupport_fastrtps_cpp.hpp"
#include "uwrt_mars_rover_xbox_controller/msg/detail/xbox_controller__struct.hpp"

#include <limits>
#include <stdexcept>
#include <string>
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_fastrtps_cpp/identifier.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_fastrtps_cpp/wstring_conversion.hpp"
#include "fastcdr/Cdr.h"


// forward declaration of message dependencies and their conversion functions

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
  eprosima::fastcdr::Cdr & cdr)
{
  // Member: drivetrain_joy_x
  cdr << ros_message.drivetrain_joy_x;
  // Member: drivetrain_joy_y
  cdr << ros_message.drivetrain_joy_y;
  // Member: gimble_joy_x
  cdr << ros_message.gimble_joy_x;
  // Member: gimble_joy_y
  cdr << ros_message.gimble_joy_y;
  // Member: lt
  cdr << ros_message.lt;
  // Member: rt
  cdr << ros_message.rt;
  return true;
}

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_uwrt_mars_rover_xbox_controller
cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  uwrt_mars_rover_xbox_controller::msg::XboxController & ros_message)
{
  // Member: drivetrain_joy_x
  cdr >> ros_message.drivetrain_joy_x;

  // Member: drivetrain_joy_y
  cdr >> ros_message.drivetrain_joy_y;

  // Member: gimble_joy_x
  cdr >> ros_message.gimble_joy_x;

  // Member: gimble_joy_y
  cdr >> ros_message.gimble_joy_y;

  // Member: lt
  cdr >> ros_message.lt;

  // Member: rt
  cdr >> ros_message.rt;

  return true;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_uwrt_mars_rover_xbox_controller
get_serialized_size(
  const uwrt_mars_rover_xbox_controller::msg::XboxController & ros_message,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // Member: drivetrain_joy_x
  {
    size_t item_size = sizeof(ros_message.drivetrain_joy_x);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: drivetrain_joy_y
  {
    size_t item_size = sizeof(ros_message.drivetrain_joy_y);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: gimble_joy_x
  {
    size_t item_size = sizeof(ros_message.gimble_joy_x);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: gimble_joy_y
  {
    size_t item_size = sizeof(ros_message.gimble_joy_y);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: lt
  {
    size_t item_size = sizeof(ros_message.lt);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: rt
  {
    size_t item_size = sizeof(ros_message.rt);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_uwrt_mars_rover_xbox_controller
max_serialized_size_XboxController(
  bool & full_bounded,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;
  (void)full_bounded;


  // Member: drivetrain_joy_x
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: drivetrain_joy_y
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: gimble_joy_x
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: gimble_joy_y
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: lt
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: rt
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  return current_alignment - initial_alignment;
}

static bool _XboxController__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  auto typed_message =
    static_cast<const uwrt_mars_rover_xbox_controller::msg::XboxController *>(
    untyped_ros_message);
  return cdr_serialize(*typed_message, cdr);
}

static bool _XboxController__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  auto typed_message =
    static_cast<uwrt_mars_rover_xbox_controller::msg::XboxController *>(
    untyped_ros_message);
  return cdr_deserialize(cdr, *typed_message);
}

static uint32_t _XboxController__get_serialized_size(
  const void * untyped_ros_message)
{
  auto typed_message =
    static_cast<const uwrt_mars_rover_xbox_controller::msg::XboxController *>(
    untyped_ros_message);
  return static_cast<uint32_t>(get_serialized_size(*typed_message, 0));
}

static size_t _XboxController__max_serialized_size(bool & full_bounded)
{
  return max_serialized_size_XboxController(full_bounded, 0);
}

static message_type_support_callbacks_t _XboxController__callbacks = {
  "uwrt_mars_rover_xbox_controller::msg",
  "XboxController",
  _XboxController__cdr_serialize,
  _XboxController__cdr_deserialize,
  _XboxController__get_serialized_size,
  _XboxController__max_serialized_size
};

static rosidl_message_type_support_t _XboxController__handle = {
  rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
  &_XboxController__callbacks,
  get_message_typesupport_handle_function,
};

}  // namespace typesupport_fastrtps_cpp

}  // namespace msg

}  // namespace uwrt_mars_rover_xbox_controller

namespace rosidl_typesupport_fastrtps_cpp
{

template<>
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_EXPORT_uwrt_mars_rover_xbox_controller
const rosidl_message_type_support_t *
get_message_type_support_handle<uwrt_mars_rover_xbox_controller::msg::XboxController>()
{
  return &uwrt_mars_rover_xbox_controller::msg::typesupport_fastrtps_cpp::_XboxController__handle;
}

}  // namespace rosidl_typesupport_fastrtps_cpp

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, uwrt_mars_rover_xbox_controller, msg, XboxController)() {
  return &uwrt_mars_rover_xbox_controller::msg::typesupport_fastrtps_cpp::_XboxController__handle;
}

#ifdef __cplusplus
}
#endif
