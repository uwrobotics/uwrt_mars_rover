// generated from rosidl_typesupport_fastrtps_c/resource/idl__type_support_c.cpp.em
// with input from uwrt_mars_rover_xbox_controller:msg/XboxController.idl
// generated code does not contain a copyright notice
#include "uwrt_mars_rover_xbox_controller/msg/detail/xbox_controller__rosidl_typesupport_fastrtps_c.h"


#include <cassert>
#include <limits>
#include <string>
#include "rosidl_typesupport_fastrtps_c/identifier.h"
#include "rosidl_typesupport_fastrtps_c/wstring_conversion.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "uwrt_mars_rover_xbox_controller/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "uwrt_mars_rover_xbox_controller/msg/detail/xbox_controller__struct.h"
#include "uwrt_mars_rover_xbox_controller/msg/detail/xbox_controller__functions.h"
#include "fastcdr/Cdr.h"

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

// includes and forward declarations of message dependencies and their conversion functions

#if defined(__cplusplus)
extern "C"
{
#endif


// forward declare type support functions


using _XboxController__ros_msg_type = uwrt_mars_rover_xbox_controller__msg__XboxController;

static bool _XboxController__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  const _XboxController__ros_msg_type * ros_message = static_cast<const _XboxController__ros_msg_type *>(untyped_ros_message);
  // Field name: drivetrain_joy_x
  {
    cdr << ros_message->drivetrain_joy_x;
  }

  // Field name: drivetrain_joy_y
  {
    cdr << ros_message->drivetrain_joy_y;
  }

  // Field name: gimble_joy_x
  {
    cdr << ros_message->gimble_joy_x;
  }

  // Field name: gimble_joy_y
  {
    cdr << ros_message->gimble_joy_y;
  }

  // Field name: lt
  {
    cdr << ros_message->lt;
  }

  // Field name: rt
  {
    cdr << ros_message->rt;
  }

  return true;
}

static bool _XboxController__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  _XboxController__ros_msg_type * ros_message = static_cast<_XboxController__ros_msg_type *>(untyped_ros_message);
  // Field name: drivetrain_joy_x
  {
    cdr >> ros_message->drivetrain_joy_x;
  }

  // Field name: drivetrain_joy_y
  {
    cdr >> ros_message->drivetrain_joy_y;
  }

  // Field name: gimble_joy_x
  {
    cdr >> ros_message->gimble_joy_x;
  }

  // Field name: gimble_joy_y
  {
    cdr >> ros_message->gimble_joy_y;
  }

  // Field name: lt
  {
    cdr >> ros_message->lt;
  }

  // Field name: rt
  {
    cdr >> ros_message->rt;
  }

  return true;
}  // NOLINT(readability/fn_size)

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_uwrt_mars_rover_xbox_controller
size_t get_serialized_size_uwrt_mars_rover_xbox_controller__msg__XboxController(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _XboxController__ros_msg_type * ros_message = static_cast<const _XboxController__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // field.name drivetrain_joy_x
  {
    size_t item_size = sizeof(ros_message->drivetrain_joy_x);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name drivetrain_joy_y
  {
    size_t item_size = sizeof(ros_message->drivetrain_joy_y);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name gimble_joy_x
  {
    size_t item_size = sizeof(ros_message->gimble_joy_x);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name gimble_joy_y
  {
    size_t item_size = sizeof(ros_message->gimble_joy_y);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name lt
  {
    size_t item_size = sizeof(ros_message->lt);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name rt
  {
    size_t item_size = sizeof(ros_message->rt);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

static uint32_t _XboxController__get_serialized_size(const void * untyped_ros_message)
{
  return static_cast<uint32_t>(
    get_serialized_size_uwrt_mars_rover_xbox_controller__msg__XboxController(
      untyped_ros_message, 0));
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_uwrt_mars_rover_xbox_controller
size_t max_serialized_size_uwrt_mars_rover_xbox_controller__msg__XboxController(
  bool & full_bounded,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;
  (void)full_bounded;

  // member: drivetrain_joy_x
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: drivetrain_joy_y
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: gimble_joy_x
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: gimble_joy_y
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: lt
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: rt
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  return current_alignment - initial_alignment;
}

static size_t _XboxController__max_serialized_size(bool & full_bounded)
{
  return max_serialized_size_uwrt_mars_rover_xbox_controller__msg__XboxController(
    full_bounded, 0);
}


static message_type_support_callbacks_t __callbacks_XboxController = {
  "uwrt_mars_rover_xbox_controller::msg",
  "XboxController",
  _XboxController__cdr_serialize,
  _XboxController__cdr_deserialize,
  _XboxController__get_serialized_size,
  _XboxController__max_serialized_size
};

static rosidl_message_type_support_t _XboxController__type_support = {
  rosidl_typesupport_fastrtps_c__identifier,
  &__callbacks_XboxController,
  get_message_typesupport_handle_function,
};

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, uwrt_mars_rover_xbox_controller, msg, XboxController)() {
  return &_XboxController__type_support;
}

#if defined(__cplusplus)
}
#endif
