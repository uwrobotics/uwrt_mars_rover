// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from uwrt_mars_rover_xbox_controller:msg/XboxController.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "uwrt_mars_rover_xbox_controller/msg/detail/xbox_controller__rosidl_typesupport_introspection_c.h"
#include "uwrt_mars_rover_xbox_controller/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "uwrt_mars_rover_xbox_controller/msg/detail/xbox_controller__functions.h"
#include "uwrt_mars_rover_xbox_controller/msg/detail/xbox_controller__struct.h"


#ifdef __cplusplus
extern "C"
{
#endif

void XboxController__rosidl_typesupport_introspection_c__XboxController_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  uwrt_mars_rover_xbox_controller__msg__XboxController__init(message_memory);
}

void XboxController__rosidl_typesupport_introspection_c__XboxController_fini_function(void * message_memory)
{
  uwrt_mars_rover_xbox_controller__msg__XboxController__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember XboxController__rosidl_typesupport_introspection_c__XboxController_message_member_array[6] = {
  {
    "drivetrain_joy_x",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(uwrt_mars_rover_xbox_controller__msg__XboxController, drivetrain_joy_x),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "drivetrain_joy_y",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(uwrt_mars_rover_xbox_controller__msg__XboxController, drivetrain_joy_y),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "gimble_joy_x",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(uwrt_mars_rover_xbox_controller__msg__XboxController, gimble_joy_x),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "gimble_joy_y",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(uwrt_mars_rover_xbox_controller__msg__XboxController, gimble_joy_y),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "lt",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(uwrt_mars_rover_xbox_controller__msg__XboxController, lt),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "rt",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(uwrt_mars_rover_xbox_controller__msg__XboxController, rt),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers XboxController__rosidl_typesupport_introspection_c__XboxController_message_members = {
  "uwrt_mars_rover_xbox_controller__msg",  // message namespace
  "XboxController",  // message name
  6,  // number of fields
  sizeof(uwrt_mars_rover_xbox_controller__msg__XboxController),
  XboxController__rosidl_typesupport_introspection_c__XboxController_message_member_array,  // message members
  XboxController__rosidl_typesupport_introspection_c__XboxController_init_function,  // function to initialize message memory (memory has to be allocated)
  XboxController__rosidl_typesupport_introspection_c__XboxController_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t XboxController__rosidl_typesupport_introspection_c__XboxController_message_type_support_handle = {
  0,
  &XboxController__rosidl_typesupport_introspection_c__XboxController_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_uwrt_mars_rover_xbox_controller
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, uwrt_mars_rover_xbox_controller, msg, XboxController)() {
  if (!XboxController__rosidl_typesupport_introspection_c__XboxController_message_type_support_handle.typesupport_identifier) {
    XboxController__rosidl_typesupport_introspection_c__XboxController_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &XboxController__rosidl_typesupport_introspection_c__XboxController_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
