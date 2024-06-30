// generated from rosidl_typesupport_introspection_cpp/resource/idl__type_support.cpp.em
// with input from uwrt_mars_rover_xbox_controller:msg/XboxController.idl
// generated code does not contain a copyright notice

#include "array"
#include "cstddef"
#include "string"
#include "vector"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_interface/macros.h"
#include "uwrt_mars_rover_xbox_controller/msg/detail/xbox_controller__struct.hpp"
#include "rosidl_typesupport_introspection_cpp/field_types.hpp"
#include "rosidl_typesupport_introspection_cpp/identifier.hpp"
#include "rosidl_typesupport_introspection_cpp/message_introspection.hpp"
#include "rosidl_typesupport_introspection_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_introspection_cpp/visibility_control.h"

namespace uwrt_mars_rover_xbox_controller
{

namespace msg
{

namespace rosidl_typesupport_introspection_cpp
{

void XboxController_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) uwrt_mars_rover_xbox_controller::msg::XboxController(_init);
}

void XboxController_fini_function(void * message_memory)
{
  auto typed_message = static_cast<uwrt_mars_rover_xbox_controller::msg::XboxController *>(message_memory);
  typed_message->~XboxController();
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember XboxController_message_member_array[6] = {
  {
    "drivetrain_joy_x",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(uwrt_mars_rover_xbox_controller::msg::XboxController, drivetrain_joy_x),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "drivetrain_joy_y",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(uwrt_mars_rover_xbox_controller::msg::XboxController, drivetrain_joy_y),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "gimble_joy_x",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(uwrt_mars_rover_xbox_controller::msg::XboxController, gimble_joy_x),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "gimble_joy_y",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(uwrt_mars_rover_xbox_controller::msg::XboxController, gimble_joy_y),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "lt",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(uwrt_mars_rover_xbox_controller::msg::XboxController, lt),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "rt",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(uwrt_mars_rover_xbox_controller::msg::XboxController, rt),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers XboxController_message_members = {
  "uwrt_mars_rover_xbox_controller::msg",  // message namespace
  "XboxController",  // message name
  6,  // number of fields
  sizeof(uwrt_mars_rover_xbox_controller::msg::XboxController),
  XboxController_message_member_array,  // message members
  XboxController_init_function,  // function to initialize message memory (memory has to be allocated)
  XboxController_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t XboxController_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &XboxController_message_members,
  get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace msg

}  // namespace uwrt_mars_rover_xbox_controller


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<uwrt_mars_rover_xbox_controller::msg::XboxController>()
{
  return &::uwrt_mars_rover_xbox_controller::msg::rosidl_typesupport_introspection_cpp::XboxController_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, uwrt_mars_rover_xbox_controller, msg, XboxController)() {
  return &::uwrt_mars_rover_xbox_controller::msg::rosidl_typesupport_introspection_cpp::XboxController_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif
