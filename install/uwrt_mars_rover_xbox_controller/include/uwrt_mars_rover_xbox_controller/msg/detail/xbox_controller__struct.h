// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from uwrt_mars_rover_xbox_controller:msg/XboxController.idl
// generated code does not contain a copyright notice

#ifndef UWRT_MARS_ROVER_XBOX_CONTROLLER__MSG__DETAIL__XBOX_CONTROLLER__STRUCT_H_
#define UWRT_MARS_ROVER_XBOX_CONTROLLER__MSG__DETAIL__XBOX_CONTROLLER__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Struct defined in msg/XboxController in the package uwrt_mars_rover_xbox_controller.
typedef struct uwrt_mars_rover_xbox_controller__msg__XboxController
{
  float drivetrain_joy_x;
  float drivetrain_joy_y;
  float gimble_joy_x;
  float gimble_joy_y;
  float lt;
  float rt;
} uwrt_mars_rover_xbox_controller__msg__XboxController;

// Struct for a sequence of uwrt_mars_rover_xbox_controller__msg__XboxController.
typedef struct uwrt_mars_rover_xbox_controller__msg__XboxController__Sequence
{
  uwrt_mars_rover_xbox_controller__msg__XboxController * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} uwrt_mars_rover_xbox_controller__msg__XboxController__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // UWRT_MARS_ROVER_XBOX_CONTROLLER__MSG__DETAIL__XBOX_CONTROLLER__STRUCT_H_
