// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from uwrt_mars_rover_xbox_controller:msg/XboxController.idl
// generated code does not contain a copyright notice
#include "uwrt_mars_rover_xbox_controller/msg/detail/xbox_controller__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>


bool
uwrt_mars_rover_xbox_controller__msg__XboxController__init(uwrt_mars_rover_xbox_controller__msg__XboxController * msg)
{
  if (!msg) {
    return false;
  }
  // drivetrain_joy_x
  // drivetrain_joy_y
  // gimble_joy_x
  // gimble_joy_y
  // lt
  // rt
  return true;
}

void
uwrt_mars_rover_xbox_controller__msg__XboxController__fini(uwrt_mars_rover_xbox_controller__msg__XboxController * msg)
{
  if (!msg) {
    return;
  }
  // drivetrain_joy_x
  // drivetrain_joy_y
  // gimble_joy_x
  // gimble_joy_y
  // lt
  // rt
}

bool
uwrt_mars_rover_xbox_controller__msg__XboxController__are_equal(const uwrt_mars_rover_xbox_controller__msg__XboxController * lhs, const uwrt_mars_rover_xbox_controller__msg__XboxController * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // drivetrain_joy_x
  if (lhs->drivetrain_joy_x != rhs->drivetrain_joy_x) {
    return false;
  }
  // drivetrain_joy_y
  if (lhs->drivetrain_joy_y != rhs->drivetrain_joy_y) {
    return false;
  }
  // gimble_joy_x
  if (lhs->gimble_joy_x != rhs->gimble_joy_x) {
    return false;
  }
  // gimble_joy_y
  if (lhs->gimble_joy_y != rhs->gimble_joy_y) {
    return false;
  }
  // lt
  if (lhs->lt != rhs->lt) {
    return false;
  }
  // rt
  if (lhs->rt != rhs->rt) {
    return false;
  }
  return true;
}

bool
uwrt_mars_rover_xbox_controller__msg__XboxController__copy(
  const uwrt_mars_rover_xbox_controller__msg__XboxController * input,
  uwrt_mars_rover_xbox_controller__msg__XboxController * output)
{
  if (!input || !output) {
    return false;
  }
  // drivetrain_joy_x
  output->drivetrain_joy_x = input->drivetrain_joy_x;
  // drivetrain_joy_y
  output->drivetrain_joy_y = input->drivetrain_joy_y;
  // gimble_joy_x
  output->gimble_joy_x = input->gimble_joy_x;
  // gimble_joy_y
  output->gimble_joy_y = input->gimble_joy_y;
  // lt
  output->lt = input->lt;
  // rt
  output->rt = input->rt;
  return true;
}

uwrt_mars_rover_xbox_controller__msg__XboxController *
uwrt_mars_rover_xbox_controller__msg__XboxController__create()
{
  uwrt_mars_rover_xbox_controller__msg__XboxController * msg = (uwrt_mars_rover_xbox_controller__msg__XboxController *)malloc(sizeof(uwrt_mars_rover_xbox_controller__msg__XboxController));
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(uwrt_mars_rover_xbox_controller__msg__XboxController));
  bool success = uwrt_mars_rover_xbox_controller__msg__XboxController__init(msg);
  if (!success) {
    free(msg);
    return NULL;
  }
  return msg;
}

void
uwrt_mars_rover_xbox_controller__msg__XboxController__destroy(uwrt_mars_rover_xbox_controller__msg__XboxController * msg)
{
  if (msg) {
    uwrt_mars_rover_xbox_controller__msg__XboxController__fini(msg);
  }
  free(msg);
}


bool
uwrt_mars_rover_xbox_controller__msg__XboxController__Sequence__init(uwrt_mars_rover_xbox_controller__msg__XboxController__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  uwrt_mars_rover_xbox_controller__msg__XboxController * data = NULL;
  if (size) {
    data = (uwrt_mars_rover_xbox_controller__msg__XboxController *)calloc(size, sizeof(uwrt_mars_rover_xbox_controller__msg__XboxController));
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = uwrt_mars_rover_xbox_controller__msg__XboxController__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        uwrt_mars_rover_xbox_controller__msg__XboxController__fini(&data[i - 1]);
      }
      free(data);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
uwrt_mars_rover_xbox_controller__msg__XboxController__Sequence__fini(uwrt_mars_rover_xbox_controller__msg__XboxController__Sequence * array)
{
  if (!array) {
    return;
  }
  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      uwrt_mars_rover_xbox_controller__msg__XboxController__fini(&array->data[i]);
    }
    free(array->data);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

uwrt_mars_rover_xbox_controller__msg__XboxController__Sequence *
uwrt_mars_rover_xbox_controller__msg__XboxController__Sequence__create(size_t size)
{
  uwrt_mars_rover_xbox_controller__msg__XboxController__Sequence * array = (uwrt_mars_rover_xbox_controller__msg__XboxController__Sequence *)malloc(sizeof(uwrt_mars_rover_xbox_controller__msg__XboxController__Sequence));
  if (!array) {
    return NULL;
  }
  bool success = uwrt_mars_rover_xbox_controller__msg__XboxController__Sequence__init(array, size);
  if (!success) {
    free(array);
    return NULL;
  }
  return array;
}

void
uwrt_mars_rover_xbox_controller__msg__XboxController__Sequence__destroy(uwrt_mars_rover_xbox_controller__msg__XboxController__Sequence * array)
{
  if (array) {
    uwrt_mars_rover_xbox_controller__msg__XboxController__Sequence__fini(array);
  }
  free(array);
}

bool
uwrt_mars_rover_xbox_controller__msg__XboxController__Sequence__are_equal(const uwrt_mars_rover_xbox_controller__msg__XboxController__Sequence * lhs, const uwrt_mars_rover_xbox_controller__msg__XboxController__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!uwrt_mars_rover_xbox_controller__msg__XboxController__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
uwrt_mars_rover_xbox_controller__msg__XboxController__Sequence__copy(
  const uwrt_mars_rover_xbox_controller__msg__XboxController__Sequence * input,
  uwrt_mars_rover_xbox_controller__msg__XboxController__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(uwrt_mars_rover_xbox_controller__msg__XboxController);
    uwrt_mars_rover_xbox_controller__msg__XboxController * data =
      (uwrt_mars_rover_xbox_controller__msg__XboxController *)realloc(output->data, allocation_size);
    if (!data) {
      return false;
    }
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!uwrt_mars_rover_xbox_controller__msg__XboxController__init(&data[i])) {
        /* free currently allocated and return false */
        for (; i-- > output->capacity; ) {
          uwrt_mars_rover_xbox_controller__msg__XboxController__fini(&data[i]);
        }
        free(data);
        return false;
      }
    }
    output->data = data;
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!uwrt_mars_rover_xbox_controller__msg__XboxController__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
