// generated from rosidl_generator_py/resource/_idl_pkg_typesupport_entry_point.c.em
// generated code does not contain a copyright notice
#include <Python.h>

static PyMethodDef uwrt_mars_rover_xbox_controller__methods[] = {
  {NULL, NULL, 0, NULL}  /* sentinel */
};

static struct PyModuleDef uwrt_mars_rover_xbox_controller__module = {
  PyModuleDef_HEAD_INIT,
  "_uwrt_mars_rover_xbox_controller_support",
  "_uwrt_mars_rover_xbox_controller_doc",
  -1,  /* -1 means that the module keeps state in global variables */
  uwrt_mars_rover_xbox_controller__methods,
  NULL,
  NULL,
  NULL,
  NULL,
};

#include <stdbool.h>
#include <stdint.h>
#include "rosidl_runtime_c/visibility_control.h"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_runtime_c/service_type_support_struct.h"
#include "rosidl_runtime_c/action_type_support_struct.h"
#include "uwrt_mars_rover_xbox_controller/msg/detail/xbox_controller__type_support.h"
#include "uwrt_mars_rover_xbox_controller/msg/detail/xbox_controller__struct.h"
#include "uwrt_mars_rover_xbox_controller/msg/detail/xbox_controller__functions.h"

static void * uwrt_mars_rover_xbox_controller__msg__xbox_controller__create_ros_message(void)
{
  return uwrt_mars_rover_xbox_controller__msg__XboxController__create();
}

static void uwrt_mars_rover_xbox_controller__msg__xbox_controller__destroy_ros_message(void * raw_ros_message)
{
  uwrt_mars_rover_xbox_controller__msg__XboxController * ros_message = (uwrt_mars_rover_xbox_controller__msg__XboxController *)raw_ros_message;
  uwrt_mars_rover_xbox_controller__msg__XboxController__destroy(ros_message);
}

ROSIDL_GENERATOR_C_IMPORT
bool uwrt_mars_rover_xbox_controller__msg__xbox_controller__convert_from_py(PyObject * _pymsg, void * ros_message);
ROSIDL_GENERATOR_C_IMPORT
PyObject * uwrt_mars_rover_xbox_controller__msg__xbox_controller__convert_to_py(void * raw_ros_message);


ROSIDL_GENERATOR_C_IMPORT
const rosidl_message_type_support_t *
ROSIDL_GET_MSG_TYPE_SUPPORT(uwrt_mars_rover_xbox_controller, msg, XboxController);

int8_t
_register_msg_type__msg__xbox_controller(PyObject * pymodule)
{
  int8_t err;

  PyObject * pyobject_create_ros_message = NULL;
  pyobject_create_ros_message = PyCapsule_New(
    (void *)&uwrt_mars_rover_xbox_controller__msg__xbox_controller__create_ros_message,
    NULL, NULL);
  if (!pyobject_create_ros_message) {
    // previously added objects will be removed when the module is destroyed
    return -1;
  }
  err = PyModule_AddObject(
    pymodule,
    "create_ros_message_msg__msg__xbox_controller",
    pyobject_create_ros_message);
  if (err) {
    // the created capsule needs to be decremented
    Py_XDECREF(pyobject_create_ros_message);
    // previously added objects will be removed when the module is destroyed
    return err;
  }

  PyObject * pyobject_destroy_ros_message = NULL;
  pyobject_destroy_ros_message = PyCapsule_New(
    (void *)&uwrt_mars_rover_xbox_controller__msg__xbox_controller__destroy_ros_message,
    NULL, NULL);
  if (!pyobject_destroy_ros_message) {
    // previously added objects will be removed when the module is destroyed
    return -1;
  }
  err = PyModule_AddObject(
    pymodule,
    "destroy_ros_message_msg__msg__xbox_controller",
    pyobject_destroy_ros_message);
  if (err) {
    // the created capsule needs to be decremented
    Py_XDECREF(pyobject_destroy_ros_message);
    // previously added objects will be removed when the module is destroyed
    return err;
  }

  PyObject * pyobject_convert_from_py = NULL;
  pyobject_convert_from_py = PyCapsule_New(
    (void *)&uwrt_mars_rover_xbox_controller__msg__xbox_controller__convert_from_py,
    NULL, NULL);
  if (!pyobject_convert_from_py) {
    // previously added objects will be removed when the module is destroyed
    return -1;
  }
  err = PyModule_AddObject(
    pymodule,
    "convert_from_py_msg__msg__xbox_controller",
    pyobject_convert_from_py);
  if (err) {
    // the created capsule needs to be decremented
    Py_XDECREF(pyobject_convert_from_py);
    // previously added objects will be removed when the module is destroyed
    return err;
  }

  PyObject * pyobject_convert_to_py = NULL;
  pyobject_convert_to_py = PyCapsule_New(
    (void *)&uwrt_mars_rover_xbox_controller__msg__xbox_controller__convert_to_py,
    NULL, NULL);
  if (!pyobject_convert_to_py) {
    // previously added objects will be removed when the module is destroyed
    return -1;
  }
  err = PyModule_AddObject(
    pymodule,
    "convert_to_py_msg__msg__xbox_controller",
    pyobject_convert_to_py);
  if (err) {
    // the created capsule needs to be decremented
    Py_XDECREF(pyobject_convert_to_py);
    // previously added objects will be removed when the module is destroyed
    return err;
  }

  PyObject * pyobject_type_support = NULL;
  pyobject_type_support = PyCapsule_New(
    (void *)ROSIDL_GET_MSG_TYPE_SUPPORT(uwrt_mars_rover_xbox_controller, msg, XboxController),
    NULL, NULL);
  if (!pyobject_type_support) {
    // previously added objects will be removed when the module is destroyed
    return -1;
  }
  err = PyModule_AddObject(
    pymodule,
    "type_support_msg__msg__xbox_controller",
    pyobject_type_support);
  if (err) {
    // the created capsule needs to be decremented
    Py_XDECREF(pyobject_type_support);
    // previously added objects will be removed when the module is destroyed
    return err;
  }
  return 0;
}

PyMODINIT_FUNC
PyInit_uwrt_mars_rover_xbox_controller_s__rosidl_typesupport_introspection_c(void)
{
  PyObject * pymodule = NULL;
  pymodule = PyModule_Create(&uwrt_mars_rover_xbox_controller__module);
  if (!pymodule) {
    return NULL;
  }
  int8_t err;

  err = _register_msg_type__msg__xbox_controller(pymodule);
  if (err) {
    Py_XDECREF(pymodule);
    return NULL;
  }

  return pymodule;
}
