// generated from rosidl_generator_py/resource/_idl_support.c.em
// with input from uwrt_mars_rover_xbox_controller:msg/XboxController.idl
// generated code does not contain a copyright notice
#define NPY_NO_DEPRECATED_API NPY_1_7_API_VERSION
#include <Python.h>
#include <stdbool.h>
#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-function"
#endif
#include "numpy/ndarrayobject.h"
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif
#include "rosidl_runtime_c/visibility_control.h"
#include "uwrt_mars_rover_xbox_controller/msg/detail/xbox_controller__struct.h"
#include "uwrt_mars_rover_xbox_controller/msg/detail/xbox_controller__functions.h"


ROSIDL_GENERATOR_C_EXPORT
bool uwrt_mars_rover_xbox_controller__msg__xbox_controller__convert_from_py(PyObject * _pymsg, void * _ros_message)
{
  // check that the passed message is of the expected Python class
  {
    char full_classname_dest[68];
    {
      char * class_name = NULL;
      char * module_name = NULL;
      {
        PyObject * class_attr = PyObject_GetAttrString(_pymsg, "__class__");
        if (class_attr) {
          PyObject * name_attr = PyObject_GetAttrString(class_attr, "__name__");
          if (name_attr) {
            class_name = (char *)PyUnicode_1BYTE_DATA(name_attr);
            Py_DECREF(name_attr);
          }
          PyObject * module_attr = PyObject_GetAttrString(class_attr, "__module__");
          if (module_attr) {
            module_name = (char *)PyUnicode_1BYTE_DATA(module_attr);
            Py_DECREF(module_attr);
          }
          Py_DECREF(class_attr);
        }
      }
      if (!class_name || !module_name) {
        return false;
      }
      snprintf(full_classname_dest, sizeof(full_classname_dest), "%s.%s", module_name, class_name);
    }
    assert(strncmp("uwrt_mars_rover_xbox_controller.msg._xbox_controller.XboxController", full_classname_dest, 67) == 0);
  }
  uwrt_mars_rover_xbox_controller__msg__XboxController * ros_message = _ros_message;
  {  // drivetrain_joy_x
    PyObject * field = PyObject_GetAttrString(_pymsg, "drivetrain_joy_x");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->drivetrain_joy_x = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // drivetrain_joy_y
    PyObject * field = PyObject_GetAttrString(_pymsg, "drivetrain_joy_y");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->drivetrain_joy_y = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // gimble_joy_x
    PyObject * field = PyObject_GetAttrString(_pymsg, "gimble_joy_x");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->gimble_joy_x = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // gimble_joy_y
    PyObject * field = PyObject_GetAttrString(_pymsg, "gimble_joy_y");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->gimble_joy_y = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // lt
    PyObject * field = PyObject_GetAttrString(_pymsg, "lt");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->lt = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // rt
    PyObject * field = PyObject_GetAttrString(_pymsg, "rt");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->rt = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }

  return true;
}

ROSIDL_GENERATOR_C_EXPORT
PyObject * uwrt_mars_rover_xbox_controller__msg__xbox_controller__convert_to_py(void * raw_ros_message)
{
  /* NOTE(esteve): Call constructor of XboxController */
  PyObject * _pymessage = NULL;
  {
    PyObject * pymessage_module = PyImport_ImportModule("uwrt_mars_rover_xbox_controller.msg._xbox_controller");
    assert(pymessage_module);
    PyObject * pymessage_class = PyObject_GetAttrString(pymessage_module, "XboxController");
    assert(pymessage_class);
    Py_DECREF(pymessage_module);
    _pymessage = PyObject_CallObject(pymessage_class, NULL);
    Py_DECREF(pymessage_class);
    if (!_pymessage) {
      return NULL;
    }
  }
  uwrt_mars_rover_xbox_controller__msg__XboxController * ros_message = (uwrt_mars_rover_xbox_controller__msg__XboxController *)raw_ros_message;
  {  // drivetrain_joy_x
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->drivetrain_joy_x);
    {
      int rc = PyObject_SetAttrString(_pymessage, "drivetrain_joy_x", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // drivetrain_joy_y
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->drivetrain_joy_y);
    {
      int rc = PyObject_SetAttrString(_pymessage, "drivetrain_joy_y", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // gimble_joy_x
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->gimble_joy_x);
    {
      int rc = PyObject_SetAttrString(_pymessage, "gimble_joy_x", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // gimble_joy_y
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->gimble_joy_y);
    {
      int rc = PyObject_SetAttrString(_pymessage, "gimble_joy_y", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // lt
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->lt);
    {
      int rc = PyObject_SetAttrString(_pymessage, "lt", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // rt
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->rt);
    {
      int rc = PyObject_SetAttrString(_pymessage, "rt", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }

  // ownership of _pymessage is transferred to the caller
  return _pymessage;
}
