// generated from rosidl_generator_py/resource/_idl_support.c.em
// with input from rover_msgs:msg/ControllerMsg.idl
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
#include "rover_msgs/msg/detail/controller_msg__struct.h"
#include "rover_msgs/msg/detail/controller_msg__functions.h"


ROSIDL_GENERATOR_C_EXPORT
bool rover_msgs__msg__controller_msg__convert_from_py(PyObject * _pymsg, void * _ros_message)
{
  // check that the passed message is of the expected Python class
  {
    char full_classname_dest[45];
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
    assert(strncmp("rover_msgs.msg._controller_msg.ControllerMsg", full_classname_dest, 44) == 0);
  }
  rover_msgs__msg__ControllerMsg * ros_message = _ros_message;
  {  // x
    PyObject * field = PyObject_GetAttrString(_pymsg, "x");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->x = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // y
    PyObject * field = PyObject_GetAttrString(_pymsg, "y");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->y = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // throttle
    PyObject * field = PyObject_GetAttrString(_pymsg, "throttle");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->throttle = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // camerax
    PyObject * field = PyObject_GetAttrString(_pymsg, "camerax");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->camerax = (int32_t)PyLong_AsLong(field);
    Py_DECREF(field);
  }
  {  // cameray
    PyObject * field = PyObject_GetAttrString(_pymsg, "cameray");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->cameray = (int32_t)PyLong_AsLong(field);
    Py_DECREF(field);
  }
  {  // light
    PyObject * field = PyObject_GetAttrString(_pymsg, "light");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->light = (int32_t)PyLong_AsLong(field);
    Py_DECREF(field);
  }
  {  // gear
    PyObject * field = PyObject_GetAttrString(_pymsg, "gear");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->gear = (int8_t)PyLong_AsLong(field);
    Py_DECREF(field);
  }

  return true;
}

ROSIDL_GENERATOR_C_EXPORT
PyObject * rover_msgs__msg__controller_msg__convert_to_py(void * raw_ros_message)
{
  /* NOTE(esteve): Call constructor of ControllerMsg */
  PyObject * _pymessage = NULL;
  {
    PyObject * pymessage_module = PyImport_ImportModule("rover_msgs.msg._controller_msg");
    assert(pymessage_module);
    PyObject * pymessage_class = PyObject_GetAttrString(pymessage_module, "ControllerMsg");
    assert(pymessage_class);
    Py_DECREF(pymessage_module);
    _pymessage = PyObject_CallObject(pymessage_class, NULL);
    Py_DECREF(pymessage_class);
    if (!_pymessage) {
      return NULL;
    }
  }
  rover_msgs__msg__ControllerMsg * ros_message = (rover_msgs__msg__ControllerMsg *)raw_ros_message;
  {  // x
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->x);
    {
      int rc = PyObject_SetAttrString(_pymessage, "x", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // y
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->y);
    {
      int rc = PyObject_SetAttrString(_pymessage, "y", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // throttle
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->throttle);
    {
      int rc = PyObject_SetAttrString(_pymessage, "throttle", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // camerax
    PyObject * field = NULL;
    field = PyLong_FromLong(ros_message->camerax);
    {
      int rc = PyObject_SetAttrString(_pymessage, "camerax", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // cameray
    PyObject * field = NULL;
    field = PyLong_FromLong(ros_message->cameray);
    {
      int rc = PyObject_SetAttrString(_pymessage, "cameray", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // light
    PyObject * field = NULL;
    field = PyLong_FromLong(ros_message->light);
    {
      int rc = PyObject_SetAttrString(_pymessage, "light", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // gear
    PyObject * field = NULL;
    field = PyLong_FromLong(ros_message->gear);
    {
      int rc = PyObject_SetAttrString(_pymessage, "gear", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }

  // ownership of _pymessage is transferred to the caller
  return _pymessage;
}
