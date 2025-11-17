// generated from rosidl_generator_py/resource/_idl_support.c.em
// with input from pointcloud_filters:msg/BuoyDetection.idl
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
#include "pointcloud_filters/msg/detail/buoy_detection__struct.h"
#include "pointcloud_filters/msg/detail/buoy_detection__functions.h"


ROSIDL_GENERATOR_C_EXPORT
bool pointcloud_filters__msg__buoy_detection__convert_from_py(PyObject * _pymsg, void * _ros_message)
{
  // check that the passed message is of the expected Python class
  {
    char full_classname_dest[53];
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
    assert(strncmp("pointcloud_filters.msg._buoy_detection.BuoyDetection", full_classname_dest, 52) == 0);
  }
  pointcloud_filters__msg__BuoyDetection * ros_message = _ros_message;
  {  // range
    PyObject * field = PyObject_GetAttrString(_pymsg, "range");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->range = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // bearing
    PyObject * field = PyObject_GetAttrString(_pymsg, "bearing");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->bearing = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // z_mean
    PyObject * field = PyObject_GetAttrString(_pymsg, "z_mean");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->z_mean = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // num_points
    PyObject * field = PyObject_GetAttrString(_pymsg, "num_points");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->num_points = PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }
  {  // lateral_extent
    PyObject * field = PyObject_GetAttrString(_pymsg, "lateral_extent");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->lateral_extent = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // radial_extent
    PyObject * field = PyObject_GetAttrString(_pymsg, "radial_extent");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->radial_extent = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // confidence
    PyObject * field = PyObject_GetAttrString(_pymsg, "confidence");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->confidence = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }

  return true;
}

ROSIDL_GENERATOR_C_EXPORT
PyObject * pointcloud_filters__msg__buoy_detection__convert_to_py(void * raw_ros_message)
{
  /* NOTE(esteve): Call constructor of BuoyDetection */
  PyObject * _pymessage = NULL;
  {
    PyObject * pymessage_module = PyImport_ImportModule("pointcloud_filters.msg._buoy_detection");
    assert(pymessage_module);
    PyObject * pymessage_class = PyObject_GetAttrString(pymessage_module, "BuoyDetection");
    assert(pymessage_class);
    Py_DECREF(pymessage_module);
    _pymessage = PyObject_CallObject(pymessage_class, NULL);
    Py_DECREF(pymessage_class);
    if (!_pymessage) {
      return NULL;
    }
  }
  pointcloud_filters__msg__BuoyDetection * ros_message = (pointcloud_filters__msg__BuoyDetection *)raw_ros_message;
  {  // range
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->range);
    {
      int rc = PyObject_SetAttrString(_pymessage, "range", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // bearing
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->bearing);
    {
      int rc = PyObject_SetAttrString(_pymessage, "bearing", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // z_mean
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->z_mean);
    {
      int rc = PyObject_SetAttrString(_pymessage, "z_mean", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // num_points
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->num_points);
    {
      int rc = PyObject_SetAttrString(_pymessage, "num_points", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // lateral_extent
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->lateral_extent);
    {
      int rc = PyObject_SetAttrString(_pymessage, "lateral_extent", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // radial_extent
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->radial_extent);
    {
      int rc = PyObject_SetAttrString(_pymessage, "radial_extent", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // confidence
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->confidence);
    {
      int rc = PyObject_SetAttrString(_pymessage, "confidence", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }

  // ownership of _pymessage is transferred to the caller
  return _pymessage;
}
