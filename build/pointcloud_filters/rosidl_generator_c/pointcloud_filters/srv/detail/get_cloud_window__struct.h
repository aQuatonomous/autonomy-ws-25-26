// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from pointcloud_filters:srv/GetCloudWindow.idl
// generated code does not contain a copyright notice

#ifndef POINTCLOUD_FILTERS__SRV__DETAIL__GET_CLOUD_WINDOW__STRUCT_H_
#define POINTCLOUD_FILTERS__SRV__DETAIL__GET_CLOUD_WINDOW__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in srv/GetCloudWindow in the package pointcloud_filters.
typedef struct pointcloud_filters__srv__GetCloudWindow_Request
{
  /// Duration of history to retrieve (0.0 to max_buffer_duration_sec)
  float window_sec;
  /// If true, return a single merged cloud; if false, return array of individual clouds
  bool merged;
} pointcloud_filters__srv__GetCloudWindow_Request;

// Struct for a sequence of pointcloud_filters__srv__GetCloudWindow_Request.
typedef struct pointcloud_filters__srv__GetCloudWindow_Request__Sequence
{
  pointcloud_filters__srv__GetCloudWindow_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} pointcloud_filters__srv__GetCloudWindow_Request__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'clouds'
// Member 'merged_cloud'
#include "sensor_msgs/msg/detail/point_cloud2__struct.h"

/// Struct defined in srv/GetCloudWindow in the package pointcloud_filters.
typedef struct pointcloud_filters__srv__GetCloudWindow_Response
{
  /// Array of individual clouds (if merged=false)
  sensor_msgs__msg__PointCloud2__Sequence clouds;
  /// Single concatenated cloud (if merged=true)
  sensor_msgs__msg__PointCloud2 merged_cloud;
} pointcloud_filters__srv__GetCloudWindow_Response;

// Struct for a sequence of pointcloud_filters__srv__GetCloudWindow_Response.
typedef struct pointcloud_filters__srv__GetCloudWindow_Response__Sequence
{
  pointcloud_filters__srv__GetCloudWindow_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} pointcloud_filters__srv__GetCloudWindow_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // POINTCLOUD_FILTERS__SRV__DETAIL__GET_CLOUD_WINDOW__STRUCT_H_
