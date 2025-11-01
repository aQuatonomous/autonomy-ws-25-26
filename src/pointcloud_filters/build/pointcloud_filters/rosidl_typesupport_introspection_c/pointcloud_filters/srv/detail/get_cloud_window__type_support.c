// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from pointcloud_filters:srv/GetCloudWindow.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "pointcloud_filters/srv/detail/get_cloud_window__rosidl_typesupport_introspection_c.h"
#include "pointcloud_filters/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "pointcloud_filters/srv/detail/get_cloud_window__functions.h"
#include "pointcloud_filters/srv/detail/get_cloud_window__struct.h"


#ifdef __cplusplus
extern "C"
{
#endif

void pointcloud_filters__srv__GetCloudWindow_Request__rosidl_typesupport_introspection_c__GetCloudWindow_Request_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  pointcloud_filters__srv__GetCloudWindow_Request__init(message_memory);
}

void pointcloud_filters__srv__GetCloudWindow_Request__rosidl_typesupport_introspection_c__GetCloudWindow_Request_fini_function(void * message_memory)
{
  pointcloud_filters__srv__GetCloudWindow_Request__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember pointcloud_filters__srv__GetCloudWindow_Request__rosidl_typesupport_introspection_c__GetCloudWindow_Request_message_member_array[2] = {
  {
    "window_sec",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(pointcloud_filters__srv__GetCloudWindow_Request, window_sec),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "merged",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(pointcloud_filters__srv__GetCloudWindow_Request, merged),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers pointcloud_filters__srv__GetCloudWindow_Request__rosidl_typesupport_introspection_c__GetCloudWindow_Request_message_members = {
  "pointcloud_filters__srv",  // message namespace
  "GetCloudWindow_Request",  // message name
  2,  // number of fields
  sizeof(pointcloud_filters__srv__GetCloudWindow_Request),
  pointcloud_filters__srv__GetCloudWindow_Request__rosidl_typesupport_introspection_c__GetCloudWindow_Request_message_member_array,  // message members
  pointcloud_filters__srv__GetCloudWindow_Request__rosidl_typesupport_introspection_c__GetCloudWindow_Request_init_function,  // function to initialize message memory (memory has to be allocated)
  pointcloud_filters__srv__GetCloudWindow_Request__rosidl_typesupport_introspection_c__GetCloudWindow_Request_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t pointcloud_filters__srv__GetCloudWindow_Request__rosidl_typesupport_introspection_c__GetCloudWindow_Request_message_type_support_handle = {
  0,
  &pointcloud_filters__srv__GetCloudWindow_Request__rosidl_typesupport_introspection_c__GetCloudWindow_Request_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_pointcloud_filters
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, pointcloud_filters, srv, GetCloudWindow_Request)() {
  if (!pointcloud_filters__srv__GetCloudWindow_Request__rosidl_typesupport_introspection_c__GetCloudWindow_Request_message_type_support_handle.typesupport_identifier) {
    pointcloud_filters__srv__GetCloudWindow_Request__rosidl_typesupport_introspection_c__GetCloudWindow_Request_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &pointcloud_filters__srv__GetCloudWindow_Request__rosidl_typesupport_introspection_c__GetCloudWindow_Request_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

// already included above
// #include <stddef.h>
// already included above
// #include "pointcloud_filters/srv/detail/get_cloud_window__rosidl_typesupport_introspection_c.h"
// already included above
// #include "pointcloud_filters/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "rosidl_typesupport_introspection_c/field_types.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
// already included above
// #include "rosidl_typesupport_introspection_c/message_introspection.h"
// already included above
// #include "pointcloud_filters/srv/detail/get_cloud_window__functions.h"
// already included above
// #include "pointcloud_filters/srv/detail/get_cloud_window__struct.h"


// Include directives for member types
// Member `clouds`
// Member `merged_cloud`
#include "sensor_msgs/msg/point_cloud2.h"
// Member `clouds`
// Member `merged_cloud`
#include "sensor_msgs/msg/detail/point_cloud2__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void pointcloud_filters__srv__GetCloudWindow_Response__rosidl_typesupport_introspection_c__GetCloudWindow_Response_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  pointcloud_filters__srv__GetCloudWindow_Response__init(message_memory);
}

void pointcloud_filters__srv__GetCloudWindow_Response__rosidl_typesupport_introspection_c__GetCloudWindow_Response_fini_function(void * message_memory)
{
  pointcloud_filters__srv__GetCloudWindow_Response__fini(message_memory);
}

size_t pointcloud_filters__srv__GetCloudWindow_Response__rosidl_typesupport_introspection_c__size_function__GetCloudWindow_Response__clouds(
  const void * untyped_member)
{
  const sensor_msgs__msg__PointCloud2__Sequence * member =
    (const sensor_msgs__msg__PointCloud2__Sequence *)(untyped_member);
  return member->size;
}

const void * pointcloud_filters__srv__GetCloudWindow_Response__rosidl_typesupport_introspection_c__get_const_function__GetCloudWindow_Response__clouds(
  const void * untyped_member, size_t index)
{
  const sensor_msgs__msg__PointCloud2__Sequence * member =
    (const sensor_msgs__msg__PointCloud2__Sequence *)(untyped_member);
  return &member->data[index];
}

void * pointcloud_filters__srv__GetCloudWindow_Response__rosidl_typesupport_introspection_c__get_function__GetCloudWindow_Response__clouds(
  void * untyped_member, size_t index)
{
  sensor_msgs__msg__PointCloud2__Sequence * member =
    (sensor_msgs__msg__PointCloud2__Sequence *)(untyped_member);
  return &member->data[index];
}

void pointcloud_filters__srv__GetCloudWindow_Response__rosidl_typesupport_introspection_c__fetch_function__GetCloudWindow_Response__clouds(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const sensor_msgs__msg__PointCloud2 * item =
    ((const sensor_msgs__msg__PointCloud2 *)
    pointcloud_filters__srv__GetCloudWindow_Response__rosidl_typesupport_introspection_c__get_const_function__GetCloudWindow_Response__clouds(untyped_member, index));
  sensor_msgs__msg__PointCloud2 * value =
    (sensor_msgs__msg__PointCloud2 *)(untyped_value);
  *value = *item;
}

void pointcloud_filters__srv__GetCloudWindow_Response__rosidl_typesupport_introspection_c__assign_function__GetCloudWindow_Response__clouds(
  void * untyped_member, size_t index, const void * untyped_value)
{
  sensor_msgs__msg__PointCloud2 * item =
    ((sensor_msgs__msg__PointCloud2 *)
    pointcloud_filters__srv__GetCloudWindow_Response__rosidl_typesupport_introspection_c__get_function__GetCloudWindow_Response__clouds(untyped_member, index));
  const sensor_msgs__msg__PointCloud2 * value =
    (const sensor_msgs__msg__PointCloud2 *)(untyped_value);
  *item = *value;
}

bool pointcloud_filters__srv__GetCloudWindow_Response__rosidl_typesupport_introspection_c__resize_function__GetCloudWindow_Response__clouds(
  void * untyped_member, size_t size)
{
  sensor_msgs__msg__PointCloud2__Sequence * member =
    (sensor_msgs__msg__PointCloud2__Sequence *)(untyped_member);
  sensor_msgs__msg__PointCloud2__Sequence__fini(member);
  return sensor_msgs__msg__PointCloud2__Sequence__init(member, size);
}

static rosidl_typesupport_introspection_c__MessageMember pointcloud_filters__srv__GetCloudWindow_Response__rosidl_typesupport_introspection_c__GetCloudWindow_Response_message_member_array[2] = {
  {
    "clouds",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(pointcloud_filters__srv__GetCloudWindow_Response, clouds),  // bytes offset in struct
    NULL,  // default value
    pointcloud_filters__srv__GetCloudWindow_Response__rosidl_typesupport_introspection_c__size_function__GetCloudWindow_Response__clouds,  // size() function pointer
    pointcloud_filters__srv__GetCloudWindow_Response__rosidl_typesupport_introspection_c__get_const_function__GetCloudWindow_Response__clouds,  // get_const(index) function pointer
    pointcloud_filters__srv__GetCloudWindow_Response__rosidl_typesupport_introspection_c__get_function__GetCloudWindow_Response__clouds,  // get(index) function pointer
    pointcloud_filters__srv__GetCloudWindow_Response__rosidl_typesupport_introspection_c__fetch_function__GetCloudWindow_Response__clouds,  // fetch(index, &value) function pointer
    pointcloud_filters__srv__GetCloudWindow_Response__rosidl_typesupport_introspection_c__assign_function__GetCloudWindow_Response__clouds,  // assign(index, value) function pointer
    pointcloud_filters__srv__GetCloudWindow_Response__rosidl_typesupport_introspection_c__resize_function__GetCloudWindow_Response__clouds  // resize(index) function pointer
  },
  {
    "merged_cloud",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(pointcloud_filters__srv__GetCloudWindow_Response, merged_cloud),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers pointcloud_filters__srv__GetCloudWindow_Response__rosidl_typesupport_introspection_c__GetCloudWindow_Response_message_members = {
  "pointcloud_filters__srv",  // message namespace
  "GetCloudWindow_Response",  // message name
  2,  // number of fields
  sizeof(pointcloud_filters__srv__GetCloudWindow_Response),
  pointcloud_filters__srv__GetCloudWindow_Response__rosidl_typesupport_introspection_c__GetCloudWindow_Response_message_member_array,  // message members
  pointcloud_filters__srv__GetCloudWindow_Response__rosidl_typesupport_introspection_c__GetCloudWindow_Response_init_function,  // function to initialize message memory (memory has to be allocated)
  pointcloud_filters__srv__GetCloudWindow_Response__rosidl_typesupport_introspection_c__GetCloudWindow_Response_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t pointcloud_filters__srv__GetCloudWindow_Response__rosidl_typesupport_introspection_c__GetCloudWindow_Response_message_type_support_handle = {
  0,
  &pointcloud_filters__srv__GetCloudWindow_Response__rosidl_typesupport_introspection_c__GetCloudWindow_Response_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_pointcloud_filters
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, pointcloud_filters, srv, GetCloudWindow_Response)() {
  pointcloud_filters__srv__GetCloudWindow_Response__rosidl_typesupport_introspection_c__GetCloudWindow_Response_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, sensor_msgs, msg, PointCloud2)();
  pointcloud_filters__srv__GetCloudWindow_Response__rosidl_typesupport_introspection_c__GetCloudWindow_Response_message_member_array[1].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, sensor_msgs, msg, PointCloud2)();
  if (!pointcloud_filters__srv__GetCloudWindow_Response__rosidl_typesupport_introspection_c__GetCloudWindow_Response_message_type_support_handle.typesupport_identifier) {
    pointcloud_filters__srv__GetCloudWindow_Response__rosidl_typesupport_introspection_c__GetCloudWindow_Response_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &pointcloud_filters__srv__GetCloudWindow_Response__rosidl_typesupport_introspection_c__GetCloudWindow_Response_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

#include "rosidl_runtime_c/service_type_support_struct.h"
// already included above
// #include "pointcloud_filters/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "pointcloud_filters/srv/detail/get_cloud_window__rosidl_typesupport_introspection_c.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/service_introspection.h"

// this is intentionally not const to allow initialization later to prevent an initialization race
static rosidl_typesupport_introspection_c__ServiceMembers pointcloud_filters__srv__detail__get_cloud_window__rosidl_typesupport_introspection_c__GetCloudWindow_service_members = {
  "pointcloud_filters__srv",  // service namespace
  "GetCloudWindow",  // service name
  // these two fields are initialized below on the first access
  NULL,  // request message
  // pointcloud_filters__srv__detail__get_cloud_window__rosidl_typesupport_introspection_c__GetCloudWindow_Request_message_type_support_handle,
  NULL  // response message
  // pointcloud_filters__srv__detail__get_cloud_window__rosidl_typesupport_introspection_c__GetCloudWindow_Response_message_type_support_handle
};

static rosidl_service_type_support_t pointcloud_filters__srv__detail__get_cloud_window__rosidl_typesupport_introspection_c__GetCloudWindow_service_type_support_handle = {
  0,
  &pointcloud_filters__srv__detail__get_cloud_window__rosidl_typesupport_introspection_c__GetCloudWindow_service_members,
  get_service_typesupport_handle_function,
};

// Forward declaration of request/response type support functions
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, pointcloud_filters, srv, GetCloudWindow_Request)();

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, pointcloud_filters, srv, GetCloudWindow_Response)();

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_pointcloud_filters
const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_introspection_c, pointcloud_filters, srv, GetCloudWindow)() {
  if (!pointcloud_filters__srv__detail__get_cloud_window__rosidl_typesupport_introspection_c__GetCloudWindow_service_type_support_handle.typesupport_identifier) {
    pointcloud_filters__srv__detail__get_cloud_window__rosidl_typesupport_introspection_c__GetCloudWindow_service_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  rosidl_typesupport_introspection_c__ServiceMembers * service_members =
    (rosidl_typesupport_introspection_c__ServiceMembers *)pointcloud_filters__srv__detail__get_cloud_window__rosidl_typesupport_introspection_c__GetCloudWindow_service_type_support_handle.data;

  if (!service_members->request_members_) {
    service_members->request_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, pointcloud_filters, srv, GetCloudWindow_Request)()->data;
  }
  if (!service_members->response_members_) {
    service_members->response_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, pointcloud_filters, srv, GetCloudWindow_Response)()->data;
  }

  return &pointcloud_filters__srv__detail__get_cloud_window__rosidl_typesupport_introspection_c__GetCloudWindow_service_type_support_handle;
}
