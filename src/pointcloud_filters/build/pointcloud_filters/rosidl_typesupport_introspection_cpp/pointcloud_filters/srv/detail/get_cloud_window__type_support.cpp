// generated from rosidl_typesupport_introspection_cpp/resource/idl__type_support.cpp.em
// with input from pointcloud_filters:srv/GetCloudWindow.idl
// generated code does not contain a copyright notice

#include "array"
#include "cstddef"
#include "string"
#include "vector"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_interface/macros.h"
#include "pointcloud_filters/srv/detail/get_cloud_window__struct.hpp"
#include "rosidl_typesupport_introspection_cpp/field_types.hpp"
#include "rosidl_typesupport_introspection_cpp/identifier.hpp"
#include "rosidl_typesupport_introspection_cpp/message_introspection.hpp"
#include "rosidl_typesupport_introspection_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_introspection_cpp/visibility_control.h"

namespace pointcloud_filters
{

namespace srv
{

namespace rosidl_typesupport_introspection_cpp
{

void GetCloudWindow_Request_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) pointcloud_filters::srv::GetCloudWindow_Request(_init);
}

void GetCloudWindow_Request_fini_function(void * message_memory)
{
  auto typed_message = static_cast<pointcloud_filters::srv::GetCloudWindow_Request *>(message_memory);
  typed_message->~GetCloudWindow_Request();
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember GetCloudWindow_Request_message_member_array[2] = {
  {
    "window_sec",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(pointcloud_filters::srv::GetCloudWindow_Request, window_sec),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "merged",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(pointcloud_filters::srv::GetCloudWindow_Request, merged),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers GetCloudWindow_Request_message_members = {
  "pointcloud_filters::srv",  // message namespace
  "GetCloudWindow_Request",  // message name
  2,  // number of fields
  sizeof(pointcloud_filters::srv::GetCloudWindow_Request),
  GetCloudWindow_Request_message_member_array,  // message members
  GetCloudWindow_Request_init_function,  // function to initialize message memory (memory has to be allocated)
  GetCloudWindow_Request_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t GetCloudWindow_Request_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &GetCloudWindow_Request_message_members,
  get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace srv

}  // namespace pointcloud_filters


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<pointcloud_filters::srv::GetCloudWindow_Request>()
{
  return &::pointcloud_filters::srv::rosidl_typesupport_introspection_cpp::GetCloudWindow_Request_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, pointcloud_filters, srv, GetCloudWindow_Request)() {
  return &::pointcloud_filters::srv::rosidl_typesupport_introspection_cpp::GetCloudWindow_Request_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif

// already included above
// #include "array"
// already included above
// #include "cstddef"
// already included above
// #include "string"
// already included above
// #include "vector"
// already included above
// #include "rosidl_runtime_c/message_type_support_struct.h"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support.hpp"
// already included above
// #include "rosidl_typesupport_interface/macros.h"
// already included above
// #include "pointcloud_filters/srv/detail/get_cloud_window__struct.hpp"
// already included above
// #include "rosidl_typesupport_introspection_cpp/field_types.hpp"
// already included above
// #include "rosidl_typesupport_introspection_cpp/identifier.hpp"
// already included above
// #include "rosidl_typesupport_introspection_cpp/message_introspection.hpp"
// already included above
// #include "rosidl_typesupport_introspection_cpp/message_type_support_decl.hpp"
// already included above
// #include "rosidl_typesupport_introspection_cpp/visibility_control.h"

namespace pointcloud_filters
{

namespace srv
{

namespace rosidl_typesupport_introspection_cpp
{

void GetCloudWindow_Response_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) pointcloud_filters::srv::GetCloudWindow_Response(_init);
}

void GetCloudWindow_Response_fini_function(void * message_memory)
{
  auto typed_message = static_cast<pointcloud_filters::srv::GetCloudWindow_Response *>(message_memory);
  typed_message->~GetCloudWindow_Response();
}

size_t size_function__GetCloudWindow_Response__clouds(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<sensor_msgs::msg::PointCloud2> *>(untyped_member);
  return member->size();
}

const void * get_const_function__GetCloudWindow_Response__clouds(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<sensor_msgs::msg::PointCloud2> *>(untyped_member);
  return &member[index];
}

void * get_function__GetCloudWindow_Response__clouds(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<sensor_msgs::msg::PointCloud2> *>(untyped_member);
  return &member[index];
}

void fetch_function__GetCloudWindow_Response__clouds(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const sensor_msgs::msg::PointCloud2 *>(
    get_const_function__GetCloudWindow_Response__clouds(untyped_member, index));
  auto & value = *reinterpret_cast<sensor_msgs::msg::PointCloud2 *>(untyped_value);
  value = item;
}

void assign_function__GetCloudWindow_Response__clouds(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<sensor_msgs::msg::PointCloud2 *>(
    get_function__GetCloudWindow_Response__clouds(untyped_member, index));
  const auto & value = *reinterpret_cast<const sensor_msgs::msg::PointCloud2 *>(untyped_value);
  item = value;
}

void resize_function__GetCloudWindow_Response__clouds(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<sensor_msgs::msg::PointCloud2> *>(untyped_member);
  member->resize(size);
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember GetCloudWindow_Response_message_member_array[2] = {
  {
    "clouds",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<sensor_msgs::msg::PointCloud2>(),  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(pointcloud_filters::srv::GetCloudWindow_Response, clouds),  // bytes offset in struct
    nullptr,  // default value
    size_function__GetCloudWindow_Response__clouds,  // size() function pointer
    get_const_function__GetCloudWindow_Response__clouds,  // get_const(index) function pointer
    get_function__GetCloudWindow_Response__clouds,  // get(index) function pointer
    fetch_function__GetCloudWindow_Response__clouds,  // fetch(index, &value) function pointer
    assign_function__GetCloudWindow_Response__clouds,  // assign(index, value) function pointer
    resize_function__GetCloudWindow_Response__clouds  // resize(index) function pointer
  },
  {
    "merged_cloud",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<sensor_msgs::msg::PointCloud2>(),  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(pointcloud_filters::srv::GetCloudWindow_Response, merged_cloud),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers GetCloudWindow_Response_message_members = {
  "pointcloud_filters::srv",  // message namespace
  "GetCloudWindow_Response",  // message name
  2,  // number of fields
  sizeof(pointcloud_filters::srv::GetCloudWindow_Response),
  GetCloudWindow_Response_message_member_array,  // message members
  GetCloudWindow_Response_init_function,  // function to initialize message memory (memory has to be allocated)
  GetCloudWindow_Response_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t GetCloudWindow_Response_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &GetCloudWindow_Response_message_members,
  get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace srv

}  // namespace pointcloud_filters


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<pointcloud_filters::srv::GetCloudWindow_Response>()
{
  return &::pointcloud_filters::srv::rosidl_typesupport_introspection_cpp::GetCloudWindow_Response_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, pointcloud_filters, srv, GetCloudWindow_Response)() {
  return &::pointcloud_filters::srv::rosidl_typesupport_introspection_cpp::GetCloudWindow_Response_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif

#include "rosidl_runtime_c/service_type_support_struct.h"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_cpp/service_type_support.hpp"
// already included above
// #include "rosidl_typesupport_interface/macros.h"
// already included above
// #include "rosidl_typesupport_introspection_cpp/visibility_control.h"
// already included above
// #include "pointcloud_filters/srv/detail/get_cloud_window__struct.hpp"
// already included above
// #include "rosidl_typesupport_introspection_cpp/identifier.hpp"
// already included above
// #include "rosidl_typesupport_introspection_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_introspection_cpp/service_introspection.hpp"
#include "rosidl_typesupport_introspection_cpp/service_type_support_decl.hpp"

namespace pointcloud_filters
{

namespace srv
{

namespace rosidl_typesupport_introspection_cpp
{

// this is intentionally not const to allow initialization later to prevent an initialization race
static ::rosidl_typesupport_introspection_cpp::ServiceMembers GetCloudWindow_service_members = {
  "pointcloud_filters::srv",  // service namespace
  "GetCloudWindow",  // service name
  // these two fields are initialized below on the first access
  // see get_service_type_support_handle<pointcloud_filters::srv::GetCloudWindow>()
  nullptr,  // request message
  nullptr  // response message
};

static const rosidl_service_type_support_t GetCloudWindow_service_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &GetCloudWindow_service_members,
  get_service_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace srv

}  // namespace pointcloud_filters


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_service_type_support_t *
get_service_type_support_handle<pointcloud_filters::srv::GetCloudWindow>()
{
  // get a handle to the value to be returned
  auto service_type_support =
    &::pointcloud_filters::srv::rosidl_typesupport_introspection_cpp::GetCloudWindow_service_type_support_handle;
  // get a non-const and properly typed version of the data void *
  auto service_members = const_cast<::rosidl_typesupport_introspection_cpp::ServiceMembers *>(
    static_cast<const ::rosidl_typesupport_introspection_cpp::ServiceMembers *>(
      service_type_support->data));
  // make sure that both the request_members_ and the response_members_ are initialized
  // if they are not, initialize them
  if (
    service_members->request_members_ == nullptr ||
    service_members->response_members_ == nullptr)
  {
    // initialize the request_members_ with the static function from the external library
    service_members->request_members_ = static_cast<
      const ::rosidl_typesupport_introspection_cpp::MessageMembers *
      >(
      ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<
        ::pointcloud_filters::srv::GetCloudWindow_Request
      >()->data
      );
    // initialize the response_members_ with the static function from the external library
    service_members->response_members_ = static_cast<
      const ::rosidl_typesupport_introspection_cpp::MessageMembers *
      >(
      ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<
        ::pointcloud_filters::srv::GetCloudWindow_Response
      >()->data
      );
  }
  // finally return the properly initialized service_type_support handle
  return service_type_support;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, pointcloud_filters, srv, GetCloudWindow)() {
  return ::rosidl_typesupport_introspection_cpp::get_service_type_support_handle<pointcloud_filters::srv::GetCloudWindow>();
}

#ifdef __cplusplus
}
#endif
