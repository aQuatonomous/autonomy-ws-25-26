// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from pointcloud_filters:srv/GetCloudWindow.idl
// generated code does not contain a copyright notice

#ifndef POINTCLOUD_FILTERS__SRV__DETAIL__GET_CLOUD_WINDOW__BUILDER_HPP_
#define POINTCLOUD_FILTERS__SRV__DETAIL__GET_CLOUD_WINDOW__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "pointcloud_filters/srv/detail/get_cloud_window__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace pointcloud_filters
{

namespace srv
{

namespace builder
{

class Init_GetCloudWindow_Request_merged
{
public:
  explicit Init_GetCloudWindow_Request_merged(::pointcloud_filters::srv::GetCloudWindow_Request & msg)
  : msg_(msg)
  {}
  ::pointcloud_filters::srv::GetCloudWindow_Request merged(::pointcloud_filters::srv::GetCloudWindow_Request::_merged_type arg)
  {
    msg_.merged = std::move(arg);
    return std::move(msg_);
  }

private:
  ::pointcloud_filters::srv::GetCloudWindow_Request msg_;
};

class Init_GetCloudWindow_Request_window_sec
{
public:
  Init_GetCloudWindow_Request_window_sec()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_GetCloudWindow_Request_merged window_sec(::pointcloud_filters::srv::GetCloudWindow_Request::_window_sec_type arg)
  {
    msg_.window_sec = std::move(arg);
    return Init_GetCloudWindow_Request_merged(msg_);
  }

private:
  ::pointcloud_filters::srv::GetCloudWindow_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::pointcloud_filters::srv::GetCloudWindow_Request>()
{
  return pointcloud_filters::srv::builder::Init_GetCloudWindow_Request_window_sec();
}

}  // namespace pointcloud_filters


namespace pointcloud_filters
{

namespace srv
{

namespace builder
{

class Init_GetCloudWindow_Response_merged_cloud
{
public:
  explicit Init_GetCloudWindow_Response_merged_cloud(::pointcloud_filters::srv::GetCloudWindow_Response & msg)
  : msg_(msg)
  {}
  ::pointcloud_filters::srv::GetCloudWindow_Response merged_cloud(::pointcloud_filters::srv::GetCloudWindow_Response::_merged_cloud_type arg)
  {
    msg_.merged_cloud = std::move(arg);
    return std::move(msg_);
  }

private:
  ::pointcloud_filters::srv::GetCloudWindow_Response msg_;
};

class Init_GetCloudWindow_Response_clouds
{
public:
  Init_GetCloudWindow_Response_clouds()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_GetCloudWindow_Response_merged_cloud clouds(::pointcloud_filters::srv::GetCloudWindow_Response::_clouds_type arg)
  {
    msg_.clouds = std::move(arg);
    return Init_GetCloudWindow_Response_merged_cloud(msg_);
  }

private:
  ::pointcloud_filters::srv::GetCloudWindow_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::pointcloud_filters::srv::GetCloudWindow_Response>()
{
  return pointcloud_filters::srv::builder::Init_GetCloudWindow_Response_clouds();
}

}  // namespace pointcloud_filters

#endif  // POINTCLOUD_FILTERS__SRV__DETAIL__GET_CLOUD_WINDOW__BUILDER_HPP_
