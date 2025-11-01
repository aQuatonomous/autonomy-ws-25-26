// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from pointcloud_filters:srv/GetCloudWindow.idl
// generated code does not contain a copyright notice

#ifndef POINTCLOUD_FILTERS__SRV__DETAIL__GET_CLOUD_WINDOW__STRUCT_HPP_
#define POINTCLOUD_FILTERS__SRV__DETAIL__GET_CLOUD_WINDOW__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__pointcloud_filters__srv__GetCloudWindow_Request __attribute__((deprecated))
#else
# define DEPRECATED__pointcloud_filters__srv__GetCloudWindow_Request __declspec(deprecated)
#endif

namespace pointcloud_filters
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct GetCloudWindow_Request_
{
  using Type = GetCloudWindow_Request_<ContainerAllocator>;

  explicit GetCloudWindow_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->window_sec = 0.0f;
      this->merged = false;
    }
  }

  explicit GetCloudWindow_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->window_sec = 0.0f;
      this->merged = false;
    }
  }

  // field types and members
  using _window_sec_type =
    float;
  _window_sec_type window_sec;
  using _merged_type =
    bool;
  _merged_type merged;

  // setters for named parameter idiom
  Type & set__window_sec(
    const float & _arg)
  {
    this->window_sec = _arg;
    return *this;
  }
  Type & set__merged(
    const bool & _arg)
  {
    this->merged = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    pointcloud_filters::srv::GetCloudWindow_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const pointcloud_filters::srv::GetCloudWindow_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<pointcloud_filters::srv::GetCloudWindow_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<pointcloud_filters::srv::GetCloudWindow_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      pointcloud_filters::srv::GetCloudWindow_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<pointcloud_filters::srv::GetCloudWindow_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      pointcloud_filters::srv::GetCloudWindow_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<pointcloud_filters::srv::GetCloudWindow_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<pointcloud_filters::srv::GetCloudWindow_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<pointcloud_filters::srv::GetCloudWindow_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__pointcloud_filters__srv__GetCloudWindow_Request
    std::shared_ptr<pointcloud_filters::srv::GetCloudWindow_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__pointcloud_filters__srv__GetCloudWindow_Request
    std::shared_ptr<pointcloud_filters::srv::GetCloudWindow_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const GetCloudWindow_Request_ & other) const
  {
    if (this->window_sec != other.window_sec) {
      return false;
    }
    if (this->merged != other.merged) {
      return false;
    }
    return true;
  }
  bool operator!=(const GetCloudWindow_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct GetCloudWindow_Request_

// alias to use template instance with default allocator
using GetCloudWindow_Request =
  pointcloud_filters::srv::GetCloudWindow_Request_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace pointcloud_filters


// Include directives for member types
// Member 'clouds'
// Member 'merged_cloud'
#include "sensor_msgs/msg/detail/point_cloud2__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__pointcloud_filters__srv__GetCloudWindow_Response __attribute__((deprecated))
#else
# define DEPRECATED__pointcloud_filters__srv__GetCloudWindow_Response __declspec(deprecated)
#endif

namespace pointcloud_filters
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct GetCloudWindow_Response_
{
  using Type = GetCloudWindow_Response_<ContainerAllocator>;

  explicit GetCloudWindow_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : merged_cloud(_init)
  {
    (void)_init;
  }

  explicit GetCloudWindow_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : merged_cloud(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _clouds_type =
    std::vector<sensor_msgs::msg::PointCloud2_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<sensor_msgs::msg::PointCloud2_<ContainerAllocator>>>;
  _clouds_type clouds;
  using _merged_cloud_type =
    sensor_msgs::msg::PointCloud2_<ContainerAllocator>;
  _merged_cloud_type merged_cloud;

  // setters for named parameter idiom
  Type & set__clouds(
    const std::vector<sensor_msgs::msg::PointCloud2_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<sensor_msgs::msg::PointCloud2_<ContainerAllocator>>> & _arg)
  {
    this->clouds = _arg;
    return *this;
  }
  Type & set__merged_cloud(
    const sensor_msgs::msg::PointCloud2_<ContainerAllocator> & _arg)
  {
    this->merged_cloud = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    pointcloud_filters::srv::GetCloudWindow_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const pointcloud_filters::srv::GetCloudWindow_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<pointcloud_filters::srv::GetCloudWindow_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<pointcloud_filters::srv::GetCloudWindow_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      pointcloud_filters::srv::GetCloudWindow_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<pointcloud_filters::srv::GetCloudWindow_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      pointcloud_filters::srv::GetCloudWindow_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<pointcloud_filters::srv::GetCloudWindow_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<pointcloud_filters::srv::GetCloudWindow_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<pointcloud_filters::srv::GetCloudWindow_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__pointcloud_filters__srv__GetCloudWindow_Response
    std::shared_ptr<pointcloud_filters::srv::GetCloudWindow_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__pointcloud_filters__srv__GetCloudWindow_Response
    std::shared_ptr<pointcloud_filters::srv::GetCloudWindow_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const GetCloudWindow_Response_ & other) const
  {
    if (this->clouds != other.clouds) {
      return false;
    }
    if (this->merged_cloud != other.merged_cloud) {
      return false;
    }
    return true;
  }
  bool operator!=(const GetCloudWindow_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct GetCloudWindow_Response_

// alias to use template instance with default allocator
using GetCloudWindow_Response =
  pointcloud_filters::srv::GetCloudWindow_Response_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace pointcloud_filters

namespace pointcloud_filters
{

namespace srv
{

struct GetCloudWindow
{
  using Request = pointcloud_filters::srv::GetCloudWindow_Request;
  using Response = pointcloud_filters::srv::GetCloudWindow_Response;
};

}  // namespace srv

}  // namespace pointcloud_filters

#endif  // POINTCLOUD_FILTERS__SRV__DETAIL__GET_CLOUD_WINDOW__STRUCT_HPP_
