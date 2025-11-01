// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from pointcloud_filters:srv/GetCloudWindow.idl
// generated code does not contain a copyright notice

#ifndef POINTCLOUD_FILTERS__SRV__DETAIL__GET_CLOUD_WINDOW__TRAITS_HPP_
#define POINTCLOUD_FILTERS__SRV__DETAIL__GET_CLOUD_WINDOW__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "pointcloud_filters/srv/detail/get_cloud_window__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace pointcloud_filters
{

namespace srv
{

inline void to_flow_style_yaml(
  const GetCloudWindow_Request & msg,
  std::ostream & out)
{
  out << "{";
  // member: window_sec
  {
    out << "window_sec: ";
    rosidl_generator_traits::value_to_yaml(msg.window_sec, out);
    out << ", ";
  }

  // member: merged
  {
    out << "merged: ";
    rosidl_generator_traits::value_to_yaml(msg.merged, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const GetCloudWindow_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: window_sec
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "window_sec: ";
    rosidl_generator_traits::value_to_yaml(msg.window_sec, out);
    out << "\n";
  }

  // member: merged
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "merged: ";
    rosidl_generator_traits::value_to_yaml(msg.merged, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const GetCloudWindow_Request & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace srv

}  // namespace pointcloud_filters

namespace rosidl_generator_traits
{

[[deprecated("use pointcloud_filters::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const pointcloud_filters::srv::GetCloudWindow_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  pointcloud_filters::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use pointcloud_filters::srv::to_yaml() instead")]]
inline std::string to_yaml(const pointcloud_filters::srv::GetCloudWindow_Request & msg)
{
  return pointcloud_filters::srv::to_yaml(msg);
}

template<>
inline const char * data_type<pointcloud_filters::srv::GetCloudWindow_Request>()
{
  return "pointcloud_filters::srv::GetCloudWindow_Request";
}

template<>
inline const char * name<pointcloud_filters::srv::GetCloudWindow_Request>()
{
  return "pointcloud_filters/srv/GetCloudWindow_Request";
}

template<>
struct has_fixed_size<pointcloud_filters::srv::GetCloudWindow_Request>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<pointcloud_filters::srv::GetCloudWindow_Request>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<pointcloud_filters::srv::GetCloudWindow_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

// Include directives for member types
// Member 'clouds'
// Member 'merged_cloud'
#include "sensor_msgs/msg/detail/point_cloud2__traits.hpp"

namespace pointcloud_filters
{

namespace srv
{

inline void to_flow_style_yaml(
  const GetCloudWindow_Response & msg,
  std::ostream & out)
{
  out << "{";
  // member: clouds
  {
    if (msg.clouds.size() == 0) {
      out << "clouds: []";
    } else {
      out << "clouds: [";
      size_t pending_items = msg.clouds.size();
      for (auto item : msg.clouds) {
        to_flow_style_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: merged_cloud
  {
    out << "merged_cloud: ";
    to_flow_style_yaml(msg.merged_cloud, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const GetCloudWindow_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: clouds
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.clouds.size() == 0) {
      out << "clouds: []\n";
    } else {
      out << "clouds:\n";
      for (auto item : msg.clouds) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "-\n";
        to_block_style_yaml(item, out, indentation + 2);
      }
    }
  }

  // member: merged_cloud
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "merged_cloud:\n";
    to_block_style_yaml(msg.merged_cloud, out, indentation + 2);
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const GetCloudWindow_Response & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace srv

}  // namespace pointcloud_filters

namespace rosidl_generator_traits
{

[[deprecated("use pointcloud_filters::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const pointcloud_filters::srv::GetCloudWindow_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  pointcloud_filters::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use pointcloud_filters::srv::to_yaml() instead")]]
inline std::string to_yaml(const pointcloud_filters::srv::GetCloudWindow_Response & msg)
{
  return pointcloud_filters::srv::to_yaml(msg);
}

template<>
inline const char * data_type<pointcloud_filters::srv::GetCloudWindow_Response>()
{
  return "pointcloud_filters::srv::GetCloudWindow_Response";
}

template<>
inline const char * name<pointcloud_filters::srv::GetCloudWindow_Response>()
{
  return "pointcloud_filters/srv/GetCloudWindow_Response";
}

template<>
struct has_fixed_size<pointcloud_filters::srv::GetCloudWindow_Response>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<pointcloud_filters::srv::GetCloudWindow_Response>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<pointcloud_filters::srv::GetCloudWindow_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<pointcloud_filters::srv::GetCloudWindow>()
{
  return "pointcloud_filters::srv::GetCloudWindow";
}

template<>
inline const char * name<pointcloud_filters::srv::GetCloudWindow>()
{
  return "pointcloud_filters/srv/GetCloudWindow";
}

template<>
struct has_fixed_size<pointcloud_filters::srv::GetCloudWindow>
  : std::integral_constant<
    bool,
    has_fixed_size<pointcloud_filters::srv::GetCloudWindow_Request>::value &&
    has_fixed_size<pointcloud_filters::srv::GetCloudWindow_Response>::value
  >
{
};

template<>
struct has_bounded_size<pointcloud_filters::srv::GetCloudWindow>
  : std::integral_constant<
    bool,
    has_bounded_size<pointcloud_filters::srv::GetCloudWindow_Request>::value &&
    has_bounded_size<pointcloud_filters::srv::GetCloudWindow_Response>::value
  >
{
};

template<>
struct is_service<pointcloud_filters::srv::GetCloudWindow>
  : std::true_type
{
};

template<>
struct is_service_request<pointcloud_filters::srv::GetCloudWindow_Request>
  : std::true_type
{
};

template<>
struct is_service_response<pointcloud_filters::srv::GetCloudWindow_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // POINTCLOUD_FILTERS__SRV__DETAIL__GET_CLOUD_WINDOW__TRAITS_HPP_
