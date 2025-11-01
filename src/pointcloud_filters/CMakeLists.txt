cmake_minimum_required(VERSION 3.5)
project(pointcloud_filters)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

# Generate service interfaces
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/GetCloudWindow.srv"
  DEPENDENCIES sensor_msgs std_msgs
)

# Install Python scripts
install(PROGRAMS
  pointcloud_filters/lidar_range_filter.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME lidar_range_filter
)

# Install Python module (without ament_python_install_package to avoid duplication)
install(DIRECTORY pointcloud_filters/
  DESTINATION lib/python3.10/site-packages/${PROJECT_NAME}
  FILES_MATCHING PATTERN "*.py"
)

# Install launch files
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

ament_package()
