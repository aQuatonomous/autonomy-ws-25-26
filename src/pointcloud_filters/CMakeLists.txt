cmake_minimum_required(VERSION 3.5)
project(pointcloud_filters)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

# Generate service and message interfaces
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/GetCloudWindow.srv"
  "msg/BuoyDetection.msg"
  "msg/BuoyDetectionArray.msg"
  DEPENDENCIES sensor_msgs std_msgs
)

# Install Python scripts
install(PROGRAMS
  pointcloud_filters/lidar_range_filter.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME lidar_range_filter
)

# Install buoy_detector separately (can't have multiple files with RENAME)
install(PROGRAMS
  pointcloud_filters/buoy_detector.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME buoy_detector
)

# Install buoy_visualizer separately
install(PROGRAMS
  pointcloud_filters/buoy_visualizer.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME buoy_visualizer
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
