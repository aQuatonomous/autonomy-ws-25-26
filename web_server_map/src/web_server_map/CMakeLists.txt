cmake_minimum_required(VERSION 3.5)
project(web_server_map)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

# Install Python package
ament_python_install_package(${PROJECT_NAME}
  PACKAGE_DIR src/${PROJECT_NAME}
)

# Install Python executables
install(PROGRAMS
  src/${PROJECT_NAME}/map_visualizer_node.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install web files
install(DIRECTORY web/
  DESTINATION share/${PROJECT_NAME}/web
)

# Install launch files
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# Install resource marker file
install(FILES resource/${PROJECT_NAME}
  DESTINATION share/ament_index/resource_index/packages
)

ament_package()
