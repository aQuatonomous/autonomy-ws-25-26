# Install script for directory: /home/lorenzo/autonomy-ws-25-26/src/asv_wave_sim/gz-waves/src

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/lorenzo/autonomy-ws-25-26/src/asv_wave_sim/install")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "RelWithDebInfo")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Install shared libraries without execute permission?
if(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  set(CMAKE_INSTALL_SO_NO_EXE "1")
endif()

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "FALSE")
endif()

# Set default install directory permissions.
if(NOT DEFINED CMAKE_OBJDUMP)
  set(CMAKE_OBJDUMP "/usr/bin/objdump")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "headers" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/gz/waves1/gz/waves/detail" TYPE FILE FILES "/home/lorenzo/autonomy-ws-25-26/src/asv_wave_sim/build/gz-waves1/include/gz/waves/detail/Export.hh")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "headers" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/gz/waves1/gz/waves" TYPE FILE FILES "/home/lorenzo/autonomy-ws-25-26/src/asv_wave_sim/build/gz-waves1/include/gz/waves/Export.hh")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  foreach(file
      "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libgz-waves1.so.1.0.0"
      "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libgz-waves1.so.1"
      )
    if(EXISTS "${file}" AND
       NOT IS_SYMLINK "${file}")
      file(RPATH_CHECK
           FILE "${file}"
           RPATH "")
    endif()
  endforeach()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" TYPE SHARED_LIBRARY FILES
    "/home/lorenzo/autonomy-ws-25-26/src/asv_wave_sim/build/gz-waves1/lib/libgz-waves1.so.1.0.0"
    "/home/lorenzo/autonomy-ws-25-26/src/asv_wave_sim/build/gz-waves1/lib/libgz-waves1.so.1"
    )
  foreach(file
      "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libgz-waves1.so.1.0.0"
      "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libgz-waves1.so.1"
      )
    if(EXISTS "${file}" AND
       NOT IS_SYMLINK "${file}")
      file(RPATH_CHANGE
           FILE "${file}"
           OLD_RPATH "/opt/ros/jazzy/opt/gz_math_vendor/lib:/opt/ros/jazzy/opt/gz_utils_vendor/lib:"
           NEW_RPATH "")
      if(CMAKE_INSTALL_DO_STRIP)
        execute_process(COMMAND "/usr/bin/strip" "${file}")
      endif()
    endif()
  endforeach()
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" TYPE SHARED_LIBRARY FILES "/home/lorenzo/autonomy-ws-25-26/src/asv_wave_sim/build/gz-waves1/lib/libgz-waves1.so")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "cmake" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/cmake/gz-waves1" TYPE FILE FILES
    "/home/lorenzo/autonomy-ws-25-26/src/asv_wave_sim/build/gz-waves1/cmake/gz-waves1-config.cmake"
    "/home/lorenzo/autonomy-ws-25-26/src/asv_wave_sim/build/gz-waves1/cmake/gz-waves1-config-version.cmake"
    )
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/cmake/gz-waves1/gz-waves1-targets.cmake")
    file(DIFFERENT _cmake_export_file_changed FILES
         "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/cmake/gz-waves1/gz-waves1-targets.cmake"
         "/home/lorenzo/autonomy-ws-25-26/src/asv_wave_sim/build/gz-waves1/src/CMakeFiles/Export/0ceb622fb857a2fb3432b2f869f00b89/gz-waves1-targets.cmake")
    if(_cmake_export_file_changed)
      file(GLOB _cmake_old_config_files "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/cmake/gz-waves1/gz-waves1-targets-*.cmake")
      if(_cmake_old_config_files)
        string(REPLACE ";" ", " _cmake_old_config_files_text "${_cmake_old_config_files}")
        message(STATUS "Old export file \"$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/cmake/gz-waves1/gz-waves1-targets.cmake\" will be replaced.  Removing files [${_cmake_old_config_files_text}].")
        unset(_cmake_old_config_files_text)
        file(REMOVE ${_cmake_old_config_files})
      endif()
      unset(_cmake_old_config_files)
    endif()
    unset(_cmake_export_file_changed)
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/cmake/gz-waves1" TYPE FILE FILES "/home/lorenzo/autonomy-ws-25-26/src/asv_wave_sim/build/gz-waves1/src/CMakeFiles/Export/0ceb622fb857a2fb3432b2f869f00b89/gz-waves1-targets.cmake")
  if(CMAKE_INSTALL_CONFIG_NAME MATCHES "^([Rr][Ee][Ll][Ww][Ii][Tt][Hh][Dd][Ee][Bb][Ii][Nn][Ff][Oo])$")
    file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/cmake/gz-waves1" TYPE FILE FILES "/home/lorenzo/autonomy-ws-25-26/src/asv_wave_sim/build/gz-waves1/src/CMakeFiles/Export/0ceb622fb857a2fb3432b2f869f00b89/gz-waves1-targets-relwithdebinfo.cmake")
  endif()
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "pkgconfig" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/lorenzo/autonomy-ws-25-26/src/asv_wave_sim/build/gz-waves1/cmake/pkgconfig/gz-waves1.pc")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for each subdirectory.
  include("/home/lorenzo/autonomy-ws-25-26/src/asv_wave_sim/build/gz-waves1/src/gui/cmake_install.cmake")
  include("/home/lorenzo/autonomy-ws-25-26/src/asv_wave_sim/build/gz-waves1/src/systems/cmake_install.cmake")

endif()

