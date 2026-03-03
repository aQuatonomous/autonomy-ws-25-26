# Install script for directory: /home/lorenzo/autonomy-ws-25-26/src/asv_wave_sim/gz-waves/src/systems/waves

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
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/gz/waves1/gz/waves/rendering/detail" TYPE FILE FILES "/home/lorenzo/autonomy-ws-25-26/src/asv_wave_sim/build/gz-waves1/include/gz/waves/rendering/detail/Export.hh")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "headers" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/gz/waves1/gz/waves/rendering" TYPE FILE FILES "/home/lorenzo/autonomy-ws-25-26/src/asv_wave_sim/build/gz-waves1/include/gz/waves/rendering/Export.hh")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  foreach(file
      "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libgz-waves1-rendering.so.1.0.0"
      "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libgz-waves1-rendering.so.1"
      )
    if(EXISTS "${file}" AND
       NOT IS_SYMLINK "${file}")
      file(RPATH_CHECK
           FILE "${file}"
           RPATH "")
    endif()
  endforeach()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" TYPE SHARED_LIBRARY FILES
    "/home/lorenzo/autonomy-ws-25-26/src/asv_wave_sim/build/gz-waves1/lib/libgz-waves1-rendering.so.1.0.0"
    "/home/lorenzo/autonomy-ws-25-26/src/asv_wave_sim/build/gz-waves1/lib/libgz-waves1-rendering.so.1"
    )
  foreach(file
      "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libgz-waves1-rendering.so.1.0.0"
      "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libgz-waves1-rendering.so.1"
      )
    if(EXISTS "${file}" AND
       NOT IS_SYMLINK "${file}")
      file(RPATH_CHANGE
           FILE "${file}"
           OLD_RPATH "/usr/include/gz/rendering8/../../../lib/gz-rendering-8/engine-plugins:/home/lorenzo/autonomy-ws-25-26/src/asv_wave_sim/build/gz-waves1/lib:/opt/ros/jazzy/opt/gz_math_vendor/lib:/opt/ros/jazzy/opt/gz_utils_vendor/lib:"
           NEW_RPATH "")
      if(CMAKE_INSTALL_DO_STRIP)
        execute_process(COMMAND "/usr/bin/strip" "${file}")
      endif()
    endif()
  endforeach()
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" TYPE SHARED_LIBRARY FILES "/home/lorenzo/autonomy-ws-25-26/src/asv_wave_sim/build/gz-waves1/lib/libgz-waves1-rendering.so")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "cmake" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/cmake/gz-waves1-rendering" TYPE FILE FILES
    "/home/lorenzo/autonomy-ws-25-26/src/asv_wave_sim/build/gz-waves1/cmake/gz-waves1-rendering-config.cmake"
    "/home/lorenzo/autonomy-ws-25-26/src/asv_wave_sim/build/gz-waves1/cmake/gz-waves1-rendering-config-version.cmake"
    )
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/cmake/gz-waves1-rendering/gz-waves1-rendering-targets.cmake")
    file(DIFFERENT _cmake_export_file_changed FILES
         "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/cmake/gz-waves1-rendering/gz-waves1-rendering-targets.cmake"
         "/home/lorenzo/autonomy-ws-25-26/src/asv_wave_sim/build/gz-waves1/src/systems/waves/CMakeFiles/Export/79255f64ce39bd6f2a749a65c0605aca/gz-waves1-rendering-targets.cmake")
    if(_cmake_export_file_changed)
      file(GLOB _cmake_old_config_files "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/cmake/gz-waves1-rendering/gz-waves1-rendering-targets-*.cmake")
      if(_cmake_old_config_files)
        string(REPLACE ";" ", " _cmake_old_config_files_text "${_cmake_old_config_files}")
        message(STATUS "Old export file \"$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/cmake/gz-waves1-rendering/gz-waves1-rendering-targets.cmake\" will be replaced.  Removing files [${_cmake_old_config_files_text}].")
        unset(_cmake_old_config_files_text)
        file(REMOVE ${_cmake_old_config_files})
      endif()
      unset(_cmake_old_config_files)
    endif()
    unset(_cmake_export_file_changed)
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/cmake/gz-waves1-rendering" TYPE FILE FILES "/home/lorenzo/autonomy-ws-25-26/src/asv_wave_sim/build/gz-waves1/src/systems/waves/CMakeFiles/Export/79255f64ce39bd6f2a749a65c0605aca/gz-waves1-rendering-targets.cmake")
  if(CMAKE_INSTALL_CONFIG_NAME MATCHES "^([Rr][Ee][Ll][Ww][Ii][Tt][Hh][Dd][Ee][Bb][Ii][Nn][Ff][Oo])$")
    file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/cmake/gz-waves1-rendering" TYPE FILE FILES "/home/lorenzo/autonomy-ws-25-26/src/asv_wave_sim/build/gz-waves1/src/systems/waves/CMakeFiles/Export/79255f64ce39bd6f2a749a65c0605aca/gz-waves1-rendering-targets-relwithdebinfo.cmake")
  endif()
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "pkgconfig" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/lorenzo/autonomy-ws-25-26/src/asv_wave_sim/build/gz-waves1/cmake/pkgconfig/gz-waves1-rendering.pc")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "headers" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/gz/waves1/gz/waves/rendering-ogre2/detail" TYPE FILE FILES "/home/lorenzo/autonomy-ws-25-26/src/asv_wave_sim/build/gz-waves1/include/gz/waves/rendering-ogre2/detail/Export.hh")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "headers" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/gz/waves1/gz/waves/rendering-ogre2" TYPE FILE FILES "/home/lorenzo/autonomy-ws-25-26/src/asv_wave_sim/build/gz-waves1/include/gz/waves/rendering-ogre2/Export.hh")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  foreach(file
      "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libgz-waves1-rendering-ogre2.so.1.0.0"
      "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libgz-waves1-rendering-ogre2.so.1"
      )
    if(EXISTS "${file}" AND
       NOT IS_SYMLINK "${file}")
      file(RPATH_CHECK
           FILE "${file}"
           RPATH "")
    endif()
  endforeach()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" TYPE SHARED_LIBRARY FILES
    "/home/lorenzo/autonomy-ws-25-26/src/asv_wave_sim/build/gz-waves1/lib/libgz-waves1-rendering-ogre2.so.1.0.0"
    "/home/lorenzo/autonomy-ws-25-26/src/asv_wave_sim/build/gz-waves1/lib/libgz-waves1-rendering-ogre2.so.1"
    )
  foreach(file
      "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libgz-waves1-rendering-ogre2.so.1.0.0"
      "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libgz-waves1-rendering-ogre2.so.1"
      )
    if(EXISTS "${file}" AND
       NOT IS_SYMLINK "${file}")
      file(RPATH_CHANGE
           FILE "${file}"
           OLD_RPATH "/usr/include/gz/rendering8/../../../lib/gz-rendering-8/engine-plugins:/usr/lib/x86_64-linux-gnu/OGRE-2.3:/home/lorenzo/autonomy-ws-25-26/src/asv_wave_sim/build/gz-waves1/lib:/opt/ros/jazzy/opt/gz_math_vendor/lib:/opt/ros/jazzy/opt/gz_utils_vendor/lib:"
           NEW_RPATH "")
      if(CMAKE_INSTALL_DO_STRIP)
        execute_process(COMMAND "/usr/bin/strip" "${file}")
      endif()
    endif()
  endforeach()
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" TYPE SHARED_LIBRARY FILES "/home/lorenzo/autonomy-ws-25-26/src/asv_wave_sim/build/gz-waves1/lib/libgz-waves1-rendering-ogre2.so")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "cmake" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/cmake/gz-waves1-rendering-ogre2" TYPE FILE FILES
    "/home/lorenzo/autonomy-ws-25-26/src/asv_wave_sim/build/gz-waves1/cmake/gz-waves1-rendering-ogre2-config.cmake"
    "/home/lorenzo/autonomy-ws-25-26/src/asv_wave_sim/build/gz-waves1/cmake/gz-waves1-rendering-ogre2-config-version.cmake"
    )
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/cmake/gz-waves1-rendering-ogre2/gz-waves1-rendering-ogre2-targets.cmake")
    file(DIFFERENT _cmake_export_file_changed FILES
         "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/cmake/gz-waves1-rendering-ogre2/gz-waves1-rendering-ogre2-targets.cmake"
         "/home/lorenzo/autonomy-ws-25-26/src/asv_wave_sim/build/gz-waves1/src/systems/waves/CMakeFiles/Export/b58ecaf284d03b5477d284d0202b06d0/gz-waves1-rendering-ogre2-targets.cmake")
    if(_cmake_export_file_changed)
      file(GLOB _cmake_old_config_files "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/cmake/gz-waves1-rendering-ogre2/gz-waves1-rendering-ogre2-targets-*.cmake")
      if(_cmake_old_config_files)
        string(REPLACE ";" ", " _cmake_old_config_files_text "${_cmake_old_config_files}")
        message(STATUS "Old export file \"$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/cmake/gz-waves1-rendering-ogre2/gz-waves1-rendering-ogre2-targets.cmake\" will be replaced.  Removing files [${_cmake_old_config_files_text}].")
        unset(_cmake_old_config_files_text)
        file(REMOVE ${_cmake_old_config_files})
      endif()
      unset(_cmake_old_config_files)
    endif()
    unset(_cmake_export_file_changed)
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/cmake/gz-waves1-rendering-ogre2" TYPE FILE FILES "/home/lorenzo/autonomy-ws-25-26/src/asv_wave_sim/build/gz-waves1/src/systems/waves/CMakeFiles/Export/b58ecaf284d03b5477d284d0202b06d0/gz-waves1-rendering-ogre2-targets.cmake")
  if(CMAKE_INSTALL_CONFIG_NAME MATCHES "^([Rr][Ee][Ll][Ww][Ii][Tt][Hh][Dd][Ee][Bb][Ii][Nn][Ff][Oo])$")
    file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/cmake/gz-waves1-rendering-ogre2" TYPE FILE FILES "/home/lorenzo/autonomy-ws-25-26/src/asv_wave_sim/build/gz-waves1/src/systems/waves/CMakeFiles/Export/b58ecaf284d03b5477d284d0202b06d0/gz-waves1-rendering-ogre2-targets-relwithdebinfo.cmake")
  endif()
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "pkgconfig" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/lorenzo/autonomy-ws-25-26/src/asv_wave_sim/build/gz-waves1/cmake/pkgconfig/gz-waves1-rendering-ogre2.pc")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "headers" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/gz/waves1/gz/waves/waves-visual-system/detail" TYPE FILE FILES "/home/lorenzo/autonomy-ws-25-26/src/asv_wave_sim/build/gz-waves1/include/gz/waves/waves-visual-system/detail/Export.hh")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "headers" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/gz/waves1/gz/waves/waves-visual-system" TYPE FILE FILES "/home/lorenzo/autonomy-ws-25-26/src/asv_wave_sim/build/gz-waves1/include/gz/waves/waves-visual-system/Export.hh")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  foreach(file
      "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libgz-waves1-waves-visual-system.so.1.0.0"
      "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libgz-waves1-waves-visual-system.so.1"
      )
    if(EXISTS "${file}" AND
       NOT IS_SYMLINK "${file}")
      file(RPATH_CHECK
           FILE "${file}"
           RPATH "")
    endif()
  endforeach()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" TYPE SHARED_LIBRARY FILES
    "/home/lorenzo/autonomy-ws-25-26/src/asv_wave_sim/build/gz-waves1/lib/libgz-waves1-waves-visual-system.so.1.0.0"
    "/home/lorenzo/autonomy-ws-25-26/src/asv_wave_sim/build/gz-waves1/lib/libgz-waves1-waves-visual-system.so.1"
    )
  foreach(file
      "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libgz-waves1-waves-visual-system.so.1.0.0"
      "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libgz-waves1-waves-visual-system.so.1"
      )
    if(EXISTS "${file}" AND
       NOT IS_SYMLINK "${file}")
      file(RPATH_CHANGE
           FILE "${file}"
           OLD_RPATH "/usr/include/gz/rendering8/../../../lib/gz-rendering-8/engine-plugins:/home/lorenzo/autonomy-ws-25-26/src/asv_wave_sim/build/gz-waves1/lib:/opt/ros/jazzy/opt/gz_math_vendor/lib:/opt/ros/jazzy/opt/gz_utils_vendor/lib:"
           NEW_RPATH "")
      if(CMAKE_INSTALL_DO_STRIP)
        execute_process(COMMAND "/usr/bin/strip" "${file}")
      endif()
    endif()
  endforeach()
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" TYPE SHARED_LIBRARY FILES "/home/lorenzo/autonomy-ws-25-26/src/asv_wave_sim/build/gz-waves1/lib/libgz-waves1-waves-visual-system.so")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "cmake" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/cmake/gz-waves1-waves-visual-system" TYPE FILE FILES
    "/home/lorenzo/autonomy-ws-25-26/src/asv_wave_sim/build/gz-waves1/cmake/gz-waves1-waves-visual-system-config.cmake"
    "/home/lorenzo/autonomy-ws-25-26/src/asv_wave_sim/build/gz-waves1/cmake/gz-waves1-waves-visual-system-config-version.cmake"
    )
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/cmake/gz-waves1-waves-visual-system/gz-waves1-waves-visual-system-targets.cmake")
    file(DIFFERENT _cmake_export_file_changed FILES
         "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/cmake/gz-waves1-waves-visual-system/gz-waves1-waves-visual-system-targets.cmake"
         "/home/lorenzo/autonomy-ws-25-26/src/asv_wave_sim/build/gz-waves1/src/systems/waves/CMakeFiles/Export/eccb94b5fc81c051139c852a2c54fcf1/gz-waves1-waves-visual-system-targets.cmake")
    if(_cmake_export_file_changed)
      file(GLOB _cmake_old_config_files "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/cmake/gz-waves1-waves-visual-system/gz-waves1-waves-visual-system-targets-*.cmake")
      if(_cmake_old_config_files)
        string(REPLACE ";" ", " _cmake_old_config_files_text "${_cmake_old_config_files}")
        message(STATUS "Old export file \"$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/cmake/gz-waves1-waves-visual-system/gz-waves1-waves-visual-system-targets.cmake\" will be replaced.  Removing files [${_cmake_old_config_files_text}].")
        unset(_cmake_old_config_files_text)
        file(REMOVE ${_cmake_old_config_files})
      endif()
      unset(_cmake_old_config_files)
    endif()
    unset(_cmake_export_file_changed)
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/cmake/gz-waves1-waves-visual-system" TYPE FILE FILES "/home/lorenzo/autonomy-ws-25-26/src/asv_wave_sim/build/gz-waves1/src/systems/waves/CMakeFiles/Export/eccb94b5fc81c051139c852a2c54fcf1/gz-waves1-waves-visual-system-targets.cmake")
  if(CMAKE_INSTALL_CONFIG_NAME MATCHES "^([Rr][Ee][Ll][Ww][Ii][Tt][Hh][Dd][Ee][Bb][Ii][Nn][Ff][Oo])$")
    file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/cmake/gz-waves1-waves-visual-system" TYPE FILE FILES "/home/lorenzo/autonomy-ws-25-26/src/asv_wave_sim/build/gz-waves1/src/systems/waves/CMakeFiles/Export/eccb94b5fc81c051139c852a2c54fcf1/gz-waves1-waves-visual-system-targets-relwithdebinfo.cmake")
  endif()
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "pkgconfig" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/lorenzo/autonomy-ws-25-26/src/asv_wave_sim/build/gz-waves1/cmake/pkgconfig/gz-waves1-waves-visual-system.pc")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "headers" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/gz/waves1/gz/waves/waves-model-system/detail" TYPE FILE FILES "/home/lorenzo/autonomy-ws-25-26/src/asv_wave_sim/build/gz-waves1/include/gz/waves/waves-model-system/detail/Export.hh")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "headers" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/gz/waves1/gz/waves/waves-model-system" TYPE FILE FILES "/home/lorenzo/autonomy-ws-25-26/src/asv_wave_sim/build/gz-waves1/include/gz/waves/waves-model-system/Export.hh")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  foreach(file
      "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libgz-waves1-waves-model-system.so.1.0.0"
      "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libgz-waves1-waves-model-system.so.1"
      )
    if(EXISTS "${file}" AND
       NOT IS_SYMLINK "${file}")
      file(RPATH_CHECK
           FILE "${file}"
           RPATH "")
    endif()
  endforeach()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" TYPE SHARED_LIBRARY FILES
    "/home/lorenzo/autonomy-ws-25-26/src/asv_wave_sim/build/gz-waves1/lib/libgz-waves1-waves-model-system.so.1.0.0"
    "/home/lorenzo/autonomy-ws-25-26/src/asv_wave_sim/build/gz-waves1/lib/libgz-waves1-waves-model-system.so.1"
    )
  foreach(file
      "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libgz-waves1-waves-model-system.so.1.0.0"
      "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libgz-waves1-waves-model-system.so.1"
      )
    if(EXISTS "${file}" AND
       NOT IS_SYMLINK "${file}")
      file(RPATH_CHANGE
           FILE "${file}"
           OLD_RPATH "/usr/include/gz/rendering8/../../../lib/gz-rendering-8/engine-plugins:/home/lorenzo/autonomy-ws-25-26/src/asv_wave_sim/build/gz-waves1/lib:/opt/ros/jazzy/opt/gz_math_vendor/lib:/opt/ros/jazzy/opt/gz_utils_vendor/lib:"
           NEW_RPATH "")
      if(CMAKE_INSTALL_DO_STRIP)
        execute_process(COMMAND "/usr/bin/strip" "${file}")
      endif()
    endif()
  endforeach()
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" TYPE SHARED_LIBRARY FILES "/home/lorenzo/autonomy-ws-25-26/src/asv_wave_sim/build/gz-waves1/lib/libgz-waves1-waves-model-system.so")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "cmake" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/cmake/gz-waves1-waves-model-system" TYPE FILE FILES
    "/home/lorenzo/autonomy-ws-25-26/src/asv_wave_sim/build/gz-waves1/cmake/gz-waves1-waves-model-system-config.cmake"
    "/home/lorenzo/autonomy-ws-25-26/src/asv_wave_sim/build/gz-waves1/cmake/gz-waves1-waves-model-system-config-version.cmake"
    )
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/cmake/gz-waves1-waves-model-system/gz-waves1-waves-model-system-targets.cmake")
    file(DIFFERENT _cmake_export_file_changed FILES
         "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/cmake/gz-waves1-waves-model-system/gz-waves1-waves-model-system-targets.cmake"
         "/home/lorenzo/autonomy-ws-25-26/src/asv_wave_sim/build/gz-waves1/src/systems/waves/CMakeFiles/Export/86ce1808e3038ed9d8300af932f9483c/gz-waves1-waves-model-system-targets.cmake")
    if(_cmake_export_file_changed)
      file(GLOB _cmake_old_config_files "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/cmake/gz-waves1-waves-model-system/gz-waves1-waves-model-system-targets-*.cmake")
      if(_cmake_old_config_files)
        string(REPLACE ";" ", " _cmake_old_config_files_text "${_cmake_old_config_files}")
        message(STATUS "Old export file \"$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/cmake/gz-waves1-waves-model-system/gz-waves1-waves-model-system-targets.cmake\" will be replaced.  Removing files [${_cmake_old_config_files_text}].")
        unset(_cmake_old_config_files_text)
        file(REMOVE ${_cmake_old_config_files})
      endif()
      unset(_cmake_old_config_files)
    endif()
    unset(_cmake_export_file_changed)
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/cmake/gz-waves1-waves-model-system" TYPE FILE FILES "/home/lorenzo/autonomy-ws-25-26/src/asv_wave_sim/build/gz-waves1/src/systems/waves/CMakeFiles/Export/86ce1808e3038ed9d8300af932f9483c/gz-waves1-waves-model-system-targets.cmake")
  if(CMAKE_INSTALL_CONFIG_NAME MATCHES "^([Rr][Ee][Ll][Ww][Ii][Tt][Hh][Dd][Ee][Bb][Ii][Nn][Ff][Oo])$")
    file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/cmake/gz-waves1-waves-model-system" TYPE FILE FILES "/home/lorenzo/autonomy-ws-25-26/src/asv_wave_sim/build/gz-waves1/src/systems/waves/CMakeFiles/Export/86ce1808e3038ed9d8300af932f9483c/gz-waves1-waves-model-system-targets-relwithdebinfo.cmake")
  endif()
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "pkgconfig" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/lorenzo/autonomy-ws-25-26/src/asv_wave_sim/build/gz-waves1/cmake/pkgconfig/gz-waves1-waves-model-system.pc")
endif()

