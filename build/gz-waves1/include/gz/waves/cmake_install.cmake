# Install script for directory: /home/ethan/autonomy-ws-25-26/src/asv_wave_sim/gz-waves/include/gz/waves

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/ethan/autonomy-ws-25-26/src/asv_wave_sim/install")
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

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xheadersx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/gz/waves1/gz/waves/components" TYPE FILE FILES "/home/ethan/autonomy-ws-25-26/src/asv_wave_sim/gz-waves/include/gz/waves/components/Wavefield.hh")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xheadersx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/gz/waves1/gz/waves" TYPE FILE FILES
    "/home/ethan/autonomy-ws-25-26/src/asv_wave_sim/gz-waves/include/gz/waves/Algorithm.hh"
    "/home/ethan/autonomy-ws-25-26/src/asv_wave_sim/gz-waves/include/gz/waves/CGALTypes.hh"
    "/home/ethan/autonomy-ws-25-26/src/asv_wave_sim/gz-waves/include/gz/waves/Convert.hh"
    "/home/ethan/autonomy-ws-25-26/src/asv_wave_sim/gz-waves/include/gz/waves/Geometry.hh"
    "/home/ethan/autonomy-ws-25-26/src/asv_wave_sim/gz-waves/include/gz/waves/Grid.hh"
    "/home/ethan/autonomy-ws-25-26/src/asv_wave_sim/gz-waves/include/gz/waves/LinearRandomFFTWaveSimulation.hh"
    "/home/ethan/autonomy-ws-25-26/src/asv_wave_sim/gz-waves/include/gz/waves/LinearRandomWaveSimulation.hh"
    "/home/ethan/autonomy-ws-25-26/src/asv_wave_sim/gz-waves/include/gz/waves/LinearRegularWaveSimulation.hh"
    "/home/ethan/autonomy-ws-25-26/src/asv_wave_sim/gz-waves/include/gz/waves/MeshTools.hh"
    "/home/ethan/autonomy-ws-25-26/src/asv_wave_sim/gz-waves/include/gz/waves/OceanTile.hh"
    "/home/ethan/autonomy-ws-25-26/src/asv_wave_sim/gz-waves/include/gz/waves/PhysicalConstants.hh"
    "/home/ethan/autonomy-ws-25-26/src/asv_wave_sim/gz-waves/include/gz/waves/Physics.hh"
    "/home/ethan/autonomy-ws-25-26/src/asv_wave_sim/gz-waves/include/gz/waves/TriangulatedGrid.hh"
    "/home/ethan/autonomy-ws-25-26/src/asv_wave_sim/gz-waves/include/gz/waves/TrochoidIrregularWaveSimulation.hh"
    "/home/ethan/autonomy-ws-25-26/src/asv_wave_sim/gz-waves/include/gz/waves/Types.hh"
    "/home/ethan/autonomy-ws-25-26/src/asv_wave_sim/gz-waves/include/gz/waves/Utilities.hh"
    "/home/ethan/autonomy-ws-25-26/src/asv_wave_sim/gz-waves/include/gz/waves/WaveParameters.hh"
    "/home/ethan/autonomy-ws-25-26/src/asv_wave_sim/gz-waves/include/gz/waves/WaveSimulation.hh"
    "/home/ethan/autonomy-ws-25-26/src/asv_wave_sim/gz-waves/include/gz/waves/WaveSpectrum.hh"
    "/home/ethan/autonomy-ws-25-26/src/asv_wave_sim/gz-waves/include/gz/waves/WaveSpreadingFunction.hh"
    "/home/ethan/autonomy-ws-25-26/src/asv_wave_sim/gz-waves/include/gz/waves/Wavefield.hh"
    "/home/ethan/autonomy-ws-25-26/src/asv_wave_sim/gz-waves/include/gz/waves/WavefieldSampler.hh"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xheadersx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/gz/waves1/gz" TYPE FILE FILES "/home/ethan/autonomy-ws-25-26/src/asv_wave_sim/build/gz-waves1/include/gz/waves/../waves.hh")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xheadersx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/gz/waves1/gz/waves" TYPE FILE FILES "/home/ethan/autonomy-ws-25-26/src/asv_wave_sim/build/gz-waves1/include/gz/waves/config.hh")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for each subdirectory.
  include("/home/ethan/autonomy-ws-25-26/src/asv_wave_sim/build/gz-waves1/include/gz/waves/components/cmake_install.cmake")

endif()

