cmake_minimum_required(VERSION 3.10.2 FATAL_ERROR)

#============================================================================
# Initialize the project
#============================================================================
project(ignition-gazebo5 VERSION 5.0.0)

#============================================================================
# Find ignition-cmake
#============================================================================
# If you get an error at this line, you need to install ignition-cmake
find_package(ignition-cmake2 2.3 REQUIRED)

#============================================================================
# Configure the project
#============================================================================
ign_configure_project(VERSION_SUFFIX)

#============================================================================
# Set project-specific options
#============================================================================

option(ENABLE_PROFILER "Enable Ignition Profiler" FALSE)

if(ENABLE_PROFILER)
  add_definitions("-DIGN_PROFILER_ENABLE=1")
else()
  add_definitions("-DIGN_PROFILER_ENABLE=0")
endif()

if (UNIX AND NOT APPLE)
  set (EXTRA_TEST_LIB_DEPS stdc++fs)
else()
  set (EXTRA_TEST_LIB_DEPS)
endif()

#============================================================================
# Search for project-specific dependencies
#============================================================================

ign_find_package(sdformat11 REQUIRED VERSION 11.1)
set(SDF_VER ${sdformat11_VERSION_MAJOR})

#--------------------------------------
# Find ignition-plugin
ign_find_package(ignition-plugin1 REQUIRED COMPONENTS loader register)
set(IGN_PLUGIN_VER ${ignition-plugin1_VERSION_MAJOR})

#--------------------------------------
# Find ignition-transport
ign_find_package(ignition-transport10 REQUIRED COMPONENTS log)
set(IGN_TRANSPORT_VER ${ignition-transport10_VERSION_MAJOR})

#--------------------------------------
# Find ignition-msgs
ign_find_package(ignition-msgs7 REQUIRED VERSION 7.1)
set(IGN_MSGS_VER ${ignition-msgs7_VERSION_MAJOR})

#--------------------------------------
# Find ignition-common
# Always use the profiler component to get the headers, regardless of status.
ign_find_package(ignition-common4
  COMPONENTS
    profiler
    events
    av
  REQUIRED
)
set(IGN_COMMON_VER ${ignition-common4_VERSION_MAJOR})

#--------------------------------------
# Find ignition-fuel_tools
ign_find_package(ignition-fuel_tools6 REQUIRED)
set(IGN_FUEL_TOOLS_VER ${ignition-fuel_tools6_VERSION_MAJOR})

#--------------------------------------
# Find ignition-gui
ign_find_package(ignition-gui5 REQUIRED)
set(IGN_GUI_VER ${ignition-gui5_VERSION_MAJOR})
ign_find_package (Qt5
  COMPONENTS
    Core
    Quick
    QuickControls2
  REQUIRED
  PKGCONFIG "Qt5Core Qt5Quick Qt5QuickControls2")

#--------------------------------------
# Find ignition-physics
ign_find_package(ignition-physics4
  COMPONENTS
    mesh
    sdf
  REQUIRED
)
set(IGN_PHYSICS_VER ${ignition-physics4_VERSION_MAJOR})

#--------------------------------------
# Find ignition-sensors
ign_find_package(ignition-sensors5 REQUIRED
  COMPONENTS
    rendering
    air_pressure
    altimeter
    camera
    gpu_lidar
    imu
    logical_camera
    magnetometer
    depth_camera
    thermal_camera
)
set(IGN_SENSORS_VER ${ignition-sensors5_VERSION_MAJOR})

#--------------------------------------
# Find ignition-rendering
ign_find_package(ignition-rendering5 REQUIRED)
set(IGN_RENDERING_VER ${ignition-rendering5_VERSION_MAJOR})

#--------------------------------------
# Find ignition-math
ign_find_package(ignition-math6 REQUIRED COMPONENTS eigen3 VERSION 6.8)
set(IGN_MATH_VER ${ignition-math6_VERSION_MAJOR})

#--------------------------------------
# Find ignition-tools
ign_find_package(ignition-tools
                 REQUIRED
                 PKGCONFIG "ignition-tools")

#--------------------------------------
# Find ignition-utils
ign_find_package(ignition-utils1 REQUIRED COMPONENTS cli)
set(IGN_UTILS_VER ${ignition-utils1_VERSION_MAJOR})

#--------------------------------------
# Find protobuf
# Module is needed to use the PROTOBUF_GENERATE_CPP
set(protobuf_MODULE_COMPATIBLE TRUE)
set(REQ_PROTOBUF_VER 3)
ign_find_package(IgnProtobuf
                 VERSION ${REQ_PROTOBUF_VER}
                 REQUIRED
                 COMPONENTS all
                 PRETTY Protobuf)
set(Protobuf_IMPORT_DIRS ${ignition-msgs7_INCLUDE_DIRS})

# Plugin install dirs
set(IGNITION_GAZEBO_PLUGIN_INSTALL_DIR
  ${CMAKE_INSTALL_PREFIX}/${IGN_LIB_INSTALL_DIR}/ign-${IGN_DESIGNATION}-${PROJECT_VERSION_MAJOR}/plugins
)
set(IGNITION_GAZEBO_GUI_PLUGIN_INSTALL_DIR
  ${CMAKE_INSTALL_PREFIX}/${IGN_LIB_INSTALL_DIR}/ign-${IGN_DESIGNATION}-${PROJECT_VERSION_MAJOR}/plugins/gui
)

#============================================================================
# Configure the build
#============================================================================
ign_configure_build(QUIT_IF_BUILD_ERRORS)

add_subdirectory(examples)

#============================================================================
# Create package information
#============================================================================
ign_create_packages()

#============================================================================
# Configure documentation
#============================================================================
configure_file(${CMAKE_SOURCE_DIR}/api.md.in ${CMAKE_BINARY_DIR}/api.md)
configure_file(${CMAKE_SOURCE_DIR}/tutorials.md.in ${CMAKE_BINARY_DIR}/tutorials.md)

# disable doxygen on macOS due to issues with doxygen 1.9.0
# there is an unreleased fix; revert this when 1.9.1 is released
# https://github.com/ignitionrobotics/ign-gazebo/issues/520
if (NOT APPLE)
  ign_create_docs(
    API_MAINPAGE_MD "${CMAKE_BINARY_DIR}/api.md"
    TUTORIALS_MAINPAGE_MD "${CMAKE_BINARY_DIR}/tutorials.md"
    ADDITIONAL_INPUT_DIRS "${CMAKE_SOURCE_DIR}/src/systems ${CMAKE_SOURCE_DIR}/src/gui/plugins"
    TAGFILES
     "${IGNITION-MATH_DOXYGEN_TAGFILE} = ${IGNITION-MATH_API_URL}"
     "${IGNITION-MSGS_DOXYGEN_TAGFILE} = ${IGNITION-MSGS_API_URL}"
     "${IGNITION-PHYSICS_DOXYGEN_TAGFILE} = ${IGNITION-PHYSICS_API_URL}"
     "${IGNITION-PLUGIN_DOXYGEN_TAGFILE} = ${IGNITION-PLUGIN_API_URL}"
     "${IGNITION-TRANSPORT_DOXYGEN_TAGFILE} = ${IGNITION-TRANSPORT_API_URL}"
     "${IGNITION-SENSORS_DOXYGEN_TAGFILE} = ${IGNITION-SENSORS_API_URL}"
     "${IGNITION-COMMON_DOXYGEN_TAGFILE} = ${IGNITION-COMMON_API_URL}"
  )
endif()

if(TARGET doc)
  file(COPY ${CMAKE_SOURCE_DIR}/tutorials/files/ DESTINATION ${CMAKE_BINARY_DIR}/doxygen/html/files/)
endif()
