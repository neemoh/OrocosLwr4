#
# The find_package macro for Orocos-RTT works best with
# cmake >= 2.6.3
#
cmake_minimum_required(VERSION 2.6.3)

#
# This creates a standard cmake project. You may extend this file with
# any cmake macro you see fit.
#
project(acGeomGen)
#
# Do setup in case of ros package, If ROS_ROOT is set, it is
# recommended to use RTT/OCL through the ros packages.
#
set (ROS_ROOT $ENV{ROS_ROOT} )
if (ROS_ROOT)
  include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
  rosbuild_init()
  rosbuild_find_ros_package( rtt )
  set( RTT_HINTS HINTS ${rtt_PACKAGE_PATH}/install )
endif()

rosbuild_genmsg()
rosbuild_include(rtt_rosnode GenerateRTTtypekit )
ros_generate_rtt_typekit(acGeomGen)

# Use 'source orocos_toolchain/env.sh' if the command below fails:
find_package(OROCOS-RTT REQUIRED ${RTT_HINTS})

if (NOT OROCOS-RTT_FOUND)
  message (FATAL_ERROR "\nCould not find Orocos. Please use the shell command\n 'source orocos_toolchain/env.sh' and then run cmake again.")
endif()

# Defines the orocos_* cmake macros. See that file for additional
# documentation.
include( ${OROCOS-RTT_USE_FILE_PATH}/UseOROCOS-RTT.cmake )


#common commands for building c++ executables and libraries
find_path(RTNET_INCLUDE_DIR NAMES rtnet.h PATH_SUFFIXES rtnet/include)
if(RTNET_INCLUDE_DIR AND ${OROCOS_TARGET} STREQUAL "xenomai" )
  message("Found RTNET: ${RTNET_INCLUDE_DIR}")
  include_directories(${RTNET_INCLUDE_DIR})
  add_definitions(-DHAVE_RTNET)
  # try Xenomai
  execute_process(COMMAND xeno-config --skin=posix --cflags OUTPUT_VARIABLE XENO_CFLAGS OUTPUT_STRIP_TRAILING_WHITESPACE )
  execute_process(COMMAND xeno-config --skin=posix --ldflags OUTPUT_VARIABLE XENO_LDFLAGS OUTPUT_STRIP_TRAILING_WHITESPACE)


  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${XENO_CFLAGS} ")
  set(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} ${XENO_LDFLAGS}")
  orocos_component(acGeomGen src/acGeomGen-component.hpp src/acGeomGen-component.cpp) # ...you may add multiple source files
  orocos_add_compile_flags(acGeomGen ${XENO_CFLAGS})
  orocos_add_link_flags(acGeomGen ${XENO_LDFLAGS})
else()
  orocos_component(acGeomGen src/acGeomGen-component.hpp src/acGeomGen-component.cpp) # ...you may add multiple source files
endif()


# Rest of the configuration is in src/
#add_subdirectory( src )

#
# Generates and installs our package. Must be the last statement such
# that it can pick up all above settings.
#
orocos_generate_package()
