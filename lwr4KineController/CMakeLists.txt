#
# The find_package macro for Orocos-RTT works best with
# cmake >= 2.6.3
#
cmake_minimum_required(VERSION 2.6.3)

#
# This creates a standard cmake project. You may extend this file with
# any cmake macro you see fit.
#
project(lwr4KineController)
#
# Do setup in case of ros package, If ROS_ROOT is set, it is
# recommended to use RTT/OCL through the ros packages.
#
set (ROS_ROOT $ENV{ROS_ROOT} )
set (ROS_STACK ${ROS_ROOT}/../../stacks/)
#set (ROS_WORKSPACE $ENV{ROS_WORKSPACE} )
if (ROS_ROOT)
  include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
  rosbuild_init()
  rosbuild_find_ros_package( rtt )
  set( RTT_HINTS HINTS ${rtt_PACKAGE_PATH}/install )
  if (ROS_ROOT STREQUAL /opt/ros/electric/ros)
    set (ROS_STACK ${ROS_ROOT}/../stacks/)
  endif()
endif()

# Set the CMAKE_PREFIX_PATH in case you're not using Orocos through ROS
# for helping these find commands find RTT.
find_package(OROCOS-RTT REQUIRED ${RTT_HINTS})

# Defines the orocos_* cmake macros. See that file for additional
# documentation.
include(${OROCOS-RTT_USE_FILE_PATH}/UseOROCOS-RTT.cmake)

include_directories(
	${ROS_STACK}
)

find_path(RTNET_INCLUDE_DIR NAMES rtnet.h PATH_SUFFIXES rtnet/include)
if (RTNET_INCLUDE_DIR AND ${OROCOS_TARGET} STREQUAL "xenomai")
	message("Found RTNET: ${RTNET_INCLUDE_DIR}")
	include_directories(${RTNET_INCLUDE_DIR})
	add_definitions(-DHAVE_RTNET)
	# try Xenomai
	execute_process(COMMAND xeno-config --skin=posix --cflags OUTPUT_VARIABLE XENO_CFLAGS OUTPUT_STRIP_TRAILING_WHITESPACE)
	execute_process(COMMAND xeno-config --skin=posix --ldflags OUTPUT_VARIABLE XENO_LDFLAGS OUTPUT_STRIP_TRAILING_WHITESPACE)
	set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fPIC ${XENO_CFLAGS}")
	set(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} ${XENO_LDFLAGS}")
endif()



# Defines the orocos_* cmake macros. See that file for additional
# documentation.
include( ${OROCOS-RTT_USE_FILE_PATH}/UseOROCOS-RTT.cmake )

# Rest of the configuration is in src/
# add_subdirectory( src )

# Creates a component library liblwr4KineController-<target>.so
# and installs in the directory lib/orocos/lwr4KineController/
#
orocos_component(lwr4KineController src/lwr4KineController-component.hpp src/lwr4KineController-component.cpp) # ...you may add multiple source files
#
# You may add multiple orocos_component statements.
#
# Additional headers:
#
# Installs in the include/orocos/lwr4KineController/ directory
orocos_install_headers(src/lwr4KineController-component.hpp) # ...you may add multiple header files
#
# You may add multiple orocos_install_headers statements.

if (RTNET_INCLUDE_DIR AND ${OROCOS_TARGET} STREQUAL "xenomai")
	orocos_add_compile_flags(lwr4KineController ${XENO_CFLAGS})
	orocos_add_link_flags(lwr4KineController ${XENO_LDFLAGS})
endif()

add_library(toolbox 		SHARED 	src/MyAPI/toolbox.cpp src/MyAPI/toolbox.hpp)
add_library(LWR4_Kinematics 	SHARED 	src/MyAPI/LWR4_Kinematics.cpp src/MyAPI/LWR4_Kinematics.hpp)
add_library(FOAW 		SHARED 	src/MyAPI/filter.cpp src/MyAPI/filter.h)

target_link_libraries(toolbox libtf_conversions.so)
target_link_libraries(lwr4KineController toolbox LWR4_Kinematics FOAW)
#
# Generates and installs our package. Must be the last statement such
# that it can pick up all above settings.
#
orocos_generate_package()
