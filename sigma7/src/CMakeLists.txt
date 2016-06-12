
#
# Components, types and plugins.
#
# The CMake 'target' names are identical to the first argument of the
# macros below, except for orocos_typegen_headers, where the target is fully
# controlled by generated code of 'typegen'.
#

# Creates a component library libSigma7-<target>.so
# and installs in the directory lib/orocos/Sigma7/
#
orocos_component(Sigma7 Sigma7-component.hpp Sigma7-component.cpp) # ...you may add multiple source files
#
# You may add multiple orocos_component statements.

#
# Additional headers:
#
# Installs in the include/orocos/Sigma7/ directory
#
orocos_install_headers(Sigma7-component.hpp) # ...you may add multiple header files
#
# You may add multiple orocos_install_headers statements.

#
# Building a Plugin
#
# Creates a plugin library libSigma7-plugin-<target>.so
# and installs in the directory lib/orocos/Sigma7/plugins/
#
# Be aware that a plugin may only have the loadRTTPlugin() function once defined in a .cpp file.
# This function is defined by the plugin and service CPP macros.
#
#orocos_plugin(Sigma7-plugin Sigma7-plugin.cpp) # ...only one plugin function per library !
#
# You may add multiple orocos_plugin statements.


#
# Building a Service:
#
# Creates a plugin library libSigma7-service-<target>.so
# and installs in the directory lib/orocos/Sigma7/plugins/
#
#orocos_service(Sigma7-service Sigma7-service.cpp) # ...only one service per library !
#
# You may add multiple orocos_service statements.

#
# Building a typekit using typegen (recommended):
#
# Creates a typekit library libSigma7-types-<target>.so
# and installs in the directory lib/orocos/@target@/Sigma7/types/
#
# The header will go in include/orocos/Sigma7/types/Sigma7/Sigma7-types.hpp
# So you can #include <Sigma7/Sigma7-types.hpp>
#
#orocos_typegen_headers(include/Sigma7/Sigma7-types.hpp) # ...you may add multiple header files
#
# You may only have *ONE* orocos_typegen_headers statement in your toplevel CMakeFile.txt !


#
# Building a normal library (optional):
#
# Creates a library libsupport-<target>.so and installs it in
# lib/
#
#orocos_library(support support.cpp) # ...you may add multiple source files
#
# You may add multiple orocos_library statements.


target_link_libraries(Sigma7 /usr/lib/x86_64-linux-gnu/libdhd.so.3.5.0 /usr/lib/x86_64-linux-gnu/libdhd.a /usr/lib/x86_64-linux-gnu/libusb-1.0)

#	/home/nearlab/Sigma/Linux/x86_64/sdk-3.5.0/lib/release/lin-x86_64-gcc/libdhd.so.3.5.0
#	/home/nearlab/Sigma/Linux/x86_64/sdk-3.5.0/lib/release/lin-x86_64-gcc/libdrd.so.3.5.0
#	/home/nearlab/Sigma/Linux/x86_64/sdk-3.5.0/lib/release/lin-x86_64-gcc/libdhd.a
#	/home/nearlab/Sigma/Linux/x86_64/sdk-3.5.0/lib/release/lin-x86_64-gcc/libdrd.a
#)
