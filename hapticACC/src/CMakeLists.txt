
#
# Components, types and plugins.
#
# The CMake 'target' names are identical to the first argument of the
# macros below, except for orocos_typegen_headers, where the target is fully
# controlled by generated code of 'typegen'.
#

# Creates a component library libhapticACC-<target>.so
# and installs in the directory lib/orocos/hapticACC/
#
orocos_component(hapticACC hapticACC-component.hpp hapticACC-component.cpp) # ...you may add multiple source files
#
# You may add multiple orocos_component statements.

#
# Additional headers:
#
# Installs in the include/orocos/hapticACC/ directory
#
orocos_install_headers(hapticACC-component.hpp) # ...you may add multiple header files
#
# You may add multiple orocos_install_headers statements.

#
# Building a Plugin
#
# Creates a plugin library libhapticACC-plugin-<target>.so
# and installs in the directory lib/orocos/hapticACC/plugins/
#
# Be aware that a plugin may only have the loadRTTPlugin() function once defined in a .cpp file.
# This function is defined by the plugin and service CPP macros.
#
orocos_plugin(hapticACC-plugin hapticACC-plugin.cpp) # ...only one plugin function per library !
#
# You may add multiple orocos_plugin statements.


#
# Building a Service:
#
# Creates a plugin library libhapticACC-service-<target>.so
# and installs in the directory lib/orocos/hapticACC/plugins/
#
orocos_service(hapticACC-service hapticACC-service.cpp) # ...only one service per library !
#
# You may add multiple orocos_service statements.


#
# Building a typekit using typegen (recommended):
#
# Creates a typekit library libhapticACC-types-<target>.so
# and installs in the directory lib/orocos/@target@/hapticACC/types/
#
# The header will go in include/orocos/hapticACC/types/hapticACC/hapticACC-types.hpp
# So you can #include <hapticACC/hapticACC-types.hpp>
#
orocos_typegen_headers(include/hapticACC/hapticACC-types.hpp) # ...you may add multiple header files
#
# You may only have *ONE* orocos_typegen_headers statement in your toplevel CMakeFile.txt !


#
# Building a normal library (optional):
#
# Creates a library libsupport-<target>.so and installs it in
# lib/
#
orocos_library(support support.cpp) # ...you may add multiple source files
#
# You may add multiple orocos_library statements.


