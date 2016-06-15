
#
# Components, types and plugins.
#
# The CMake 'target' names are identical to the first argument of the
# macros below, except for orocos_typegen_headers, where the target is fully
# controlled by generated code of 'typegen'.
#

# Creates a component library libacGeomGen-<target>.so
# and installs in the directory lib/orocos/acGeomGen/
#
orocos_component(acGeomGen acGeomGen-component.hpp acGeomGen-component.cpp) # ...you may add multiple source files
#
# You may add multiple orocos_component statements.

#
# Additional headers:
#
# Installs in the include/orocos/acGeomGen/ directory
#
orocos_install_headers(acGeomGen-component.hpp) # ...you may add multiple header files
#
# You may add multiple orocos_install_headers statements.

#
# Building a Plugin
#
# Creates a plugin library libacGeomGen-plugin-<target>.so
# and installs in the directory lib/orocos/acGeomGen/plugins/
#
# Be aware that a plugin may only have the loadRTTPlugin() function once defined in a .cpp file.
# This function is defined by the plugin and service CPP macros.
#
orocos_plugin(acGeomGen-plugin acGeomGen-plugin.cpp) # ...only one plugin function per library !
#
# You may add multiple orocos_plugin statements.


#
# Building a Service:
#
# Creates a plugin library libacGeomGen-service-<target>.so
# and installs in the directory lib/orocos/acGeomGen/plugins/
#
orocos_service(acGeomGen-service acGeomGen-service.cpp) # ...only one service per library !
#
# You may add multiple orocos_service statements.


#
# Building a typekit using typegen (recommended):
#
# Creates a typekit library libacGeomGen-types-<target>.so
# and installs in the directory lib/orocos/@target@/acGeomGen/types/
#
# The header will go in include/orocos/acGeomGen/types/acGeomGen/acGeomGen-types.hpp
# So you can #include <acGeomGen/acGeomGen-types.hpp>
#
orocos_typegen_headers(include/acGeomGen/acGeomGen-types.hpp) # ...you may add multiple header files
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


