
#
# Components, types and plugins.
#
# The CMake 'target' names are identical to the first argument of the
# macros below, except for orocos_typegen_headers, where the target is fully
# controlled by generated code of 'typegen'.
#

# Creates a component library libacForceGen-<target>.so
# and installs in the directory lib/orocos/acForceGen/
#
orocos_component(acForceGen acForceGen-component.hpp acForceGen-component.cpp) # ...you may add multiple source files
#
# You may add multiple orocos_component statements.

#
# Additional headers:
#
# Installs in the include/orocos/acForceGen/ directory
#
orocos_install_headers(acForceGen-component.hpp) # ...you may add multiple header files
#
# You may add multiple orocos_install_headers statements.

#
# Building a Plugin
#
# Creates a plugin library libacForceGen-plugin-<target>.so
# and installs in the directory lib/orocos/acForceGen/plugins/
#
# Be aware that a plugin may only have the loadRTTPlugin() function once defined in a .cpp file.
# This function is defined by the plugin and service CPP macros.
#
orocos_plugin(acForceGen-plugin acForceGen-plugin.cpp) # ...only one plugin function per library !
#
# You may add multiple orocos_plugin statements.


#
# Building a Service:
#
# Creates a plugin library libacForceGen-service-<target>.so
# and installs in the directory lib/orocos/acForceGen/plugins/
#
orocos_service(acForceGen-service acForceGen-service.cpp) # ...only one service per library !
#
# You may add multiple orocos_service statements.


#
# Building a typekit using typegen (recommended):
#
# Creates a typekit library libacForceGen-types-<target>.so
# and installs in the directory lib/orocos/@target@/acForceGen/types/
#
# The header will go in include/orocos/acForceGen/types/acForceGen/acForceGen-types.hpp
# So you can #include <acForceGen/acForceGen-types.hpp>
#
orocos_typegen_headers(include/acForceGen/acForceGen-types.hpp) # ...you may add multiple header files
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


