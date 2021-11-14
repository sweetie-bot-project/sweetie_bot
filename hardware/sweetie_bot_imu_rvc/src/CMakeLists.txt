
#
# Components, types and plugins.
#
# The CMake 'target' names are identical to the first argument of the
# macros below, except for orocos_typegen_headers, where the target is fully
# controlled by generated code of 'typegen'.
#

# Creates a component library libbno080_rvc-<target>.so
# and installs in the directory lib/orocos/bno080_rvc/
#
orocos_component(imu_rvc imu_bno080_rvc.hpp imu_bno080_rvc.cpp) # ...you may add multiple source files
target_link_libraries(imu_rvc)
#
# You may add multiple orocos_component statements.

#
# Additional headers:
#
# Installs in the include/orocos/bno080_rvc/ directory
#
# orocos_install_headers(bno080_rvc.hpp) # ...you may add multiple header files
#
# You may add multiple orocos_install_headers statements.

