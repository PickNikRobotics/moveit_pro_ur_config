cmake_minimum_required(VERSION 3.22)
project(generate_shape_urdf CXX)

find_package(moveit_pro_package REQUIRED)
moveit_pro_package()

set(THIS_PACKAGE_INCLUDE_DEPENDS geometry_msgs moveit_pro_behavior_interface tf2_eigen pluginlib fmt)
foreach(package IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${package} REQUIRED)
endforeach()

add_library(
  generate_shape_urdf
  SHARED
  src/generate_shape_urdf.cpp
  src/register_behaviors.cpp)
target_include_directories(
  generate_shape_urdf
  PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
         $<INSTALL_INTERFACE:include>)
ament_target_dependencies(generate_shape_urdf
                          ${THIS_PACKAGE_INCLUDE_DEPENDS})

# Install Libraries
install(
  TARGETS generate_shape_urdf
  EXPORT generate_shape_urdfTargets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES
  DESTINATION include)

if(BUILD_TESTING)
  moveit_pro_behavior_test(generate_shape_urdf)
endif()

# Export the behavior plugins defined in this package so they are available to
# plugin loaders that load the behavior base class library from the
# moveit_pro_behavior package.
pluginlib_export_plugin_description_file(
  moveit_pro_behavior_interface generate_shape_urdf_plugin_description.xml)

ament_export_targets(generate_shape_urdfTargets HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()
