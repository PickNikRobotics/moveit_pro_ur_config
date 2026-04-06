cmake_minimum_required(VERSION 3.22)
project(picknik_009_ur5e_behaviors CXX)

find_package(moveit_pro_package REQUIRED)
moveit_pro_package()

set(THIS_PACKAGE_INCLUDE_DEPENDS geometry_msgs moveit_pro_behavior_interface pluginlib shape_msgs)
foreach(package IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${package} REQUIRED)
endforeach()

find_package(spdlog REQUIRED)

add_library(
  picknik_009_ur5e_behaviors
  SHARED
  src/create_solid_primitive_from_shape.cpp
  src/register_behaviors.cpp)
target_include_directories(
  picknik_009_ur5e_behaviors
  PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
         $<INSTALL_INTERFACE:include>)
ament_target_dependencies(picknik_009_ur5e_behaviors
                          ${THIS_PACKAGE_INCLUDE_DEPENDS})
target_link_libraries(picknik_009_ur5e_behaviors spdlog::spdlog)

install(
  TARGETS picknik_009_ur5e_behaviors
  EXPORT picknik_009_ur5e_behaviorsTargets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES
  DESTINATION include)

pluginlib_export_plugin_description_file(
  moveit_pro_behavior_interface picknik_009_ur5e_behaviors_plugin_description.xml)

ament_export_targets(picknik_009_ur5e_behaviorsTargets HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()