cmake_minimum_required(VERSION 3.22)
project(compute_place_orientation CXX)

find_package(moveit_pro_package REQUIRED)
moveit_pro_package()

set(THIS_PACKAGE_INCLUDE_DEPENDS
  geometry_msgs
  moveit_pro_behavior_interface
  pluginlib
  tf2_eigen
)
foreach(package IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${package} REQUIRED)
endforeach()

find_package(spdlog REQUIRED)

add_library(compute_place_orientation SHARED
  src/compute_place_orientation.cpp
  src/register_behaviors.cpp
)
target_include_directories(compute_place_orientation
  PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
         $<INSTALL_INTERFACE:include>)
ament_target_dependencies(compute_place_orientation ${THIS_PACKAGE_INCLUDE_DEPENDS})
target_link_libraries(compute_place_orientation spdlog::spdlog)

install(
  TARGETS compute_place_orientation
  EXPORT compute_place_orientationTargets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include)

if(BUILD_TESTING)
  add_subdirectory(test)
endif()

pluginlib_export_plugin_description_file(
  moveit_pro_behavior_interface compute_place_orientation_plugin_description.xml)

ament_export_targets(compute_place_orientationTargets HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()
