cmake_minimum_required(VERSION 3.22)
project(get_graspable_grocery_objects_from_masks3d CXX)

find_package(moveit_pro_package REQUIRED)
moveit_pro_package()

set(THIS_PACKAGE_INCLUDE_DEPENDS
  moveit_pro_behavior_interface
  moveit_studio_vision_msgs
  pcl_conversions
  pcl_ros
  sensor_msgs
  shape_msgs
  tf2_eigen
  tf2_ros
  visualization_msgs
  pluginlib
)
foreach(package IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${package} REQUIRED)
endforeach()

find_package(PCL REQUIRED COMPONENTS common segmentation search features)
find_package(spdlog REQUIRED)

add_library(
  get_graspable_grocery_objects_from_masks3d
  SHARED
  src/shape_fitting.cpp
  src/compute_grocery_place_height.cpp
  src/get_graspable_grocery_objects_from_masks3d.cpp
  src/register_behaviors.cpp)
target_include_directories(
  get_graspable_grocery_objects_from_masks3d
  PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
         $<INSTALL_INTERFACE:include>)
ament_target_dependencies(get_graspable_grocery_objects_from_masks3d
                          ${THIS_PACKAGE_INCLUDE_DEPENDS})
target_link_libraries(get_graspable_grocery_objects_from_masks3d
  ${PCL_LIBRARIES}
  spdlog::spdlog)
target_include_directories(get_graspable_grocery_objects_from_masks3d
  PRIVATE ${PCL_INCLUDE_DIRS})

# Install Libraries
install(
  TARGETS get_graspable_grocery_objects_from_masks3d
  EXPORT get_graspable_grocery_objects_from_masks3dTargets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES
  DESTINATION include)

if(BUILD_TESTING)
  moveit_pro_behavior_test(get_graspable_grocery_objects_from_masks3d)
endif()

# Export the behavior plugins defined in this package so they are available to
# plugin loaders that load the behavior base class library from the
# moveit_pro_behavior package.
pluginlib_export_plugin_description_file(
  moveit_pro_behavior_interface get_graspable_grocery_objects_from_masks3d_plugin_description.xml)

ament_export_targets(get_graspable_grocery_objects_from_masks3dTargets HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()
