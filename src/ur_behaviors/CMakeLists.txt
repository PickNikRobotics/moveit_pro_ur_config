cmake_minimum_required(VERSION 3.22)
project(ur_behaviors)

find_package(ament_cmake REQUIRED)
find_package(moveit_pro_package REQUIRED)
find_package(pluginlib REQUIRED)
moveit_pro_package()

# add custom behaviors
add_subdirectory(behaviors)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

# Export the behavior plugins defined in this package so they are available to
# plugin loaders that load the behavior base class library from the
# moveit_pro_behavior package.
pluginlib_export_plugin_description_file(
        moveit_pro_behavior_interface behaviors/ur_behaviors_plugin_description.xml)

ament_package()
