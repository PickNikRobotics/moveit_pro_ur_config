cmake_minimum_required(VERSION 3.22)
project(ur_sim)

find_package(ament_cmake REQUIRED)
find_package(moveit_studio_common REQUIRED)
moveit_studio_package()

# add custom behaviors
# add_subdirectory(behaviors)

install(
  DIRECTORY
    config
    description
    launch
  DESTINATION
    share/${PROJECT_NAME}
)

# Install all XML files in directory
set(PICKNIK_ACCESSORIES_SHARE_DIR
"${CMAKE_INSTALL_PREFIX}/../picknik_accessories/share/picknik_accessories/mujoco_assets/"
)
# Destination directory
set(DEST_DIR "${CMAKE_INSTALL_PREFIX}/share/${PROJECT_NAME}/description/")

install(DIRECTORY "${PICKNIK_ACCESSORIES_SHARE_DIR}"
        DESTINATION "${DEST_DIR}"
        FILES_MATCHING PATTERN "*")

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

# Export the behavior plugins defined in this package so they are available to
# plugin loaders that load the behavior base class library from the
# moveit_pro_behavior package.
# pluginlib_export_plugin_description_file(
#         moveit_pro_behavior_interface)

ament_package()
