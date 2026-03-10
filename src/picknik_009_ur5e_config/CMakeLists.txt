cmake_minimum_required(VERSION 3.22)
project(picknik_009_ur5e_config)

find_package(ament_cmake REQUIRED)
find_package(picknik_accessories REQUIRED)

# Copy MuJoCo assets from picknik_accessories into our description directory
# so scene.xml can reference them with relative paths (e.g., ur5e/ur5e_globals.xml)
set(PICKNIK_ACCESSORIES_SHARE_DIR
  "${CMAKE_INSTALL_PREFIX}/../picknik_accessories/share/picknik_accessories/mujoco_assets/"
)
set(DEST_DIR "${CMAKE_INSTALL_PREFIX}/share/${PROJECT_NAME}/description/mujoco/")

install(DIRECTORY "${PICKNIK_ACCESSORIES_SHARE_DIR}"
        DESTINATION "${DEST_DIR}"
        FILES_MATCHING PATTERN "*")

# Also copy the workcell mesh into the mujoco directory for the scene
install(FILES meshes/workcell_no_totes.stl
        DESTINATION "share/${PROJECT_NAME}/description/mujoco/")

install(
  DIRECTORY
    config
    description
    meshes
    launch
  DESTINATION
    share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
