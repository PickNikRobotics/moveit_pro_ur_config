set(THIS_PACKAGE_INCLUDE_DEPENDS
  moveit_pro_behavior_interface
  pluginlib
  tf2_eigen
  visualization_msgs
  geometry_msgs
  moveit_studio_vision_msgs
  )
foreach(package IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${package} REQUIRED)
endforeach()
find_package(yaml-cpp REQUIRED)

add_library(
        ur_behaviors
        SHARED
        src/visualize_camera_frustum.cpp
        src/create_pose_stamped_grid.cpp
        src/visualize_placement_zones.cpp
        src/get_highest_confidence_mask.cpp
        src/register_behaviors.cpp)
target_include_directories(
        ur_behaviors
        PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>)
ament_target_dependencies(ur_behaviors
        ${THIS_PACKAGE_INCLUDE_DEPENDS})
target_link_libraries(ur_behaviors yaml-cpp)

# Install Libraries
install(
        TARGETS ur_behaviors
        EXPORT ur_behaviorsTargets
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
        INCLUDES
        DESTINATION include)

ament_export_targets(ur_behaviorsTargets HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
