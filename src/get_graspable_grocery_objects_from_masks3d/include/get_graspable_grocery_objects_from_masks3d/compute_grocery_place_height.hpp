// Copyright 2026 PickNik Inc.
// All rights reserved.
//
// Unauthorized copying of this code base via any medium is strictly prohibited.
// Proprietary and confidential.

#pragma once

#include <behaviortree_cpp/action_node.h>
#include <moveit_pro_behavior_interface/shared_resources_node.hpp>
#include <moveit_studio_vision_msgs/msg/graspable_object.hpp>

namespace get_graspable_grocery_objects_from_masks3d
{

/**
 * @brief Computes the place height for a grocery item based on its fitted shape and grasp convention.
 *
 * @details Given a GraspableObject with a fitted shape type stored in grasp_info.object_type, this
 * behavior computes the height at which grasp_link should be positioned above the shelf target so
 * that the object rests on the shelf with its longest axis perpendicular to the surface.
 *
 * Grasp conventions:
 * - Box: grasped at the center of a face. Place height = half the longest box dimension.
 * - Cylinder: grasped on the top flat face. Place height = full cylinder height.
 * - Sphere: grasped anywhere. Place height = full diameter (2 * radius).
 *
 * | Data Port Name    | Port Type | Object Type                                         |
 * | ----------------- | --------- | --------------------------------------------------- |
 * | graspable_object  | input     | moveit_studio_vision_msgs::msg::GraspableObject     |
 * | place_height      | output    | double                                              |
 * | shape_type        | output    | std::string                                         |
 */
class ComputeGroceryPlaceHeight final : public moveit_pro::behaviors::SharedResourcesNode<BT::SyncActionNode>
{
public:
  ComputeGroceryPlaceHeight(const std::string& name, const BT::NodeConfiguration& config,
                            const std::shared_ptr<moveit_pro::behaviors::BehaviorContext>& shared_resources);

  static BT::PortsList providedPorts();

  static BT::KeyValueVector metadata();

  BT::NodeStatus tick() override;
};

}  // namespace get_graspable_grocery_objects_from_masks3d
