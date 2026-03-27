// Copyright 2026 PickNik Inc.
// All rights reserved.
//
// Unauthorized copying of this code base via any medium is strictly prohibited.
// Proprietary and confidential.

#pragma once

#include <behaviortree_cpp/action_node.h>

namespace compute_place_orientation
{

/**
 * @brief Computes the grasp_link pose for placing an object upright at a target position.
 *
 * @details Given the object's pose at detection time, the grasp_link pose at pick time,
 * and a target place position, this behavior computes the grasp_link orientation needed
 * so that the object's longest axis (Z axis of the fitted shape frame) is vertical
 * when placed.
 *
 * The math:
 *   T_grasp_object = T_world_grasp^-1 * T_world_object  (fixed after grasping)
 *   R_world_object_desired = rotation that aligns object Z with world Z
 *   T_world_grasp_desired = T_world_object_desired * T_grasp_object^-1
 *
 * | Data Port Name     | Port Type | Object Type                          |
 * | ------------------ | --------- | ------------------------------------ |
 * | object_pose        | input     | geometry_msgs::msg::PoseStamped      |
 * | pick_pose          | input     | geometry_msgs::msg::PoseStamped      |
 * | place_position     | input     | geometry_msgs::msg::PoseStamped      |
 * | place_pose         | output    | geometry_msgs::msg::PoseStamped      |
 */
class ComputePlaceOrientation final : public BT::SyncActionNode
{
public:
  ComputePlaceOrientation(const std::string& name, const BT::NodeConfiguration& config);

  static BT::PortsList providedPorts();

  static BT::KeyValueVector metadata();

  BT::NodeStatus tick() override;
};

}  // namespace compute_place_orientation
