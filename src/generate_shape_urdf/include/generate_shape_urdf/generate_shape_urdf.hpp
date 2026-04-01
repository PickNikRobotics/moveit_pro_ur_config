// Copyright 2026 PickNik Inc.
// All rights reserved.
//
// Unauthorized copying of this code base via any medium is strictly prohibited.
// Proprietary and confidential.

#pragma once

#include <behaviortree_cpp/action_node.h>

#include <string>
#include <vector>

namespace generate_shape_urdf
{

/**
 * @brief Generates a URDF string for a primitive shape (box, sphere, or cylinder).
 *
 * @details Given a shape type and a vector of dimensions, this Behavior produces a complete
 * URDF XML string with both visual and collision geometry. The generated URDF can then be
 * passed to AddURDF or written to a file.
 *
 * Dimension requirements per shape type:
 * - box: [width, height, depth] (3 values in meters)
 * - sphere: [radius] (1 value in meters)
 * - cylinder: [radius, height] (2 values in meters)
 *
 * | Data Port Name | Port Type | Object Type         |
 * | -------------- | --------- | ------------------- |
 * | shape_type     | input     | std::string         |
 * | dimensions     | input     | std::vector<double> |
 * | urdf_string    | output    | std::string         |
 */
class GenerateShapeUrdf final : public BT::SyncActionNode
{
public:
  GenerateShapeUrdf(const std::string& name, const BT::NodeConfiguration& config);

  static BT::PortsList providedPorts();

  static BT::KeyValueVector metadata();

  BT::NodeStatus tick() override;
};

}  // namespace generate_shape_urdf
