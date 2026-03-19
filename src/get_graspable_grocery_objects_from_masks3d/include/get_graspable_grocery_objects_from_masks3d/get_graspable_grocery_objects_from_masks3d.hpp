// Copyright 2026 PickNik Inc.
// All rights reserved.
//
// Unauthorized copying of this code base via any medium is strictly prohibited.
// Proprietary and confidential.

#pragma once

#include <behaviortree_cpp/basic_types.h>

#include <moveit_pro_behavior_interface/async_behavior_base.hpp>
#include <rclcpp/publisher.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

namespace get_graspable_grocery_objects_from_masks3d
{

/**
 * @brief Creates graspable objects from 3D masks, fitting the best primitive shape (box, cylinder, or sphere).
 *
 * @details For each point cloud segment defined by a 3D mask, this behavior fits three primitive shapes
 * (box, cylinder, sphere) and selects the one with the smallest bounding volume. The fitted shape type
 * is stored in the GraspInfo's object_type field as "box", "cylinder", or "sphere", and the bounding_volume
 * primitive is set to the corresponding shape_msgs::SolidPrimitive type.
 *
 * | Data Port Name            | Port Type | Object Type                                         |
 * | --------------------------| --------- | --------------------------------------------------- |
 * | point_cloud               | input     | sensor_msgs::msg::PointCloud2                       |
 * | masks3d                   | input     | std::vector<moveit_studio_vision_msgs::msg::Mask3D> |
 * | base_frame                | input     | std::string                                         |
 * | plane_inlier_threshold    | input     | double                                              |
 * | minimum_face_area         | input     | double                                              |
 * | face_separation_threshold | input     | double                                              |
 * | graspable_objects         | output    | std::vector<moveit_studio_vision_msgs::msg::GraspableObject> |
 */
class GetGraspableGroceryObjectsFromMasks3D final : public moveit_pro::behaviors::AsyncBehaviorBase
{
public:
  /**
   * @param name Name of this behavior tree node.
   * @param config Node configuration.
   * @param shared_resources Shared behavior context with ROS node, TF buffer, etc.
   */
  GetGraspableGroceryObjectsFromMasks3D(const std::string& name, const BT::NodeConfiguration& config,
                                        const std::shared_ptr<moveit_pro::behaviors::BehaviorContext>& shared_resources);

  static BT::PortsList providedPorts();

  static BT::KeyValueVector metadata();

protected:
  tl::expected<bool, std::string> doWork() override;

private:
  std::shared_future<tl::expected<bool, std::string>>& getFuture() override
  {
    return future_;
  }

  std::shared_future<tl::expected<bool, std::string>> future_;

  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
};

}  // namespace get_graspable_grocery_objects_from_masks3d
