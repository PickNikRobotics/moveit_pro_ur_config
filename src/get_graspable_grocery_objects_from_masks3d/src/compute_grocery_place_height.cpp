// Copyright 2026 PickNik Inc.
// All rights reserved.
//
// Unauthorized copying of this code base via any medium is strictly prohibited.
// Proprietary and confidential.

#include <get_graspable_grocery_objects_from_masks3d/compute_grocery_place_height.hpp>

#include <algorithm>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <spdlog/spdlog.h>

namespace
{
constexpr auto kPortIDGraspableObject = "graspable_object";
constexpr auto kPortIDPlaceHeight = "place_height";
constexpr auto kPortIDShapeType = "shape_type";
}  // namespace

namespace get_graspable_grocery_objects_from_masks3d
{

ComputeGroceryPlaceHeight::ComputeGroceryPlaceHeight(
    const std::string& name, const BT::NodeConfiguration& config,
    const std::shared_ptr<moveit_pro::behaviors::BehaviorContext>& shared_resources)
  : moveit_pro::behaviors::SharedResourcesNode<BT::SyncActionNode>(name, config, shared_resources)
{
}

BT::PortsList ComputeGroceryPlaceHeight::providedPorts()
{
  return {
    BT::InputPort<moveit_studio_vision_msgs::msg::GraspableObject>(
        kPortIDGraspableObject, "{graspable_object}",
        "GraspableObject with fitted shape. The grasp_info.object_type field must be set to "
        "\"box\", \"cylinder\", or \"sphere\"."),
    BT::OutputPort<double>(kPortIDPlaceHeight, "{place_height}",
                           "Computed height above shelf target for grasp_link placement, in meters."),
    BT::OutputPort<std::string>(kPortIDShapeType, "{shape_type}",
                                "The detected shape type: \"box\", \"cylinder\", or \"sphere\"."),
  };
}

BT::KeyValueVector ComputeGroceryPlaceHeight::metadata()
{
  return { { "description",
             "Computes the place height for a grocery item based on its fitted shape and grasp convention. "
             "Box (grasped at center): half longest dimension. "
             "Cylinder (grasped on top): full height. "
             "Sphere (grasped anywhere): full diameter." },
           { "subcategory", "Perception - 3D" } };
}

BT::NodeStatus ComputeGroceryPlaceHeight::tick()
{
  const auto object_input =
      getInput<moveit_studio_vision_msgs::msg::GraspableObject>(kPortIDGraspableObject);

  if (!object_input.has_value())
  {
    spdlog::error("ComputeGroceryPlaceHeight: failed to get graspable_object input port.");
    return BT::NodeStatus::FAILURE;
  }

  const auto& object = object_input.value();
  const auto& bv = object.grasp_info.bounding_volume;
  const std::string shape_type = object.grasp_info.object_type;

  double place_height = 0.0;

  if (shape_type == "box")
  {
    if (bv.type != shape_msgs::msg::SolidPrimitive::BOX || bv.dimensions.size() < 3)
    {
      spdlog::error("ComputeGroceryPlaceHeight: object_type is 'box' but bounding_volume is not a valid BOX.");
      return BT::NodeStatus::FAILURE;
    }
    // Grasped at the center of a face. The longest dimension is the axis that will be vertical.
    // Place height = half of the longest dimension so the bottom touches the shelf.
    const double longest_dim = std::max({ bv.dimensions[0], bv.dimensions[1], bv.dimensions[2] });
    place_height = longest_dim / 2.0;
    spdlog::info("ComputeGroceryPlaceHeight: box dims=[{:.4f}, {:.4f}, {:.4f}], longest={:.4f}, place_height={:.4f}",
                 bv.dimensions[0], bv.dimensions[1], bv.dimensions[2], longest_dim, place_height);
  }
  else if (shape_type == "cylinder")
  {
    if (bv.type != shape_msgs::msg::SolidPrimitive::CYLINDER || bv.dimensions.size() < 2)
    {
      spdlog::error(
          "ComputeGroceryPlaceHeight: object_type is 'cylinder' but bounding_volume is not a valid CYLINDER.");
      return BT::NodeStatus::FAILURE;
    }
    // CYLINDER dimensions: [height, radius].
    // Grasped on the top flat face. Place height = full height of the cylinder.
    const double height = bv.dimensions[0];
    place_height = height;
    spdlog::info("ComputeGroceryPlaceHeight: cylinder height={:.4f}, radius={:.4f}, place_height={:.4f}",
                 bv.dimensions[0], bv.dimensions[1], place_height);
  }
  else if (shape_type == "sphere")
  {
    if (bv.type != shape_msgs::msg::SolidPrimitive::SPHERE || bv.dimensions.empty())
    {
      spdlog::error("ComputeGroceryPlaceHeight: object_type is 'sphere' but bounding_volume is not a valid SPHERE.");
      return BT::NodeStatus::FAILURE;
    }
    // SPHERE dimensions: [radius].
    // Grasped anywhere. Place height = full diameter so the bottom touches the shelf.
    const double radius = bv.dimensions[0];
    place_height = radius * 2.0;
    spdlog::info("ComputeGroceryPlaceHeight: sphere radius={:.4f}, place_height={:.4f}", radius, place_height);
  }
  else
  {
    spdlog::error("ComputeGroceryPlaceHeight: unknown shape type '{}'. Expected 'box', 'cylinder', or 'sphere'.",
                  shape_type);
    return BT::NodeStatus::FAILURE;
  }

  setOutput(kPortIDPlaceHeight, place_height);
  setOutput(kPortIDShapeType, shape_type);
  return BT::NodeStatus::SUCCESS;
}

}  // namespace get_graspable_grocery_objects_from_masks3d
