// Copyright 2026 PickNik Inc.
// All rights reserved.
//
// Unauthorized copying of this code base via any medium is strictly prohibited.
// Proprietary and confidential.

#include <get_graspable_grocery_objects_from_masks3d/compute_grocery_place_height.hpp>

#include <algorithm>
#include <cmath>
#include <vector>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <Eigen/Dense>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <spdlog/spdlog.h>

namespace
{
constexpr auto kPortIDGraspableObject = "graspable_object";
constexpr auto kPortIDPlaceHeight = "place_height";
constexpr auto kPortIDPlaceTranslation = "place_translation_xyz";
constexpr auto kPortIDShapeType = "shape_type";
constexpr auto kPortIDGraspApproachTranslation = "grasp_approach_translation_xyz";
constexpr auto kPortIDTopOffset = "top_offset";
constexpr auto kPortIDCollisionDimensions = "collision_dimensions";
constexpr auto kPortIDCollisionRPY = "collision_rpy";
constexpr auto kPortIDPickPose = "pick_pose";
constexpr auto kPortIDIsOnSide = "is_on_side";
constexpr auto kPortIDDepthOffset = "depth_offset";
constexpr double kGraspApproachClearance = 0.0;  // directly at top surface
constexpr double kOnSideThreshold = 0.5;  // axis_z < this means object is on its side
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
    BT::OutputPort<std::vector<double>>(kPortIDPlaceTranslation, "{place_translation_xyz}",
                                        "Translation vector [0, 0, -place_height] for use with TransformPose."),
    BT::OutputPort<std::string>(kPortIDShapeType, "{shape_type}",
                                "The detected shape type: \"box\", \"cylinder\", or \"sphere\"."),
    BT::OutputPort<double>(kPortIDTopOffset, "{top_offset}",
                           "Distance from object center to topmost point in world Z. Use as collision URDF origin offset."),
    BT::OutputPort<std::vector<double>>(kPortIDGraspApproachTranslation, "{grasp_approach_translation_xyz}",
                                        "Translation from object center to grasp approach point [0, 0, -(half_height + clearance)]."),
    BT::OutputPort<std::vector<double>>(kPortIDCollisionDimensions, "{collision_dimensions}",
                                        "Dimensions for GenerateShapeUrdf. Cylinder: [radius, height], Box: [x, y, z], Sphere: [radius]."),
    BT::OutputPort<std::vector<double>>(kPortIDCollisionRPY, "{collision_rpy}",
                                        "Roll, pitch, yaw of the object shape frame relative to grasp_link."),
    BT::InputPort<geometry_msgs::msg::PoseStamped>(kPortIDPickPose, "{pick_pose}",
                                                    "Grasp_link pose at pick time, used to compute relative orientation."),
    BT::OutputPort<bool>(kPortIDIsOnSide, "{is_on_side}",
                          "True if the object's longest axis is roughly horizontal (on its side)."),
    BT::OutputPort<double>(kPortIDDepthOffset, "{depth_offset}",
                           "Forward offset for placement: cylinder/sphere radius, or box half-depth."),
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

  // Get the object's orientation in world to determine which shape axis points up.
  const auto& q = object.object.pose.orientation;
  const Eigen::Quaterniond object_quat(q.w, q.x, q.y, q.z);
  const Eigen::Matrix3d R = object_quat.toRotationMatrix();

  // The shape frame axes in world frame
  const Eigen::Vector3d shape_x = R.col(0);
  const Eigen::Vector3d shape_y = R.col(1);
  const Eigen::Vector3d shape_z = R.col(2);  // cylinder axis, or box Z axis

  double place_height = 0.0;
  double top_offset = 0.0;  // distance from object center to topmost point in world Z

  if (shape_type == "box")
  {
    if (bv.type != shape_msgs::msg::SolidPrimitive::BOX || bv.dimensions.size() < 3)
    {
      spdlog::error("ComputeGroceryPlaceHeight: object_type is 'box' but bounding_volume is not a valid BOX.");
      return BT::NodeStatus::FAILURE;
    }
    // Half-extents along each shape axis
    const double hx = bv.dimensions[0] / 2.0;
    const double hy = bv.dimensions[1] / 2.0;
    const double hz = bv.dimensions[2] / 2.0;
    // The topmost point above center = sum of projections of each half-extent onto world Z
    top_offset = std::abs(shape_x.z()) * hx + std::abs(shape_y.z()) * hy + std::abs(shape_z.z()) * hz;
    // Place height = longest dimension (object will be placed upright on shortest face)
    const double longest_dim = std::max({ bv.dimensions[0], bv.dimensions[1], bv.dimensions[2] });
    place_height = longest_dim / 2.0;
    spdlog::info("ComputeGroceryPlaceHeight: box dims=[{:.4f}, {:.4f}, {:.4f}], top_offset={:.4f}, place_height={:.4f}",
                 bv.dimensions[0], bv.dimensions[1], bv.dimensions[2], top_offset, place_height);
  }
  else if (shape_type == "cylinder")
  {
    if (bv.type != shape_msgs::msg::SolidPrimitive::CYLINDER || bv.dimensions.size() < 2)
    {
      spdlog::error(
          "ComputeGroceryPlaceHeight: object_type is 'cylinder' but bounding_volume is not a valid CYLINDER.");
      return BT::NodeStatus::FAILURE;
    }
    const double height = bv.dimensions[0];
    const double cyl_radius = bv.dimensions[1];
    // Cylinder Z axis = shape_z. The topmost point above center depends on orientation:
    // Along cylinder axis: half_height * |shape_z . world_z|
    // Along radial direction: radius * sqrt(1 - (shape_z . world_z)^2)
    const double axis_z_component = std::abs(shape_z.z());
    top_offset = (height / 2.0) * axis_z_component + cyl_radius * std::sqrt(1.0 - axis_z_component * axis_z_component);
    // Place height = distance from shelf surface to grasp_link when object is upright.
    // - Picked from top (upright): grasp is at the top → place_height = full height
    // - Picked from side: grasp is at center → place_height = half height
    if (axis_z_component < kOnSideThreshold)
    {
      place_height = height / 2.0;
      spdlog::info("ComputeGroceryPlaceHeight: cylinder ON SIDE, height={:.4f}, radius={:.4f}, axis_z={:.3f}, place_height={:.4f} (half)",
                   height, cyl_radius, axis_z_component, place_height);
    }
    else
    {
      place_height = height;
      spdlog::info("ComputeGroceryPlaceHeight: cylinder UPRIGHT, height={:.4f}, radius={:.4f}, axis_z={:.3f}, place_height={:.4f} (full)",
                   height, cyl_radius, axis_z_component, place_height);
    }
  }
  else if (shape_type == "sphere")
  {
    if (bv.type != shape_msgs::msg::SolidPrimitive::SPHERE || bv.dimensions.empty())
    {
      spdlog::error("ComputeGroceryPlaceHeight: object_type is 'sphere' but bounding_volume is not a valid SPHERE.");
      return BT::NodeStatus::FAILURE;
    }
    const double radius = bv.dimensions[0];
    top_offset = radius;
    place_height = radius * 2.0;
    spdlog::info("ComputeGroceryPlaceHeight: sphere radius={:.4f}, place_height={:.4f}", radius, place_height);
  }
  else
  {
    spdlog::error("ComputeGroceryPlaceHeight: unknown shape type '{}'. Expected 'box', 'cylinder', or 'sphere'.",
                  shape_type);
    return BT::NodeStatus::FAILURE;
  }

  // Grasp approach: from object center, move up in world Z to the topmost point.
  // In the pick_pose frame (gripper pointing down), world-up = negative Z.
  const double approach_z = -(top_offset + kGraspApproachClearance);
  spdlog::info("ComputeGroceryPlaceHeight: top_offset={:.4f}, approach_z={:.4f}", top_offset, approach_z);

  // Output dimensions in the format GenerateShapeUrdf expects.
  // Cylinder: [radius, height], Box: [x, y, z], Sphere: [radius]
  std::vector<double> collision_dims;
  if (shape_type == "cylinder")
  {
    collision_dims = { bv.dimensions[1], bv.dimensions[0] };  // [radius, height]
  }
  else if (shape_type == "box")
  {
    collision_dims = { bv.dimensions[0], bv.dimensions[1], bv.dimensions[2] };
  }
  else  // sphere
  {
    collision_dims = { bv.dimensions[0] };  // [radius]
  }

  // Compute the object's orientation relative to grasp_link for the collision URDF.
  // R_grasp_object = R_world_grasp^-1 * R_world_object
  std::vector<double> collision_rpy = {0.0, 0.0, 0.0};
  const auto pick_pose_input = getInput<geometry_msgs::msg::PoseStamped>(kPortIDPickPose);
  if (pick_pose_input.has_value())
  {
    const auto& gq = pick_pose_input->pose.orientation;
    const Eigen::Quaterniond q_grasp(gq.w, gq.x, gq.y, gq.z);
    const Eigen::Quaterniond q_object(object_quat);
    const Eigen::Quaterniond q_grasp_object = q_grasp.inverse() * q_object;
    const Eigen::Vector3d euler = q_grasp_object.toRotationMatrix().eulerAngles(0, 1, 2);
    collision_rpy = {euler.x(), euler.y(), euler.z()};
    spdlog::info("ComputeGroceryPlaceHeight: collision_rpy=[{:.4f}, {:.4f}, {:.4f}]",
                 collision_rpy[0], collision_rpy[1], collision_rpy[2]);
  }

  // Determine if the object is on its side (longest axis roughly horizontal).
  // For cylinder: check if the cylinder Z axis has a small world-Z component.
  // For box: check if the longest dimension's axis has a small world-Z component.
  // For sphere: never on its side.
  bool is_on_side = false;
  if (shape_type == "cylinder")
  {
    const double axis_z_component = std::abs(shape_z.z());
    is_on_side = (axis_z_component < kOnSideThreshold);
    spdlog::info("ComputeGroceryPlaceHeight: cylinder axis_z={:.3f}, is_on_side={}", axis_z_component, is_on_side);
  }
  else if (shape_type == "box")
  {
    // Find which axis corresponds to the longest dimension
    const std::array<double, 3> dims = {bv.dimensions[0], bv.dimensions[1], bv.dimensions[2]};
    const std::array<Eigen::Vector3d, 3> axes = {shape_x, shape_y, shape_z};
    const auto max_it = std::max_element(dims.begin(), dims.end());
    const size_t max_idx = std::distance(dims.begin(), max_it);
    const double longest_axis_z = std::abs(axes[max_idx].z());
    is_on_side = (longest_axis_z < kOnSideThreshold);
    spdlog::info("ComputeGroceryPlaceHeight: box longest_axis_z={:.3f}, is_on_side={}", longest_axis_z, is_on_side);
  }

  // Compute depth offset: how far to scoot the placement toward the robot
  // so the object center clears the shelf surface.
  double depth_offset = 0.0;
  if (shape_type == "cylinder")
  {
    depth_offset = bv.dimensions[1];  // radius
    spdlog::info("ComputeGroceryPlaceHeight: depth_offset={:.4f} (cylinder radius)", depth_offset);
  }
  else if (shape_type == "sphere")
  {
    depth_offset = bv.dimensions[0];  // radius
    spdlog::info("ComputeGroceryPlaceHeight: depth_offset={:.4f} (sphere radius)", depth_offset);
  }
  else if (shape_type == "box")
  {
    // Use the smallest horizontal dimension as the depth (object placed upright, thinnest side faces shelf back)
    const std::array<double, 3> dims = {bv.dimensions[0], bv.dimensions[1], bv.dimensions[2]};
    // Find the two non-longest dimensions (the longest will be vertical when upright)
    double max_dim = *std::max_element(dims.begin(), dims.end());
    double min_horizontal = max_dim;
    for (const auto& d : dims)
    {
      if (d < max_dim && d < min_horizontal)
        min_horizontal = d;
    }
    depth_offset = min_horizontal / 2.0;
    spdlog::info("ComputeGroceryPlaceHeight: depth_offset={:.4f} (box half-depth)", depth_offset);
  }

  setOutput(kPortIDPlaceHeight, place_height);
  setOutput(kPortIDDepthOffset, depth_offset);
  setOutput(kPortIDPlaceTranslation, std::vector<double>{0.0, 0.0, -place_height});
  setOutput(kPortIDGraspApproachTranslation, std::vector<double>{0.0, 0.0, approach_z});
  setOutput(kPortIDTopOffset, top_offset);
  setOutput(kPortIDCollisionDimensions, collision_dims);
  setOutput(kPortIDCollisionRPY, collision_rpy);
  setOutput(kPortIDShapeType, shape_type);
  setOutput(kPortIDIsOnSide, is_on_side);
  return BT::NodeStatus::SUCCESS;
}

}  // namespace get_graspable_grocery_objects_from_masks3d
