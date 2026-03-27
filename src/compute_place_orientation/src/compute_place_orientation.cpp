// Copyright 2026 PickNik Inc.
// All rights reserved.
//
// Unauthorized copying of this code base via any medium is strictly prohibited.
// Proprietary and confidential.

#include <compute_place_orientation/compute_place_orientation.hpp>

#include <Eigen/Dense>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <spdlog/spdlog.h>
#include <tf2_eigen/tf2_eigen.hpp>

namespace
{
constexpr auto kPortIDObjectPose = "object_pose";
constexpr auto kPortIDPickPose = "pick_pose";
constexpr auto kPortIDPlacePosition = "place_position";
constexpr auto kPortIDPlacePose = "place_pose";
constexpr auto kPortIDHeightOffset = "height_offset";

/**
 * @brief Convert a PoseStamped to an Eigen::Isometry3d.
 */
Eigen::Isometry3d poseToIsometry(const geometry_msgs::msg::PoseStamped& pose_msg)
{
  Eigen::Isometry3d iso = Eigen::Isometry3d::Identity();
  iso.translation() = Eigen::Vector3d(pose_msg.pose.position.x, pose_msg.pose.position.y, pose_msg.pose.position.z);
  Eigen::Quaterniond q(pose_msg.pose.orientation.w, pose_msg.pose.orientation.x, pose_msg.pose.orientation.y,
                       pose_msg.pose.orientation.z);
  iso.linear() = q.toRotationMatrix();
  return iso;
}

}  // namespace

namespace compute_place_orientation
{

ComputePlaceOrientation::ComputePlaceOrientation(const std::string& name, const BT::NodeConfiguration& config)
  : BT::SyncActionNode(name, config)
{
}

BT::PortsList ComputePlaceOrientation::providedPorts()
{
  return {
    BT::InputPort<geometry_msgs::msg::PoseStamped>(kPortIDObjectPose, "{object_pose}",
                                                    "Object pose in world frame from detection."),
    BT::InputPort<geometry_msgs::msg::PoseStamped>(kPortIDPickPose, "{pick_pose}",
                                                    "Grasp_link pose in world frame at the time of picking."),
    BT::InputPort<geometry_msgs::msg::PoseStamped>(kPortIDPlacePosition, "{shelf_target_pose}",
                                                    "Target place position (only XY used, Z computed from height_offset)."),
    BT::InputPort<double>(kPortIDHeightOffset, 0.0,
                          "Height offset above the place position for the object bottom (typically place_height)."),
    BT::OutputPort<geometry_msgs::msg::PoseStamped>(kPortIDPlacePose, "{place_pose}",
                                                     "Computed grasp_link pose for placing the object upright."),
  };
}

BT::KeyValueVector ComputePlaceOrientation::metadata()
{
  return { { "subcategory", "Motion - Planning" },
           { "description",
             "Computes the grasp_link pose for placing a held object upright at a target location. "
             "The object's longest axis (Z of its fitted shape frame) is aligned with world Z." } };
}

BT::NodeStatus ComputePlaceOrientation::tick()
{
  const auto object_pose_input = getInput<geometry_msgs::msg::PoseStamped>(kPortIDObjectPose);
  const auto pick_pose_input = getInput<geometry_msgs::msg::PoseStamped>(kPortIDPickPose);
  const auto place_position_input = getInput<geometry_msgs::msg::PoseStamped>(kPortIDPlacePosition);
  const auto height_offset_input = getInput<double>(kPortIDHeightOffset);

  if (!object_pose_input || !pick_pose_input || !place_position_input)
  {
    spdlog::error("ComputePlaceOrientation: failed to get required input ports.");
    return BT::NodeStatus::FAILURE;
  }

  const double height_offset = height_offset_input.value_or(0.0);

  // T_world_object: object's pose in world frame at detection time
  const Eigen::Isometry3d T_world_object = poseToIsometry(object_pose_input.value());
  // T_world_grasp: grasp_link's pose in world frame at pick time
  const Eigen::Isometry3d T_world_grasp = poseToIsometry(pick_pose_input.value());

  // Compute the fixed transform from grasp_link to object (doesn't change after grasping)
  const Eigen::Isometry3d T_grasp_object = T_world_grasp.inverse() * T_world_object;

  spdlog::info("ComputePlaceOrientation: T_grasp_object translation = [{:.4f}, {:.4f}, {:.4f}]",
               T_grasp_object.translation().x(), T_grasp_object.translation().y(),
               T_grasp_object.translation().z());

  // Desired object orientation at place: object Z aligned with world Z.
  // We want the object's Z axis (longest axis of fitted shape) to point up.
  // The simplest: R_world_object_desired = Identity (object Z = world Z).
  // But we also need to choose a yaw. Use the object's original yaw (around Z).
  const Eigen::Vector3d object_z = T_world_object.linear().col(2);
  spdlog::info("ComputePlaceOrientation: object Z axis in world = [{:.4f}, {:.4f}, {:.4f}]",
               object_z.x(), object_z.y(), object_z.z());

  // For the desired orientation, we want object Z = world Z.
  // Preserve the object's XY orientation (yaw) as much as possible.
  Eigen::Isometry3d T_world_object_desired = Eigen::Isometry3d::Identity();

  // Place position from input. The object center when upright is at shelf_z + half of height_offset.
  // height_offset is the full object height (longest axis), so center = shelf_z + height_offset/2.
  const auto& place_pos = place_position_input.value();
  T_world_object_desired.translation() =
      Eigen::Vector3d(place_pos.pose.position.x, place_pos.pose.position.y,
                      place_pos.pose.position.z + height_offset / 2.0);
  spdlog::info("ComputePlaceOrientation: object center at z={:.4f} (shelf_z={:.4f} + height/2={:.4f})",
               T_world_object_desired.translation().z(), place_pos.pose.position.z, height_offset / 2.0);

  // Desired object rotation: Z = world Z, preserve original yaw
  // Project the object's X axis onto the XY plane for yaw preservation
  Eigen::Vector3d object_x = T_world_object.linear().col(0);
  object_x.z() = 0.0;
  if (object_x.norm() < 1e-6)
  {
    // Object X is nearly vertical, use world X instead
    object_x = Eigen::Vector3d::UnitX();
  }
  object_x.normalize();
  const Eigen::Vector3d desired_z = Eigen::Vector3d::UnitZ();
  const Eigen::Vector3d desired_y = desired_z.cross(object_x).normalized();
  const Eigen::Vector3d desired_x = desired_y.cross(desired_z).normalized();

  Eigen::Matrix3d R_desired;
  R_desired.col(0) = desired_x;
  R_desired.col(1) = desired_y;
  R_desired.col(2) = desired_z;
  T_world_object_desired.linear() = R_desired;

  // Compute the desired grasp_link pose: T_world_grasp_desired = T_world_object_desired * T_grasp_object^-1
  const Eigen::Isometry3d T_world_grasp_desired = T_world_object_desired * T_grasp_object.inverse();

  spdlog::info("ComputePlaceOrientation: desired grasp position = [{:.4f}, {:.4f}, {:.4f}]",
               T_world_grasp_desired.translation().x(), T_world_grasp_desired.translation().y(),
               T_world_grasp_desired.translation().z());

  // Convert to PoseStamped
  geometry_msgs::msg::PoseStamped place_pose;
  place_pose.header.frame_id = object_pose_input->header.frame_id;
  place_pose.pose = tf2::toMsg(T_world_grasp_desired);

  setOutput(kPortIDPlacePose, place_pose);
  return BT::NodeStatus::SUCCESS;
}

}  // namespace compute_place_orientation
