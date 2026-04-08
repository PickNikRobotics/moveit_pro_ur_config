#include <ur_behaviors/compute_camera_pose_from_tag.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit_pro_behavior_interface/get_required_ports.hpp>
#include <moveit_pro_behavior_interface/metadata_fields.hpp>
#include <tf2_eigen/tf2_eigen.hpp>

#include <Eigen/Geometry>
#include <string>

namespace
{
inline constexpr auto kDescription = R"(
    <p>Computes a camera's world-frame pose from a shared AprilTag observed by two cameras.</p>
    <p>Given the tag pose in world frame (from a camera with known TF, e.g. wrist camera)
       and the tag pose in the target camera's optical frame, computes:
       T_world_camera = T_world_tag * T_camera_tag^{-1}</p>
    <p>The output is the optical frame pose in world. Use the optical-to-link offset
       to get the camera link pose if needed.</p>
)";

constexpr auto kPortIDTagPoseWorld = "tag_pose_world";
constexpr auto kPortIDTagPoseCamera = "tag_pose_camera";
constexpr auto kPortIDCameraPoseWorld = "camera_pose_world";

}  // namespace

namespace ur_behaviors
{

ComputeCameraPoseFromTag::ComputeCameraPoseFromTag(
    const std::string& name, const BT::NodeConfiguration& config,
    const std::shared_ptr<moveit_pro::behaviors::BehaviorContext>& shared_resources)
  : SharedResourcesNode(name, config, shared_resources)
{
}

BT::PortsList ComputeCameraPoseFromTag::providedPorts()
{
  return { BT::InputPort<geometry_msgs::msg::PoseStamped>(
               kPortIDTagPoseWorld, "Tag pose in world frame (from wrist camera detection + TransformPoseFrame)"),
           BT::InputPort<geometry_msgs::msg::PoseStamped>(kPortIDTagPoseCamera,
                                                          "Tag pose in target camera's optical frame (from scene "
                                                          "camera detection, NOT transformed)"),
           BT::OutputPort<geometry_msgs::msg::PoseStamped>(kPortIDCameraPoseWorld, "{camera_pose_world}",
                                                           "Target camera optical frame pose in world frame") };
}

BT::KeyValueVector ComputeCameraPoseFromTag::metadata()
{
  return { { moveit_pro::behaviors::kDescriptionMetadataKey, kDescription },
           { moveit_pro::behaviors::kSubcategoryMetadataKey, "Application - Grocery" } };
}

BT::NodeStatus ComputeCameraPoseFromTag::tick()
{
  const auto ports =
      moveit_pro::behaviors::getRequiredInputs(getInput<geometry_msgs::msg::PoseStamped>(kPortIDTagPoseWorld),
                                               getInput<geometry_msgs::msg::PoseStamped>(kPortIDTagPoseCamera));

  if (!ports.has_value())
  {
    shared_resources_->logger->publishFailureMessage(name(), "Failed to get required value from input data port: " +
                                                                 ports.error());
    return BT::NodeStatus::FAILURE;
  }

  const auto& [tag_pose_world, tag_pose_camera] = ports.value();

  // T_world_tag: tag pose in world frame
  Eigen::Isometry3d T_world_tag;
  tf2::fromMsg(tag_pose_world.pose, T_world_tag);

  // T_camera_tag: tag pose in camera optical frame
  Eigen::Isometry3d T_camera_tag;
  tf2::fromMsg(tag_pose_camera.pose, T_camera_tag);

  // T_world_camera = T_world_tag * T_camera_tag^{-1}
  Eigen::Isometry3d T_world_camera = T_world_tag * T_camera_tag.inverse();

  // Build output
  geometry_msgs::msg::PoseStamped camera_pose_world;
  camera_pose_world.header.frame_id = "world";
  camera_pose_world.header.stamp = shared_resources_->node->now();
  camera_pose_world.pose = tf2::toMsg(T_world_camera);

  setOutput(kPortIDCameraPoseWorld, camera_pose_world);

  // Log the result for debugging
  const auto& p = camera_pose_world.pose.position;
  const auto& q = camera_pose_world.pose.orientation;
  shared_resources_->logger->publishInfoMessage(
      name(), "Computed camera pose in world: position=(" + std::to_string(p.x) + ", " + std::to_string(p.y) + ", " +
                  std::to_string(p.z) + ") orientation=(" + std::to_string(q.x) + ", " + std::to_string(q.y) + ", " +
                  std::to_string(q.z) + ", " + std::to_string(q.w) + ")");

  return BT::NodeStatus::SUCCESS;
}

}  // namespace ur_behaviors
