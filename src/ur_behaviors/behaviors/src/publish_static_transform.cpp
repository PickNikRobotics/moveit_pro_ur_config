#include <ur_behaviors/publish_static_transform.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <moveit_pro_behavior_interface/get_required_ports.hpp>
#include <moveit_pro_behavior_interface/metadata_fields.hpp>

#include <string>

namespace
{
inline constexpr auto kDescription = R"(
    <p>Publishes a static transform to /tf_static from a PoseStamped.</p>
    <p>Use this to bridge frames that are not connected in the TF tree,
       e.g. connecting a calibrated scene camera to the world frame.</p>
)";

constexpr auto kPortIDPose = "pose";
constexpr auto kPortIDChildFrameID = "child_frame_id";

}  // namespace

namespace ur_behaviors
{

PublishStaticTransform::PublishStaticTransform(
    const std::string& name,
    const BT::NodeConfiguration& config,
    const std::shared_ptr<moveit_pro::behaviors::BehaviorContext>& shared_resources)
  : SharedResourcesNode(name, config, shared_resources)
  , static_broadcaster_{ std::make_shared<tf2_ros::StaticTransformBroadcaster>(shared_resources_->node) }
{
}

BT::PortsList PublishStaticTransform::providedPorts()
{
  return {
    BT::InputPort<geometry_msgs::msg::PoseStamped>(kPortIDPose,
      "Pose defining the transform (frame_id = parent, position/orientation = transform)"),
    BT::InputPort<std::string>(kPortIDChildFrameID, "scene_camera_link",
      "Child frame ID for the static transform")
  };
}

BT::KeyValueVector PublishStaticTransform::metadata()
{
  return {
    { moveit_pro::behaviors::kDescriptionMetadataKey, kDescription },
    { moveit_pro::behaviors::kSubcategoryMetadataKey, "Utilities" }
  };
}

BT::NodeStatus PublishStaticTransform::tick()
{
  const auto ports = moveit_pro::behaviors::getRequiredInputs(
      getInput<geometry_msgs::msg::PoseStamped>(kPortIDPose),
      getInput<std::string>(kPortIDChildFrameID));

  if (!ports.has_value())
  {
    shared_resources_->logger->publishFailureMessage(name(),
        "Failed to get required value from input data port: " + ports.error());
    return BT::NodeStatus::FAILURE;
  }

  const auto& [pose, child_frame_id] = ports.value();

  geometry_msgs::msg::TransformStamped tf_msg;
  tf_msg.header.stamp = shared_resources_->node->now();
  tf_msg.header.frame_id = pose.header.frame_id;
  tf_msg.child_frame_id = child_frame_id;
  tf_msg.transform.translation.x = pose.pose.position.x;
  tf_msg.transform.translation.y = pose.pose.position.y;
  tf_msg.transform.translation.z = pose.pose.position.z;
  tf_msg.transform.rotation = pose.pose.orientation;

  static_broadcaster_->sendTransform(tf_msg);

  shared_resources_->logger->publishInfoMessage(name(),
      "Published static transform: " + pose.header.frame_id + " -> " + child_frame_id);

  return BT::NodeStatus::SUCCESS;
}

}  // namespace ur_behaviors
