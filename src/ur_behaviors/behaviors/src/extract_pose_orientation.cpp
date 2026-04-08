#include <ur_behaviors/extract_pose_orientation.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit_pro_behavior_interface/get_required_ports.hpp>
#include <moveit_pro_behavior_interface/metadata_fields.hpp>

#include <vector>

namespace ur_behaviors
{

ExtractPoseOrientation::ExtractPoseOrientation(
    const std::string& name, const BT::NodeConfiguration& config,
    const std::shared_ptr<moveit_pro::behaviors::BehaviorContext>& shared_resources)
  : SharedResourcesNode(name, config, shared_resources)
{
}

BT::PortsList ExtractPoseOrientation::providedPorts()
{
  return { BT::InputPort<geometry_msgs::msg::PoseStamped>("input_pose", "PoseStamped to extract orientation from"),
           BT::OutputPort<std::vector<double>>("orientation_xyzw", "{orientation_xyzw}",
                                               "Quaternion as [x, y, z, w] vector, compatible with "
                                               "OverridePoseOrientation") };
}

BT::KeyValueVector ExtractPoseOrientation::metadata()
{
  return { { moveit_pro::behaviors::kDescriptionMetadataKey,
             "Extracts orientation from a PoseStamped as a [x,y,z,w] vector." },
           { moveit_pro::behaviors::kSubcategoryMetadataKey, "Pose Handling" } };
}

BT::NodeStatus ExtractPoseOrientation::tick()
{
  const auto pose_input = getInput<geometry_msgs::msg::PoseStamped>("input_pose");
  if (!pose_input.has_value())
  {
    shared_resources_->logger->publishFailureMessage(name(), "Failed to get input_pose");
    return BT::NodeStatus::FAILURE;
  }

  const auto& q = pose_input->pose.orientation;
  setOutput("orientation_xyzw", std::vector<double>{ q.x, q.y, q.z, q.w });
  return BT::NodeStatus::SUCCESS;
}

}  // namespace ur_behaviors
