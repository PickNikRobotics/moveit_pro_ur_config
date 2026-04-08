#include <ur_behaviors/create_pose_stamped_grid.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit_pro_behavior_interface/get_required_ports.hpp>
#include <moveit_pro_behavior_interface/metadata_fields.hpp>
#include <tf2_eigen/tf2_eigen.hpp>

#include <Eigen/Geometry>
#include <cmath>
#include <vector>

namespace
{
inline constexpr auto kDescriptionCreatePoseStampedGrid = R"(
    <p>Creates a 3D grid of PoseStamped messages from a single input pose.</p>
    <p>Offsets are applied in the local frame of the input pose. Rotations are incremental per step along each axis, in degrees.</p>
)";

constexpr auto kPortPose = "pose";
constexpr auto kPortRepeatX = "repeat_x";
constexpr auto kPortRepeatY = "repeat_y";
constexpr auto kPortRepeatZ = "repeat_z";
constexpr auto kPortOffsetX = "offset_x";
constexpr auto kPortOffsetY = "offset_y";
constexpr auto kPortOffsetZ = "offset_z";
constexpr auto kPortRotateX = "rotate_x";
constexpr auto kPortRotateY = "rotate_y";
constexpr auto kPortRotateZ = "rotate_z";
constexpr auto kPortPoses = "poses";

inline double degToRad(double degrees)
{
  return degrees * M_PI / 180.0;
}
}  // namespace

namespace ur_behaviors
{

CreatePoseStampedGrid::CreatePoseStampedGrid(
    const std::string& name, const BT::NodeConfiguration& config,
    const std::shared_ptr<moveit_pro::behaviors::BehaviorContext>& shared_resources)
  : SharedResourcesNode(name, config, shared_resources)
{
}

BT::PortsList CreatePoseStampedGrid::providedPorts()
{
  return {
    BT::InputPort<geometry_msgs::msg::PoseStamped>(kPortPose, "Base pose to replicate"),
    BT::InputPort<int>(kPortRepeatX, 0, "Number of additional copies along local X"),
    BT::InputPort<int>(kPortRepeatY, 0, "Number of additional copies along local Y"),
    BT::InputPort<int>(kPortRepeatZ, 0, "Number of additional copies along local Z"),
    BT::InputPort<double>(kPortOffsetX, 0.0, "Spacing between copies along local X (meters)"),
    BT::InputPort<double>(kPortOffsetY, 0.0, "Spacing between copies along local Y (meters)"),
    BT::InputPort<double>(kPortOffsetZ, 0.0, "Spacing between copies along local Z (meters)"),
    BT::InputPort<double>(kPortRotateX, 0.0, "Incremental rotation per X step (degrees)"),
    BT::InputPort<double>(kPortRotateY, 0.0, "Incremental rotation per Y step (degrees)"),
    BT::InputPort<double>(kPortRotateZ, 0.0, "Incremental rotation per Z step (degrees)"),
    BT::OutputPort<std::vector<geometry_msgs::msg::PoseStamped>>(kPortPoses, "Generated grid of poses"),
  };
}

BT::KeyValueVector CreatePoseStampedGrid::metadata()
{
  return {
    { moveit_pro::behaviors::kDescriptionMetadataKey, kDescriptionCreatePoseStampedGrid },
  };
}

BT::NodeStatus CreatePoseStampedGrid::tick()
{
  // Get required inputs
  const auto ports = moveit_pro::behaviors::getRequiredInputs(
      getInput<geometry_msgs::msg::PoseStamped>(kPortPose), getInput<int>(kPortRepeatX), getInput<int>(kPortRepeatY),
      getInput<int>(kPortRepeatZ), getInput<double>(kPortOffsetX), getInput<double>(kPortOffsetY),
      getInput<double>(kPortOffsetZ), getInput<double>(kPortRotateX), getInput<double>(kPortRotateY),
      getInput<double>(kPortRotateZ));

  if (!ports.has_value())
  {
    shared_resources_->logger->publishFailureMessage(name(), "Failed to get required value from input data port: " +
                                                                 ports.error());
    return BT::NodeStatus::FAILURE;
  }

  const auto& [pose, repeat_x, repeat_y, repeat_z, offset_x, offset_y, offset_z, rotate_x, rotate_y, rotate_z] =
      ports.value();

  // Validate repeat counts
  if (repeat_x < 0 || repeat_y < 0 || repeat_z < 0)
  {
    shared_resources_->logger->publishFailureMessage(name(), "repeat_x, repeat_y, and repeat_z must be >= 0");
    return BT::NodeStatus::FAILURE;
  }

  // Extract the original pose as an Eigen transform
  Eigen::Isometry3d original_transform;
  tf2::fromMsg(pose.pose, original_transform);
  const Eigen::Quaterniond original_rotation(original_transform.rotation());

  // Generate grid
  std::vector<geometry_msgs::msg::PoseStamped> output_poses;
  output_poses.reserve(static_cast<size_t>((repeat_x + 1) * (repeat_y + 1) * (repeat_z + 1)));

  for (int iz = 0; iz <= repeat_z; ++iz)
  {
    for (int iy = 0; iy <= repeat_y; ++iy)
    {
      for (int ix = 0; ix <= repeat_x; ++ix)
      {
        // Compute local-frame offset
        const Eigen::Vector3d local_offset(offset_x * ix, offset_y * iy, offset_z * iz);
        const Eigen::Vector3d world_position = original_transform.translation() + original_rotation * local_offset;

        // Compute incremental rotation (intrinsic: applied in local frame)
        const Eigen::Quaterniond incremental_rotation =
            Eigen::AngleAxisd(degToRad(rotate_z * iz), Eigen::Vector3d::UnitZ()) *
            Eigen::AngleAxisd(degToRad(rotate_y * iy), Eigen::Vector3d::UnitY()) *
            Eigen::AngleAxisd(degToRad(rotate_x * ix), Eigen::Vector3d::UnitX());
        const Eigen::Quaterniond final_rotation = original_rotation * incremental_rotation;

        // Build the output pose
        geometry_msgs::msg::PoseStamped grid_pose;
        grid_pose.header = pose.header;
        grid_pose.pose.position.x = world_position.x();
        grid_pose.pose.position.y = world_position.y();
        grid_pose.pose.position.z = world_position.z();
        grid_pose.pose.orientation = tf2::toMsg(final_rotation);

        output_poses.push_back(grid_pose);
      }
    }
  }

  setOutput(kPortPoses, output_poses);
  return BT::NodeStatus::SUCCESS;
}

}  // namespace ur_behaviors
