#include <ur_behaviors/get_placement_pose_for_object.hpp>

#include <yaml-cpp/yaml.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit_pro_behavior_interface/get_required_ports.hpp>
#include <moveit_pro_behavior_interface/metadata_fields.hpp>
#include <tf2_eigen/tf2_eigen.hpp>

#include <Eigen/Geometry>
#include <string>
#include <vector>

namespace
{
inline constexpr auto kDescription = R"(
    <p>Looks up an object's designated placement zone from placement_zones.yaml and returns the world-frame pose.</p>
    <p>Searches both top and bottom shelves for the given object name.</p>
)";

constexpr auto kPortIDTopShelfPose = "top_shelf_pose";
constexpr auto kPortIDBottomShelfPose = "bottom_shelf_pose";
constexpr auto kPortIDConfigFile = "config_file";
constexpr auto kPortIDObjectName = "object_name";
constexpr auto kPortIDPlacementPose = "placement_pose";
constexpr auto kPortIDShelfName = "shelf_name";

// Shelf geometry constants — must match VisualizePlacementZones
constexpr double kShelfWidth = 1.27;
constexpr int kSlotsPerShelf = 6;
constexpr double kSlotSpacing = kShelfWidth / kSlotsPerShelf;
constexpr double kDepthOffsetShelfFrame = -0.15;

double slotCenterX(int slot_index)
{
  return -kShelfWidth / 2.0 + kSlotSpacing / 2.0 + slot_index * kSlotSpacing;
}

}  // namespace

namespace ur_behaviors
{

GetPlacementPoseForObject::GetPlacementPoseForObject(
    const std::string& name, const BT::NodeConfiguration& config,
    const std::shared_ptr<moveit_pro::behaviors::BehaviorContext>& shared_resources)
  : SharedResourcesNode(name, config, shared_resources)
{
}

BT::PortsList GetPlacementPoseForObject::providedPorts()
{
  return { BT::InputPort<geometry_msgs::msg::PoseStamped>(kPortIDTopShelfPose, "Calibrated top shelf pose"),
           BT::InputPort<geometry_msgs::msg::PoseStamped>(kPortIDBottomShelfPose, "Calibrated bottom shelf pose"),
           BT::InputPort<std::string>(kPortIDConfigFile, "../config/placement_zones.yaml",
                                      "Path to placement zones YAML config (relative to config source directory)"),
           BT::InputPort<std::string>(kPortIDObjectName, "pringles",
                                      "Name of the object to look up in placement_zones.yaml"),
           BT::OutputPort<geometry_msgs::msg::PoseStamped>(kPortIDPlacementPose, "{placement_pose}",
                                                           "World-frame placement pose for the requested object"),
           BT::OutputPort<std::string>(kPortIDShelfName, "{shelf_name}",
                                       "Which shelf the object is on: 'top_shelf' or 'bottom_shelf'") };
}

BT::KeyValueVector GetPlacementPoseForObject::metadata()
{
  return { { moveit_pro::behaviors::kDescriptionMetadataKey, kDescription },
           { moveit_pro::behaviors::kSubcategoryMetadataKey, "Application - Grocery" } };
}

BT::NodeStatus GetPlacementPoseForObject::tick()
{
  const auto ports =
      moveit_pro::behaviors::getRequiredInputs(getInput<geometry_msgs::msg::PoseStamped>(kPortIDTopShelfPose),
                                               getInput<geometry_msgs::msg::PoseStamped>(kPortIDBottomShelfPose),
                                               getInput<std::string>(kPortIDConfigFile),
                                               getInput<std::string>(kPortIDObjectName));

  if (!ports.has_value())
  {
    shared_resources_->logger->publishFailureMessage(name(), "Failed to get required value from input data port: " +
                                                                 ports.error());
    return BT::NodeStatus::FAILURE;
  }

  const auto& [top_shelf_pose, bottom_shelf_pose, config_file, object_name] = ports.value();

  // Resolve config file path
  std::string config_source_directory;
  try
  {
    config_source_directory = shared_resources_->node->get_parameter("config_source_directory").as_string();
  }
  catch (const std::exception& e)
  {
    shared_resources_->logger->publishFailureMessage(
        name(), std::string("Failed to get config_source_directory parameter: ") + e.what());
    return BT::NodeStatus::FAILURE;
  }

  const std::string full_config_path = config_source_directory + "/" + config_file;

  // Load and parse YAML
  YAML::Node yaml;
  try
  {
    yaml = YAML::LoadFile(full_config_path);
  }
  catch (const std::exception& e)
  {
    shared_resources_->logger->publishFailureMessage(name(), "Failed to load placement zones from '" +
                                                                 full_config_path + "': " + e.what());
    return BT::NodeStatus::FAILURE;
  }

  // Search both shelves for the object name
  std::string found_shelf;
  int found_slot = -1;

  for (const auto& shelf_name : { "top_shelf", "bottom_shelf" })
  {
    if (!yaml[shelf_name])
      continue;
    for (int i = 0; i < kSlotsPerShelf; ++i)
    {
      const std::string key = "slot_" + std::to_string(i + 1);
      if (yaml[shelf_name][key] && yaml[shelf_name][key].as<std::string>() == object_name)
      {
        found_shelf = shelf_name;
        found_slot = i;
        break;
      }
    }
    if (found_slot >= 0)
      break;
  }

  if (found_slot < 0)
  {
    shared_resources_->logger->publishFailureMessage(
        name(), "Object '" + object_name + "' not found in placement zones config: " + full_config_path);
    return BT::NodeStatus::FAILURE;
  }

  // Select the correct shelf transform
  const auto& shelf_pose = (found_shelf == "top_shelf") ? top_shelf_pose : bottom_shelf_pose;
  Eigen::Isometry3d shelf_transform;
  tf2::fromMsg(shelf_pose.pose, shelf_transform);

  // Compute slot position in shelf frame and transform to world
  const Eigen::Vector3d slot_in_shelf(slotCenterX(found_slot), kDepthOffsetShelfFrame, 0.0);
  const Eigen::Isometry3d slot_transform = shelf_transform * Eigen::Translation3d(slot_in_shelf);

  // Build output pose
  geometry_msgs::msg::PoseStamped placement_pose;
  placement_pose.header.frame_id = "world";
  placement_pose.header.stamp = shared_resources_->node->now();
  placement_pose.pose = tf2::toMsg(slot_transform);

  setOutput(kPortIDPlacementPose, placement_pose);
  setOutput(kPortIDShelfName, found_shelf);

  shared_resources_->logger->publishInfoMessage(name(), "Found '" + object_name + "' at " + found_shelf + " slot " +
                                                            std::to_string(found_slot + 1) + ". Placement pose set.");

  return BT::NodeStatus::SUCCESS;
}

}  // namespace ur_behaviors
