#include <ur_behaviors/visualize_placement_zones.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit_pro_behavior_interface/get_required_ports.hpp>
#include <moveit_pro_behavior_interface/metadata_fields.hpp>
#include <tf2_eigen/tf2_eigen.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <yaml-cpp/yaml.h>

#include <Eigen/Geometry>
#include <algorithm>
#include <cmath>
#include <fstream>
#include <string>
#include <vector>

namespace
{
inline constexpr auto kDescriptionVisualizePlacementZones = R"(
    <p>Reads placement_zones.yaml and visualizes labeled placement positions on each shelf.</p>
    <p>Each shelf has 6 evenly spaced slots across the 1.27m shelf width.</p>
)";

constexpr auto kPortIDTopShelfPose = "top_shelf_pose";
constexpr auto kPortIDBottomShelfPose = "bottom_shelf_pose";
constexpr auto kPortIDConfigFile = "config_file";

constexpr auto kDefaultMarkerTopic = "/visual_markers";

// Shelf geometry constants
constexpr double kShelfWidth = 1.27;       // meters
constexpr int kSlotsPerShelf = 6;
constexpr double kSlotSpacing = kShelfWidth / kSlotsPerShelf;  // ~0.2117m
// Slot centers in shelf X: from -(width/2 - spacing/2) to +(width/2 - spacing/2)
// Depth offset in shelf frame: -0.20 (center of 0.40m depth)
constexpr double kDepthOffsetShelfFrame = -0.15;

// Compute slot center X positions in shelf frame
double slotCenterX(int slot_index)
{
  // slot_index 0..5, left to right in shelf frame
  return -kShelfWidth / 2.0 + kSlotSpacing / 2.0 + slot_index * kSlotSpacing;
}

struct PlacementConfig
{
  std::vector<std::string> top_shelf;     // 6 product names
  std::vector<std::string> bottom_shelf;  // 6 product names
};

PlacementConfig loadConfig(const std::string& file_path)
{
  PlacementConfig config;
  config.top_shelf.resize(kSlotsPerShelf, "empty");
  config.bottom_shelf.resize(kSlotsPerShelf, "empty");

  try
  {
    YAML::Node yaml = YAML::LoadFile(file_path);

    if (yaml["top_shelf"])
    {
      for (int i = 0; i < kSlotsPerShelf; ++i)
      {
        std::string key = "slot_" + std::to_string(i + 1);
        if (yaml["top_shelf"][key])
        {
          config.top_shelf[i] = yaml["top_shelf"][key].as<std::string>();
        }
      }
    }

    if (yaml["bottom_shelf"])
    {
      for (int i = 0; i < kSlotsPerShelf; ++i)
      {
        std::string key = "slot_" + std::to_string(i + 1);
        if (yaml["bottom_shelf"][key])
        {
          config.bottom_shelf[i] = yaml["bottom_shelf"][key].as<std::string>();
        }
      }
    }
  }
  catch (const std::exception& e)
  {
    // Log error but return defaults with "empty" labels
    RCLCPP_ERROR(rclcpp::get_logger("VisualizePlacementZones"),
        "Failed to load placement zones from '%s': %s", file_path.c_str(), e.what());
  }

  return config;
}

}  // namespace

namespace ur_behaviors
{

VisualizePlacementZones::VisualizePlacementZones(
    const std::string& name,
    const BT::NodeConfiguration& config,
    const std::shared_ptr<moveit_pro::behaviors::BehaviorContext>& shared_resources)
  : SharedResourcesNode(name, config, shared_resources)
  , publisher_{ shared_resources_->node->create_publisher<visualization_msgs::msg::MarkerArray>(
        kDefaultMarkerTopic, rclcpp::SystemDefaultsQoS()) }
  , text_publisher_{ shared_resources_->node->create_publisher<visualization_msgs::msg::MarkerArray>(
        "/placement_zone_labels", rclcpp::SystemDefaultsQoS()) }
{
}

BT::PortsList VisualizePlacementZones::providedPorts()
{
  return {
    BT::InputPort<geometry_msgs::msg::PoseStamped>(kPortIDTopShelfPose,
      "Calibrated top shelf pose"),
    BT::InputPort<geometry_msgs::msg::PoseStamped>(kPortIDBottomShelfPose,
      "Calibrated bottom shelf pose"),
    BT::InputPort<std::string>(kPortIDConfigFile, "../config/placement_zones.yaml",
      "Path to placement zones YAML config (relative to config source directory)")
  };
}

BT::KeyValueVector VisualizePlacementZones::metadata()
{
  return {
    { moveit_pro::behaviors::kDescriptionMetadataKey, kDescriptionVisualizePlacementZones },
    { moveit_pro::behaviors::kSubcategoryMetadataKey, "Application - Grocery" }
  };
}

BT::NodeStatus VisualizePlacementZones::tick()
{
  const auto ports = moveit_pro::behaviors::getRequiredInputs(
      getInput<geometry_msgs::msg::PoseStamped>(kPortIDTopShelfPose),
      getInput<geometry_msgs::msg::PoseStamped>(kPortIDBottomShelfPose),
      getInput<std::string>(kPortIDConfigFile));

  if (!ports.has_value())
  {
    shared_resources_->logger->publishFailureMessage(name(),
        "Failed to get required value from input data port: " + ports.error());
    return BT::NodeStatus::FAILURE;
  }

  const auto& [top_shelf_pose, bottom_shelf_pose, config_file] = ports.value();

  // Resolve config file path relative to the config source directory
  std::string config_source_directory;
  try
  {
    config_source_directory =
        shared_resources_->node->get_parameter("config_source_directory").as_string();
  }
  catch (const std::exception& e)
  {
    shared_resources_->logger->publishFailureMessage(name(),
        std::string("Failed to get config_source_directory parameter: ") + e.what());
    return BT::NodeStatus::FAILURE;
  }

  // config_source_directory points to the objectives directory.
  // Go up one level to reach the package root.
  const std::string full_config_path = config_source_directory + "/" + config_file;

  shared_resources_->logger->publishInfoMessage(name(),
      "Loading placement zones from: " + full_config_path);

  // Load placement zone configuration
  const auto config = loadConfig(full_config_path);

  // Log what we loaded for debugging
  shared_resources_->logger->publishInfoMessage(name(),
      "Top slot 1: " + config.top_shelf[0] + ", Bottom slot 1: " + config.bottom_shelf[0]);

  // Get shelf transforms
  Eigen::Isometry3d top_shelf_transform, bottom_shelf_transform;
  tf2::fromMsg(top_shelf_pose.pose, top_shelf_transform);
  tf2::fromMsg(bottom_shelf_pose.pose, bottom_shelf_transform);

  visualization_msgs::msg::MarkerArray marker_array;

  int marker_id = 100;  // Start at 100 to avoid ID conflicts with other marker publishers

  constexpr double kAxisLength = 0.05;
  constexpr double kAxisWidth = 0.005;

  // Helper to create an axis line from origin to tip
  auto makeAxisLine = [&](const Eigen::Vector3d& origin, const Eigen::Vector3d& tip,
                          float r, float g, float b) {
    visualization_msgs::msg::Marker line;
    line.header.frame_id = "world";
    line.header.stamp = shared_resources_->node->now();
    line.ns = "placement_zones";
    line.id = marker_id++;
    line.type = visualization_msgs::msg::Marker::LINE_LIST;
    line.action = visualization_msgs::msg::Marker::ADD;
    line.pose.orientation.w = 1.0;
    line.scale.x = kAxisWidth;
    line.color.r = r;
    line.color.g = g;
    line.color.b = b;
    line.color.a = 1.0;
    line.lifetime = rclcpp::Duration(0, 0);
    geometry_msgs::msg::Point p1, p2;
    p1.x = origin.x(); p1.y = origin.y(); p1.z = origin.z();
    p2.x = tip.x(); p2.y = tip.y(); p2.z = tip.z();
    line.points.push_back(p1);
    line.points.push_back(p2);
    marker_array.markers.push_back(line);
  };

  // Helper lambda to create axes + text label at a slot position
  auto createSlotMarker = [&](const Eigen::Isometry3d& shelf_transform,
                               int slot_index, const std::string& label,
                               const std::string& prefix) {
    // Compute slot position in shelf frame
    double x_shelf = slotCenterX(slot_index);
    double y_shelf = kDepthOffsetShelfFrame;
    double z_shelf = 0.0;  // On shelf surface

    // Transform position and axes to world frame
    Eigen::Vector3d slot_in_shelf(x_shelf, y_shelf, z_shelf);
    Eigen::Vector3d slot_in_world = shelf_transform * slot_in_shelf;

    // Get the shelf rotation to orient the axes
    Eigen::Matrix3d rotation = shelf_transform.rotation();
    Eigen::Vector3d x_axis = rotation * Eigen::Vector3d(kAxisLength, 0, 0);
    Eigen::Vector3d y_axis = rotation * Eigen::Vector3d(0, kAxisLength, 0);
    Eigen::Vector3d z_axis = rotation * Eigen::Vector3d(0, 0, kAxisLength);

    std::string slot_ns = prefix + std::to_string(slot_index + 1);
    // X axis (red)
    makeAxisLine(slot_in_world, slot_in_world + x_axis, 1.0, 0.0, 0.0);
    // Override namespace for the last 3 markers to match this slot
    marker_array.markers.back().ns = slot_ns;
    marker_array.markers.back().id = 0;
    // Y axis (green)
    makeAxisLine(slot_in_world, slot_in_world + y_axis, 0.0, 1.0, 0.0);
    marker_array.markers.back().ns = slot_ns;
    marker_array.markers.back().id = 1;
    // Z axis (blue)
    makeAxisLine(slot_in_world, slot_in_world + z_axis, 0.0, 0.0, 1.0);
    marker_array.markers.back().ns = slot_ns;
    marker_array.markers.back().id = 2;

    // Text label — using same format as VisualizePose / marker_utils::createTextMarker
    // The MoveIt Pro UI requires this specific orientation and scale to render text.
    visualization_msgs::msg::Marker text_marker;
    text_marker.header.frame_id = "world";
    text_marker.header.stamp = shared_resources_->node->now();
    text_marker.ns = prefix + std::to_string(slot_index + 1);
    text_marker.id = 3;  // Same ID as VisualizePose uses for text (after 3 axis lines at 0,1,2)
    text_marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    text_marker.action = visualization_msgs::msg::Marker::ADD;
    // Offset text slightly from the axis origin, matching VisualizePose
    Eigen::Vector3d text_offset = rotation * Eigen::Vector3d(kAxisLength * 0.4, 0, kAxisLength * 0.4);
    text_marker.pose.position.x = slot_in_world.x() + text_offset.x();
    text_marker.pose.position.y = slot_in_world.y() + text_offset.y();
    text_marker.pose.position.z = slot_in_world.z() + text_offset.z();
    text_marker.pose.orientation.x = 0.707;
    text_marker.pose.orientation.y = 0.0;
    text_marker.pose.orientation.z = 0.0;
    text_marker.pose.orientation.w = 0.707;
    text_marker.text = prefix + std::to_string(slot_index + 1) + ": " + label;
    text_marker.scale.x = 1.0;
    text_marker.scale.y = 1.0;
    text_marker.scale.z = std::max(0.05, kAxisLength * 1.0);
    text_marker.color.r = 1.0;
    text_marker.color.g = 1.0;
    text_marker.color.b = 1.0;
    text_marker.color.a = 1.0;
    text_marker.lifetime = rclcpp::Duration(0, 0);
    marker_array.markers.push_back(text_marker);
  };

  // Create markers for top shelf
  for (int i = 0; i < kSlotsPerShelf; ++i)
  {
    createSlotMarker(top_shelf_transform, i, config.top_shelf[i], "T");
  }

  // Create markers for bottom shelf
  for (int i = 0; i < kSlotsPerShelf; ++i)
  {
    createSlotMarker(bottom_shelf_transform, i, config.bottom_shelf[i], "B");
  }

  publisher_->publish(marker_array);

  shared_resources_->logger->publishInfoMessage(name(),
      "Published " + std::to_string(kSlotsPerShelf * 2) + " placement zone markers to /visual_markers and labels to /placement_zone_labels from " + full_config_path);

  return BT::NodeStatus::SUCCESS;
}

}  // namespace ur_behaviors
