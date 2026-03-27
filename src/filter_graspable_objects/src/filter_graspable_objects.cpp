#include <filter_graspable_objects/filter_graspable_objects.hpp>

#include <algorithm>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <spdlog/spdlog.h>

namespace
{
constexpr auto kPortIDObjectsIn = "objects_in";
constexpr auto kPortIDObjectsOut = "objects_out";
constexpr auto kPortIDMaxLinearDimension = "max_linear_dimension";
constexpr auto kPortIDMaxVolumeCm3 = "max_volume_cm3";

constexpr auto kMarkerTopic = "/visual_markers";
constexpr auto kMarkerNamespace = "filter_graspable_objects";

visualization_msgs::msg::Marker makeBoxMarker(
    const moveit_studio_vision_msgs::msg::GraspableObject& obj,
    int id, float r, float g, float b, float a)
{
  visualization_msgs::msg::Marker m;
  m.action = visualization_msgs::msg::Marker::ADD;
  m.header = obj.object.header;
  m.header.frame_id = "world";
  m.id = id;
  m.ns = kMarkerNamespace;
  m.type = visualization_msgs::msg::Marker::CUBE;
  m.pose = obj.object.pose;
  m.color.r = r;
  m.color.g = g;
  m.color.b = b;
  m.color.a = a;
  m.lifetime.sec = 10;
  const auto& bv = obj.grasp_info.bounding_volume;
  if (bv.dimensions.size() >= 3)
  {
    m.scale.x = bv.dimensions[shape_msgs::msg::SolidPrimitive::BOX_X];
    m.scale.y = bv.dimensions[shape_msgs::msg::SolidPrimitive::BOX_Y];
    m.scale.z = bv.dimensions[shape_msgs::msg::SolidPrimitive::BOX_Z];
  }
  return m;
}
}  // namespace

namespace filter_graspable_objects
{

FilterGraspableObjects::FilterGraspableObjects(
    const std::string& name, const BT::NodeConfiguration& config,
    const std::shared_ptr<moveit_pro::behaviors::BehaviorContext>& shared_resources)
  : moveit_pro::behaviors::SharedResourcesNode<BT::SyncActionNode>(name, config, shared_resources)
{
  marker_pub_ = shared_resources->node->create_publisher<visualization_msgs::msg::MarkerArray>(
      kMarkerTopic, rclcpp::QoS(1).transient_local());
}

BT::PortsList FilterGraspableObjects::providedPorts()
{
  return BT::PortsList{
    BT::InputPort<std::vector<moveit_studio_vision_msgs::msg::GraspableObject>>(
        kPortIDObjectsIn, "{objects}", "Input vector of graspable objects to filter."),
    BT::InputPort<double>(kPortIDMaxLinearDimension, "0.20",
                          "Maximum allowed dimension along any single axis (meters)."),
    BT::InputPort<double>(kPortIDMaxVolumeCm3, "-1.0",
                          "Maximum allowed bounding volume in cubic centimeters. "
                          "Set to <= 0 to skip volume filtering."),
    BT::OutputPort<std::vector<moveit_studio_vision_msgs::msg::GraspableObject>>(
        kPortIDObjectsOut, "{filtered_objects}", "Filtered and sorted graspable objects (highest centroid Z first)."),
  };
}

BT::KeyValueVector FilterGraspableObjects::metadata()
{
  return { { "description",
             "Filters graspable objects by maximum linear dimension and optionally by volume. "
             "Results are sorted by centroid height (highest Z first). "
             "Publishes blue markers for rejected objects and green for accepted." },
           { "subcategory", "Perception - 3D" } };
}

BT::NodeStatus FilterGraspableObjects::tick()
{
  const auto objects_in =
      getInput<std::vector<moveit_studio_vision_msgs::msg::GraspableObject>>(kPortIDObjectsIn);
  const auto max_linear_dim = getInput<double>(kPortIDMaxLinearDimension);
  const auto max_volume_cm3 = getInput<double>(kPortIDMaxVolumeCm3);

  if (!objects_in.has_value() || !max_linear_dim.has_value())
  {
    spdlog::error("FilterGraspableObjects: failed to get required input ports.");
    return BT::NodeStatus::FAILURE;
  }

  const double max_dim = max_linear_dim.value();
  const double max_vol = max_volume_cm3.has_value() ? max_volume_cm3.value() : -1.0;

  spdlog::warn("=== FilterGraspableObjects: {} input objects, max_dim={:.3f}m, max_vol={:.1f}cm3 ===",
               objects_in.value().size(), max_dim, max_vol);

  std::vector<moveit_studio_vision_msgs::msg::GraspableObject> accepted;
  std::vector<moveit_studio_vision_msgs::msg::GraspableObject> rejected;

  int idx = 0;
  for (const auto& obj : objects_in.value())
  {
    const auto& bv = obj.grasp_info.bounding_volume;

    if (bv.type != shape_msgs::msg::SolidPrimitive::BOX || bv.dimensions.size() < 3)
    {
      spdlog::warn("  [{}] NON-BOX type={}, REJECTED", idx, bv.type);
      rejected.push_back(obj);
      ++idx;
      continue;
    }

    const double dx = bv.dimensions[shape_msgs::msg::SolidPrimitive::BOX_X];
    const double dy = bv.dimensions[shape_msgs::msg::SolidPrimitive::BOX_Y];
    const double dz = bv.dimensions[shape_msgs::msg::SolidPrimitive::BOX_Z];
    const double largest_dim = std::max({ dx, dy, dz });
    const double volume_cm3 = dx * dy * dz * 1e6;

    spdlog::warn("  [{}] dims: {:.4f} x {:.4f} x {:.4f} m, largest={:.4f}m, vol={:.1f}cm3, pos=({:.3f},{:.3f},{:.3f})",
                 idx, dx, dy, dz, largest_dim, volume_cm3,
                 obj.object.pose.position.x, obj.object.pose.position.y, obj.object.pose.position.z);

    bool reject = false;
    if (largest_dim > max_dim)
    {
      spdlog::warn("       -> REJECTED: largest dim {:.4f}m > max {:.3f}m", largest_dim, max_dim);
      reject = true;
    }
    else if (max_vol > 0.0 && volume_cm3 > max_vol)
    {
      spdlog::warn("       -> REJECTED: volume {:.1f}cm3 > max {:.1f}cm3", volume_cm3, max_vol);
      reject = true;
    }
    else
    {
      spdlog::warn("       -> ACCEPTED");
    }

    if (reject)
    {
      rejected.push_back(obj);
    }
    else
    {
      accepted.push_back(obj);
    }
    ++idx;
  }

  // Sort accepted by centroid Z height (highest first)
  std::sort(accepted.begin(), accepted.end(),
            [](const moveit_studio_vision_msgs::msg::GraspableObject& a,
               const moveit_studio_vision_msgs::msg::GraspableObject& b) {
              return a.object.pose.position.z > b.object.pose.position.z;
            });

  // Publish visualization markers
  visualization_msgs::msg::MarkerArray marker_array;
  int marker_id = 5000;

  // Clear previous filter markers
  visualization_msgs::msg::Marker delete_marker;
  delete_marker.action = visualization_msgs::msg::Marker::DELETEALL;
  delete_marker.ns = kMarkerNamespace;
  marker_array.markers.push_back(delete_marker);

  // Only show accepted objects (green transparent)
  for (const auto& obj : accepted)
  {
    marker_array.markers.push_back(makeBoxMarker(obj, marker_id++, 0.2f, 1.0f, 0.4f, 0.4f));
  }

  marker_pub_->publish(marker_array);

  spdlog::warn("=== FilterGraspableObjects: {} ACCEPTED, {} REJECTED ===", accepted.size(), rejected.size());

  if (accepted.empty())
  {
    spdlog::warn("FilterGraspableObjects: NO objects passed the filter!");
    return BT::NodeStatus::FAILURE;
  }

  setOutput(kPortIDObjectsOut, accepted);
  return BT::NodeStatus::SUCCESS;
}

}  // namespace filter_graspable_objects
