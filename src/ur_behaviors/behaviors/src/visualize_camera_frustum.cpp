#include <ur_behaviors/visualize_camera_frustum.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit_pro_behavior_interface/get_required_ports.hpp>
#include <moveit_pro_behavior_interface/metadata_fields.hpp>
#include <tf2_eigen/tf2_eigen.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <Eigen/Geometry>
#include <array>
#include <cmath>

namespace
{
inline constexpr auto kDescriptionVisualizeCameraFrustum = R"(
    <p>Visualizes a camera frustum by publishing LINE_LIST markers.</p>
    <p>The camera pose uses the optical frame convention: Z forward (viewing direction), X right, Y down.</p>
)";

constexpr auto kPortIDTopicName = "topic_name";
constexpr auto kPortIDCameraPose = "camera_pose";
constexpr auto kPortIDNearPlane = "near_plane";
constexpr auto kPortIDFarPlane = "far_plane";
constexpr auto kPortIDHorizontalFov = "horizontal_fov";
constexpr auto kPortIDVerticalFov = "vertical_fov";
constexpr auto kPortIDMarkerNamespace = "marker_namespace";

constexpr auto kDefaultMarkerTopic = "/visual_markers";
constexpr auto kDefaultNamespace = "camera_frustum";

inline double degToRad(double degrees) {
  return degrees * M_PI / 180.0;
}

struct FrustumCorners {
  std::array<Eigen::Vector3d, 8> corners;
};

FrustumCorners calculateFrustumCorners(double near_plane, double far_plane,
                                      double h_fov_deg, double v_fov_deg) {
  const double h_fov_rad = degToRad(h_fov_deg);
  const double v_fov_rad = degToRad(v_fov_deg);

  const double half_width_near = near_plane * std::tan(h_fov_rad / 2.0);
  const double half_height_near = near_plane * std::tan(v_fov_rad / 2.0);
  const double half_width_far = far_plane * std::tan(h_fov_rad / 2.0);
  const double half_height_far = far_plane * std::tan(v_fov_rad / 2.0);

  FrustumCorners result;
  // Near plane corners (indices 0-3): top-left, top-right, bottom-right, bottom-left
  result.corners[0] = Eigen::Vector3d(-half_width_near, -half_height_near, near_plane);
  result.corners[1] = Eigen::Vector3d(+half_width_near, -half_height_near, near_plane);
  result.corners[2] = Eigen::Vector3d(+half_width_near, +half_height_near, near_plane);
  result.corners[3] = Eigen::Vector3d(-half_width_near, +half_height_near, near_plane);

  // Far plane corners (indices 4-7): top-left, top-right, bottom-right, bottom-left
  result.corners[4] = Eigen::Vector3d(-half_width_far, -half_height_far, far_plane);
  result.corners[5] = Eigen::Vector3d(+half_width_far, -half_height_far, far_plane);
  result.corners[6] = Eigen::Vector3d(+half_width_far, +half_height_far, far_plane);
  result.corners[7] = Eigen::Vector3d(-half_width_far, +half_height_far, far_plane);

  return result;
}

[[nodiscard]] tl::expected<void, std::string> validateParameters(double near_plane, double far_plane,
                                                                   double horizontal_fov, double vertical_fov)
{
  if (near_plane <= 0.0)
  {
    return tl::unexpected(std::string("Near plane must be greater than 0"));
  }

  if (far_plane <= near_plane)
  {
    return tl::unexpected(std::string("Far plane must be greater than near plane"));
  }

  if (horizontal_fov <= 0.0 || horizontal_fov >= 180.0)
  {
    return tl::unexpected(std::string("Horizontal FOV must be in range (0, 180) degrees"));
  }

  if (vertical_fov <= 0.0 || vertical_fov >= 180.0)
  {
    return tl::unexpected(std::string("Vertical FOV must be in range (0, 180) degrees"));
  }

  return {};
}

geometry_msgs::msg::Point eigenToPoint(const Eigen::Vector3d& vec) {
  geometry_msgs::msg::Point point;
  point.x = vec.x();
  point.y = vec.y();
  point.z = vec.z();
  return point;
}

}  // namespace

namespace ur_behaviors
{

VisualizeCameraFrustum::VisualizeCameraFrustum(
    const std::string& name,
    const BT::NodeConfiguration& config,
    const std::shared_ptr<moveit_pro::behaviors::BehaviorContext>& shared_resources)
  : SharedResourcesNode(name, config, shared_resources)
  , publisher_{ shared_resources_->node->create_publisher<visualization_msgs::msg::MarkerArray>(
        kDefaultMarkerTopic, rclcpp::SystemDefaultsQoS()) }
{
}

BT::PortsList VisualizeCameraFrustum::providedPorts()
{
  return {
    BT::InputPort<std::string>(kPortIDTopicName, kDefaultMarkerTopic,
      "Topic to publish visualization markers"),
    BT::InputPort<geometry_msgs::msg::PoseStamped>(kPortIDCameraPose,
      "Camera pose (Z forward, X right, Y down)"),
    BT::InputPort<double>(kPortIDNearPlane,
      "Near plane distance in meters"),
    BT::InputPort<double>(kPortIDFarPlane,
      "Far plane distance in meters"),
    BT::InputPort<double>(kPortIDHorizontalFov,
      "Horizontal field of view in degrees"),
    BT::InputPort<double>(kPortIDVerticalFov,
      "Vertical field of view in degrees"),
    BT::InputPort<std::string>(kPortIDMarkerNamespace, kDefaultNamespace,
      "Namespace for markers")
  };
}

BT::KeyValueVector VisualizeCameraFrustum::metadata()
{
  return {
    { moveit_pro::behaviors::kDescriptionMetadataKey, kDescriptionVisualizeCameraFrustum }
  };
}

BT::NodeStatus VisualizeCameraFrustum::tick()
{
  // Get required inputs from ports
  const auto ports = moveit_pro::behaviors::getRequiredInputs(
      getInput<std::string>(kPortIDTopicName),
      getInput<geometry_msgs::msg::PoseStamped>(kPortIDCameraPose),
      getInput<double>(kPortIDNearPlane),
      getInput<double>(kPortIDFarPlane),
      getInput<double>(kPortIDHorizontalFov),
      getInput<double>(kPortIDVerticalFov),
      getInput<std::string>(kPortIDMarkerNamespace));

  if (!ports.has_value())
  {
    shared_resources_->logger->publishFailureMessage(name(),
        "Failed to get required value from input data port: " + ports.error());
    return BT::NodeStatus::FAILURE;
  }

  const auto& [topic_name, camera_pose, near_plane, far_plane,
               horizontal_fov, vertical_fov, marker_namespace] = ports.value();

  // Validate parameter ranges
  if (const auto validation_result = validateParameters(near_plane, far_plane, horizontal_fov, vertical_fov);
      !validation_result.has_value())
  {
    shared_resources_->logger->publishFailureMessage(name(), validation_result.error());
    return BT::NodeStatus::FAILURE;
  }

  // Calculate frustum corners in camera frame
  const auto frustum_corners = calculateFrustumCorners(
    near_plane, far_plane, horizontal_fov, vertical_fov);

  // Convert camera pose to Eigen transform
  Eigen::Isometry3d camera_transform;
  tf2::fromMsg(camera_pose.pose, camera_transform);

  // Transform corners to world frame
  std::array<Eigen::Vector3d, 8> world_corners;
  for (size_t i = 0; i < 8; ++i)
  {
    world_corners[i] = camera_transform * frustum_corners.corners[i];
  }

  // Create marker array
  visualization_msgs::msg::MarkerArray marker_array;

  // Create LINE_LIST marker for frustum edges
  visualization_msgs::msg::Marker line_marker;
  line_marker.header = camera_pose.header;
  line_marker.header.stamp = rclcpp::Time(0);
  line_marker.ns = marker_namespace;
  line_marker.id = 1;
  line_marker.type = visualization_msgs::msg::Marker::LINE_LIST;
  line_marker.action = visualization_msgs::msg::Marker::ADD;
  line_marker.pose.orientation.w = 1.0;
  line_marker.scale.x = 0.005;
  line_marker.color.r = 0.0;
  line_marker.color.g = 1.0;
  line_marker.color.b = 0.0;
  line_marker.color.a = 0.7;
  line_marker.lifetime = rclcpp::Duration(0, 0);

  // Near plane edges (4 edges)
  line_marker.points.push_back(eigenToPoint(world_corners[0]));
  line_marker.points.push_back(eigenToPoint(world_corners[1]));

  line_marker.points.push_back(eigenToPoint(world_corners[1]));
  line_marker.points.push_back(eigenToPoint(world_corners[2]));

  line_marker.points.push_back(eigenToPoint(world_corners[2]));
  line_marker.points.push_back(eigenToPoint(world_corners[3]));

  line_marker.points.push_back(eigenToPoint(world_corners[3]));
  line_marker.points.push_back(eigenToPoint(world_corners[0]));

  // Far plane edges (4 edges)
  line_marker.points.push_back(eigenToPoint(world_corners[4]));
  line_marker.points.push_back(eigenToPoint(world_corners[5]));

  line_marker.points.push_back(eigenToPoint(world_corners[5]));
  line_marker.points.push_back(eigenToPoint(world_corners[6]));

  line_marker.points.push_back(eigenToPoint(world_corners[6]));
  line_marker.points.push_back(eigenToPoint(world_corners[7]));

  line_marker.points.push_back(eigenToPoint(world_corners[7]));
  line_marker.points.push_back(eigenToPoint(world_corners[4]));

  // Connecting edges (4 edges)
  line_marker.points.push_back(eigenToPoint(world_corners[0]));
  line_marker.points.push_back(eigenToPoint(world_corners[4]));

  line_marker.points.push_back(eigenToPoint(world_corners[1]));
  line_marker.points.push_back(eigenToPoint(world_corners[5]));

  line_marker.points.push_back(eigenToPoint(world_corners[2]));
  line_marker.points.push_back(eigenToPoint(world_corners[6]));

  line_marker.points.push_back(eigenToPoint(world_corners[3]));
  line_marker.points.push_back(eigenToPoint(world_corners[7]));

  marker_array.markers.push_back(line_marker);

  // Publish the marker array
  publisher_->publish(marker_array);

  shared_resources_->logger->publishInfoMessage(name(),
      "Published camera frustum visualization to topic: " + topic_name);

  return BT::NodeStatus::SUCCESS;
}

}  // namespace ur_behaviors
