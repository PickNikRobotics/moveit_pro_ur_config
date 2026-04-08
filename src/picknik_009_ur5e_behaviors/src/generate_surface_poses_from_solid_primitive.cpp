#include <picknik_009_ur5e_behaviors/generate_surface_poses_from_solid_primitive.hpp>

#include <algorithm>
#include <cmath>
#include <functional>
#include <numbers>
#include <string>
#include <vector>

#include <Eigen/Geometry>

#include <spdlog/spdlog.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <tf2_eigen/tf2_eigen.hpp>

namespace
{
constexpr auto kPortIDSolidPrimitive = "solid_primitive";
constexpr auto kPortIDPrimitivePose = "primitive_pose";
constexpr auto kPortIDAngularFaceResolution = "angular_face_resolution";
constexpr auto kPortIDZOffset = "z_offset";
constexpr auto kPortIDPoses = "poses";

inline constexpr auto kDescription = R"(
    <p>
        Generates surface-aligned <b>geometry_msgs::msg::PoseStamped</b> samples for a
        <b>shape_msgs::msg::SolidPrimitive</b>.
    </p>
    <p>
        The primitive pose is required because <b>SolidPrimitive</b> does not carry frame information.
        Generated poses are first built in the primitive local frame and then transformed into the
        frame stored in <b>primitive_pose.header.frame_id</b>.
    </p>
    <p>
        Sampling strategy:
        <ul>
            <li><b>box</b>: the 6 face centroids</li>
            <li><b>sphere</b>: 3 great circles on the X=0, Y=0, and Z=0 planes</li>
            <li><b>cylinder</b>: top and bottom face centroids plus samples around the curved surface at Z=0</li>
        </ul>
    </p>
    <p>
        Each pose is oriented so its local +Z axis points toward the primitive centroid, then translated
        by <b>z_offset</b> along local -Z to back away from the surface.
    </p>
)";

constexpr double kAxisParallelThreshold = 0.9;
constexpr double kDuplicateTolerance = 1e-9;

[[nodiscard]] double degToRad(const double degrees)
{
  return degrees * std::numbers::pi_v<double> / 180.0;
}

[[nodiscard]] Eigen::Vector3d chooseReferenceAxis(const Eigen::Vector3d& z_axis)
{
  if (std::abs(z_axis.dot(Eigen::Vector3d::UnitZ())) < kAxisParallelThreshold)
  {
    return Eigen::Vector3d::UnitZ();
  }
  if (std::abs(z_axis.dot(Eigen::Vector3d::UnitY())) < kAxisParallelThreshold)
  {
    return Eigen::Vector3d::UnitY();
  }
  return Eigen::Vector3d::UnitX();
}

[[nodiscard]] Eigen::Quaterniond orientationTowardCentroid(const Eigen::Vector3d& surface_point)
{
  const Eigen::Vector3d z_axis = (-surface_point).normalized();
  const Eigen::Vector3d reference_axis = chooseReferenceAxis(z_axis);
  const Eigen::Vector3d x_axis = reference_axis.cross(z_axis).normalized();
  const Eigen::Vector3d y_axis = z_axis.cross(x_axis).normalized();

  Eigen::Matrix3d rotation;
  rotation.col(0) = x_axis;
  rotation.col(1) = y_axis;
  rotation.col(2) = z_axis;
  return Eigen::Quaterniond(rotation);
}

void appendUniquePoint(std::vector<Eigen::Vector3d>& points, const Eigen::Vector3d& candidate)
{
  const auto duplicate = std::ranges::any_of(points, [&](const Eigen::Vector3d& point) {
    return (point - candidate).norm() <= kDuplicateTolerance;
  });

  if (!duplicate)
  {
    points.push_back(candidate);
  }
}

[[nodiscard]] std::vector<Eigen::Vector3d>
sampleCirclePoints(const std::function<Eigen::Vector3d(double)>& point_generator, const double angular_resolution_deg)
{
  std::vector<Eigen::Vector3d> points;

  for (double angle_deg = 0.0; angle_deg < 360.0; angle_deg += angular_resolution_deg)
  {
    appendUniquePoint(points, point_generator(degToRad(angle_deg)));
  }

  return points;
}

[[nodiscard]] bool dimensionsValid(const shape_msgs::msg::SolidPrimitive& primitive)
{
  switch (primitive.type)
  {
    case shape_msgs::msg::SolidPrimitive::BOX:
      return primitive.dimensions.size() >= 3;
    case shape_msgs::msg::SolidPrimitive::SPHERE:
      return !primitive.dimensions.empty();
    case shape_msgs::msg::SolidPrimitive::CYLINDER:
      return primitive.dimensions.size() >= 2;
    default:
      return false;
  }
}

[[nodiscard]] bool dimensionsPositive(const shape_msgs::msg::SolidPrimitive& primitive)
{
  return std::ranges::all_of(primitive.dimensions, [](const double dimension) { return dimension > 0.0; });
}

[[nodiscard]] std::vector<Eigen::Vector3d> sampleSurfacePoints(const shape_msgs::msg::SolidPrimitive& primitive,
                                                               const double angular_resolution_deg)
{
  std::vector<Eigen::Vector3d> points;

  switch (primitive.type)
  {
    case shape_msgs::msg::SolidPrimitive::BOX:
    {
      const double half_x = primitive.dimensions[shape_msgs::msg::SolidPrimitive::BOX_X] * 0.5;
      const double half_y = primitive.dimensions[shape_msgs::msg::SolidPrimitive::BOX_Y] * 0.5;
      const double half_z = primitive.dimensions[shape_msgs::msg::SolidPrimitive::BOX_Z] * 0.5;
      points = {
        Eigen::Vector3d(half_x, 0.0, 0.0),  Eigen::Vector3d(-half_x, 0.0, 0.0), Eigen::Vector3d(0.0, half_y, 0.0),
        Eigen::Vector3d(0.0, -half_y, 0.0), Eigen::Vector3d(0.0, 0.0, half_z),  Eigen::Vector3d(0.0, 0.0, -half_z),
      };
      break;
    }
    case shape_msgs::msg::SolidPrimitive::SPHERE:
    {
      const double radius = primitive.dimensions[shape_msgs::msg::SolidPrimitive::SPHERE_RADIUS];
      for (const auto& circle_points :
           { sampleCirclePoints(
                 [&](const double angle_rad) {
                   return Eigen::Vector3d(0.0, radius * std::cos(angle_rad), radius * std::sin(angle_rad));
                 },
                 angular_resolution_deg),
             sampleCirclePoints(
                 [&](const double angle_rad) {
                   return Eigen::Vector3d(radius * std::cos(angle_rad), 0.0, radius * std::sin(angle_rad));
                 },
                 angular_resolution_deg),
             sampleCirclePoints(
                 [&](const double angle_rad) {
                   return Eigen::Vector3d(radius * std::cos(angle_rad), radius * std::sin(angle_rad), 0.0);
                 },
                 angular_resolution_deg) })
      {
        for (const auto& point : circle_points)
        {
          appendUniquePoint(points, point);
        }
      }
      break;
    }
    case shape_msgs::msg::SolidPrimitive::CYLINDER:
    {
      const double height = primitive.dimensions[shape_msgs::msg::SolidPrimitive::CYLINDER_HEIGHT];
      const double radius = primitive.dimensions[shape_msgs::msg::SolidPrimitive::CYLINDER_RADIUS];
      points = {
        Eigen::Vector3d(0.0, 0.0, height * 0.5),
        Eigen::Vector3d(0.0, 0.0, -height * 0.5),
      };

      for (const auto& point : sampleCirclePoints(
               [&](const double angle_rad) {
                 return Eigen::Vector3d(radius * std::cos(angle_rad), radius * std::sin(angle_rad), 0.0);
               },
               angular_resolution_deg))
      {
        appendUniquePoint(points, point);
      }
      break;
    }
    default:
      break;
  }

  return points;
}

}  // namespace

namespace picknik_009_ur5e_behaviors
{

GenerateSurfacePosesFromSolidPrimitive::GenerateSurfacePosesFromSolidPrimitive(const std::string& name,
                                                                               const BT::NodeConfiguration& config)
  : BT::SyncActionNode(name, config)
{
}

BT::PortsList GenerateSurfacePosesFromSolidPrimitive::providedPorts()
{
  return {
    BT::InputPort<shape_msgs::msg::SolidPrimitive>(kPortIDSolidPrimitive, "{solid_primitive}", "Primitive to sample."),
    BT::InputPort<geometry_msgs::msg::PoseStamped>(kPortIDPrimitivePose, "{primitive_pose}",
                                                   "Pose of the primitive frame in the output frame. Required because "
                                                   "SolidPrimitive has no header."),
    BT::InputPort<double>(kPortIDAngularFaceResolution, "{angular_face_resolution}",
                          "Angular resolution in degrees for sphere and cylinder surface sampling."),
    BT::InputPort<double>(kPortIDZOffset, "{z_offset}",
                          "Offset applied along local -Z after orienting poses toward the centroid."),
    BT::OutputPort<std::vector<geometry_msgs::msg::PoseStamped>>(kPortIDPoses, "{poses}", "Generated surface poses."),
  };
}

BT::KeyValueVector GenerateSurfacePosesFromSolidPrimitive::metadata()
{
  return { { "subcategory", "Planning Scene" }, { "description", kDescription } };
}

BT::NodeStatus GenerateSurfacePosesFromSolidPrimitive::tick()
{
  const auto primitive_input = getInput<shape_msgs::msg::SolidPrimitive>(kPortIDSolidPrimitive);
  const auto primitive_pose_input = getInput<geometry_msgs::msg::PoseStamped>(kPortIDPrimitivePose);
  const auto angular_resolution_input = getInput<double>(kPortIDAngularFaceResolution);
  const auto z_offset_input = getInput<double>(kPortIDZOffset);

  if (!primitive_input.has_value())
  {
    spdlog::error("GenerateSurfacePosesFromSolidPrimitive: failed to get 'solid_primitive' input port.");
    return BT::NodeStatus::FAILURE;
  }
  if (!primitive_pose_input.has_value())
  {
    spdlog::error("GenerateSurfacePosesFromSolidPrimitive: failed to get 'primitive_pose' input port.");
    return BT::NodeStatus::FAILURE;
  }
  if (!angular_resolution_input.has_value())
  {
    spdlog::error("GenerateSurfacePosesFromSolidPrimitive: failed to get 'angular_face_resolution' input port.");
    return BT::NodeStatus::FAILURE;
  }
  if (!z_offset_input.has_value())
  {
    spdlog::error("GenerateSurfacePosesFromSolidPrimitive: failed to get 'z_offset' input port.");
    return BT::NodeStatus::FAILURE;
  }

  const auto& primitive = primitive_input.value();
  const auto& primitive_pose = primitive_pose_input.value();
  const double angular_resolution_deg = angular_resolution_input.value();
  const double z_offset = z_offset_input.value();

  if (primitive_pose.header.frame_id.empty())
  {
    spdlog::error("GenerateSurfacePosesFromSolidPrimitive: 'primitive_pose.header.frame_id' must not be empty.");
    return BT::NodeStatus::FAILURE;
  }

  if (angular_resolution_deg <= 0.0)
  {
    spdlog::error("GenerateSurfacePosesFromSolidPrimitive: 'angular_face_resolution' must be > 0.");
    return BT::NodeStatus::FAILURE;
  }

  if (!dimensionsValid(primitive))
  {
    spdlog::error("GenerateSurfacePosesFromSolidPrimitive: primitive dimensions are invalid for type {}.",
                  primitive.type);
    return BT::NodeStatus::FAILURE;
  }

  if (!dimensionsPositive(primitive))
  {
    spdlog::error("GenerateSurfacePosesFromSolidPrimitive: primitive dimensions must all be positive.");
    return BT::NodeStatus::FAILURE;
  }

  if (primitive.type != shape_msgs::msg::SolidPrimitive::BOX &&
      primitive.type != shape_msgs::msg::SolidPrimitive::SPHERE &&
      primitive.type != shape_msgs::msg::SolidPrimitive::CYLINDER)
  {
    spdlog::error("GenerateSurfacePosesFromSolidPrimitive: unsupported primitive type {}. Only BOX, SPHERE, and "
                  "CYLINDER are supported.",
                  primitive.type);
    return BT::NodeStatus::FAILURE;
  }

  Eigen::Isometry3d primitive_transform = Eigen::Isometry3d::Identity();
  tf2::fromMsg(primitive_pose.pose, primitive_transform);
  const Eigen::Quaterniond primitive_rotation(primitive_transform.rotation());

  const auto surface_points = sampleSurfacePoints(primitive, angular_resolution_deg);
  std::vector<geometry_msgs::msg::PoseStamped> poses;
  poses.reserve(surface_points.size());

  for (const auto& surface_point : surface_points)
  {
    const Eigen::Quaterniond local_orientation = orientationTowardCentroid(surface_point);
    const Eigen::Vector3d local_negative_z = local_orientation * Eigen::Vector3d::UnitZ();
    const Eigen::Vector3d backed_off_local_position = surface_point - (z_offset * local_negative_z);
    const Eigen::Vector3d world_position = primitive_transform * backed_off_local_position;
    const Eigen::Quaterniond world_orientation = primitive_rotation * local_orientation;

    geometry_msgs::msg::PoseStamped pose;
    pose.header = primitive_pose.header;
    pose.pose.position.x = world_position.x();
    pose.pose.position.y = world_position.y();
    pose.pose.position.z = world_position.z();
    pose.pose.orientation = tf2::toMsg(world_orientation.normalized());
    poses.push_back(pose);
  }

  setOutput(kPortIDPoses, poses);
  return BT::NodeStatus::SUCCESS;
}

}  // namespace picknik_009_ur5e_behaviors
