// Copyright 2026 PickNik Inc.
// All rights reserved.
//
// Unauthorized copying of this code base via any medium is strictly prohibited.
// Proprietary and confidential.

#include <generate_shape_urdf/generate_shape_urdf.hpp>

#include <algorithm>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

#include <fmt/format.h>
#include <spdlog/spdlog.h>
#include <Eigen/Dense>
#include <geometry_msgs/msg/pose_stamped.hpp>

namespace
{
constexpr auto kPortIDShapeType = "shape_type";
constexpr auto kPortIDDimensions = "dimensions";
constexpr auto kPortIDUrdfString = "urdf_string";
constexpr auto kPortIDUrdfFilePath = "urdf_file_path";
constexpr auto kPortIDObjectName = "object_name";
constexpr auto kPortIDOriginOffsetZ = "origin_offset_z";
constexpr auto kPortIDOriginRPY = "origin_rpy";
constexpr auto kPortIDObjectPose = "object_pose";
constexpr auto kPortIDGraspPose = "grasp_pose";
constexpr auto kDefaultUrdfFilePath = "/tmp/generated_collision.urdf";

inline constexpr auto kDescription = R"(
    <p>
        Generates a URDF string for a primitive shape (box, sphere, or cylinder)
        from a shape type and dimension parameters.
    </p>
    <p>
        Dimension requirements per shape type:
        <ul>
            <li><b>box</b>: [width, height, depth] (3 values in meters)</li>
            <li><b>sphere</b>: [radius] (1 value in meters)</li>
            <li><b>cylinder</b>: [radius, height] (2 values in meters)</li>
        </ul>
    </p>
)";

/**
 * @brief Validates that all dimensions are positive.
 */
[[nodiscard]] bool allPositive(const std::vector<double>& dims)
{
  return std::ranges::all_of(dims, [](double d) { return d > 0.0; });
}

}  // namespace

namespace generate_shape_urdf
{

GenerateShapeUrdf::GenerateShapeUrdf(const std::string& name, const BT::NodeConfiguration& config)
  : BT::SyncActionNode(name, config)
{
}

BT::PortsList GenerateShapeUrdf::providedPorts()
{
  return {
    BT::InputPort<std::string>(kPortIDShapeType, "{shape_type}",
                               "Shape type to generate: 'box', 'sphere', or 'cylinder'."),
    BT::InputPort<std::vector<double>>(kPortIDDimensions, "{dimensions}",
                                       "Dimensions for the shape in meters. "
                                       "Box requires 3 values [width;height;depth], "
                                       "sphere requires 1 value [radius], "
                                       "cylinder requires 2 values [radius;height]."),
    BT::OutputPort<std::string>(kPortIDUrdfString, "{urdf_string}", "The generated URDF XML string."),
    BT::InputPort<std::string>(kPortIDUrdfFilePath, kDefaultUrdfFilePath, "File path to write the URDF to."),
    BT::InputPort<std::string>(kPortIDObjectName, "generated_object",
                               "Name used for the robot and link elements in the URDF."),
    BT::InputPort<double>(kPortIDOriginOffsetZ, 0.0, "Z offset for the collision origin."),
    BT::InputPort<std::vector<double>>(kPortIDOriginRPY, "",
                                       "Roll, pitch, yaw (radians) for the collision origin orientation. "
                                       "If empty, computed from object_pose and grasp_pose if provided."),
    BT::InputPort<geometry_msgs::msg::PoseStamped>(
        kPortIDObjectPose, "", "Object pose in world frame. Used with grasp_pose to auto-compute orientation."),
    BT::InputPort<geometry_msgs::msg::PoseStamped>(kPortIDGraspPose, "",
                                                   "Grasp_link pose in world frame at pick time. Used with object_pose "
                                                   "to auto-compute orientation."),
    BT::OutputPort<std::string>("output_file_path", "{generated_urdf_path}", "The file path the URDF was written to."),
  };
}

BT::KeyValueVector GenerateShapeUrdf::metadata()
{
  return { { "subcategory", "Planning Scene" }, { "description", kDescription } };
}

BT::NodeStatus GenerateShapeUrdf::tick()
{
  const auto shape_type_input = getInput<std::string>(kPortIDShapeType);
  const auto dimensions_input = getInput<std::vector<double>>(kPortIDDimensions);

  if (!shape_type_input.has_value())
  {
    spdlog::error("GenerateShapeUrdf: failed to get 'shape_type' input port.");
    return BT::NodeStatus::FAILURE;
  }
  if (!dimensions_input.has_value())
  {
    spdlog::error("GenerateShapeUrdf: failed to get 'dimensions' input port.");
    return BT::NodeStatus::FAILURE;
  }

  const auto& shape_type = shape_type_input.value();
  const auto& dimensions = dimensions_input.value();

  // Validate dimensions are positive
  if (!allPositive(dimensions))
  {
    spdlog::error("GenerateShapeUrdf: all dimensions must be positive.");
    return BT::NodeStatus::FAILURE;
  }

  // Generate geometry XML based on shape type
  std::string geometry_xml;

  if (shape_type == "box")
  {
    if (dimensions.size() != 3)
    {
      spdlog::error("GenerateShapeUrdf: 'box' requires exactly 3 dimensions [width, height, depth], got {}.",
                    dimensions.size());
      return BT::NodeStatus::FAILURE;
    }
    geometry_xml = fmt::format(R"(<box size="{} {} {}"/>)", dimensions[0], dimensions[1], dimensions[2]);
  }
  else if (shape_type == "sphere")
  {
    if (dimensions.size() != 1)
    {
      spdlog::error("GenerateShapeUrdf: 'sphere' requires exactly 1 dimension [radius], got {}.", dimensions.size());
      return BT::NodeStatus::FAILURE;
    }
    geometry_xml = fmt::format(R"(<sphere radius="{}"/>)", dimensions[0]);
  }
  else if (shape_type == "cylinder")
  {
    if (dimensions.size() != 2)
    {
      spdlog::error("GenerateShapeUrdf: 'cylinder' requires exactly 2 dimensions [radius, height], got {}.",
                    dimensions.size());
      return BT::NodeStatus::FAILURE;
    }
    geometry_xml = fmt::format(R"(<cylinder radius="{}" length="{}"/>)", dimensions[0], dimensions[1]);
  }
  else
  {
    spdlog::error("GenerateShapeUrdf: unsupported shape type '{}'. Must be 'box', 'sphere', or 'cylinder'.", shape_type);
    return BT::NodeStatus::FAILURE;
  }

  // Get optional parameters
  const std::string object_name = getInput<std::string>(kPortIDObjectName).value_or("generated_object");
  const double origin_offset_z = getInput<double>(kPortIDOriginOffsetZ).value_or(0.0);
  const std::string file_path = getInput<std::string>(kPortIDUrdfFilePath).value_or(kDefaultUrdfFilePath);

  // Get orientation: either explicit RPY, or computed from object_pose and grasp_pose.
  double roll = 0.0, pitch = 0.0, yaw = 0.0;
  const auto rpy_input = getInput<std::vector<double>>(kPortIDOriginRPY);
  if (rpy_input.has_value() && rpy_input->size() == 3)
  {
    roll = rpy_input->at(0);
    pitch = rpy_input->at(1);
    yaw = rpy_input->at(2);
  }
  else
  {
    // Try to compute from object_pose and grasp_pose
    const auto obj_pose = getInput<geometry_msgs::msg::PoseStamped>(kPortIDObjectPose);
    const auto grasp_pose = getInput<geometry_msgs::msg::PoseStamped>(kPortIDGraspPose);
    if (obj_pose.has_value() && grasp_pose.has_value())
    {
      const auto& oq = obj_pose->pose.orientation;
      const auto& gq = grasp_pose->pose.orientation;
      const Eigen::Quaterniond q_object(oq.w, oq.x, oq.y, oq.z);
      const Eigen::Quaterniond q_grasp(gq.w, gq.x, gq.y, gq.z);
      // R_grasp_object = R_world_grasp^-1 * R_world_object
      const Eigen::Quaterniond q_rel = q_grasp.inverse() * q_object;
      const Eigen::Vector3d euler = q_rel.toRotationMatrix().eulerAngles(0, 1, 2);
      roll = euler.x();
      pitch = euler.y();
      yaw = euler.z();
      spdlog::info("GenerateShapeUrdf: auto-computed RPY from poses: [{:.4f}, {:.4f}, {:.4f}]", roll, pitch, yaw);
    }
  }

  // Build the full URDF string with collision origin offset and orientation
  const std::string urdf_string =
      fmt::format("<?xml version=\"1.0\"?>\n"
                  "<robot name=\"{name}\">\n"
                  "  <link name=\"{name}\">\n"
                  "    <visual>\n"
                  "      <origin xyz=\"0 0 {offset_z}\" rpy=\"{roll} {pitch} {yaw}\"/>\n"
                  "      <geometry>\n"
                  "        {geometry}\n"
                  "      </geometry>\n"
                  "    </visual>\n"
                  "    <collision>\n"
                  "      <origin xyz=\"0 0 {offset_z}\" rpy=\"{roll} {pitch} {yaw}\"/>\n"
                  "      <geometry>\n"
                  "        {geometry}\n"
                  "      </geometry>\n"
                  "    </collision>\n"
                  "  </link>\n"
                  "</robot>\n",
                  fmt::arg("name", object_name), fmt::arg("offset_z", origin_offset_z), fmt::arg("roll", roll),
                  fmt::arg("pitch", pitch), fmt::arg("yaw", yaw), fmt::arg("geometry", geometry_xml));

  // Write to file
  std::ofstream urdf_file(file_path);
  if (!urdf_file.is_open())
  {
    spdlog::error("GenerateShapeUrdf: failed to write URDF to '{}'.", file_path);
    return BT::NodeStatus::FAILURE;
  }
  urdf_file << urdf_string;
  urdf_file.close();

  spdlog::info("GenerateShapeUrdf: wrote URDF to '{}' (shape={}, object={}, offset_z={:.4f}).", file_path, shape_type,
               object_name, origin_offset_z);

  setOutput(kPortIDUrdfString, urdf_string);
  setOutput("output_file_path", file_path);
  return BT::NodeStatus::SUCCESS;
}

}  // namespace generate_shape_urdf
