#include <picknik_009_ur5e_behaviors/create_solid_primitive_from_shape.hpp>

#include <algorithm>
#include <cctype>
#include <string>
#include <vector>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <spdlog/spdlog.h>

namespace
{
constexpr auto kPortIDShapeType = "shape_type";
constexpr auto kPortIDDimensions = "dimensions";
constexpr auto kPortIDPoseStamped = "pose_stamped";
constexpr auto kPortIDSolidPrimitive = "solid_primitive";

inline constexpr auto kDescription = R"(
    <p>
        Creates a <b>shape_msgs::msg::SolidPrimitive</b> from a primitive type,
        a dimension vector, and a pose.
    </p>
    <p>
        Supported shape types:
        <ul>
        <li><b>cube</b>: [x, y, z]</li>
            <li><b>sphere</b>: [radius]</li>
            <li><b>cylinder</b>: [radius, height]</li>
        </ul>
    </p>
)";

[[nodiscard]] bool allPositive(const std::vector<double>& dimensions)
{
  return std::ranges::all_of(dimensions, [](double dimension) { return dimension > 0.0; });
}

[[nodiscard]] std::string normalizeShapeType(std::string shape_type)
{
  std::transform(shape_type.begin(), shape_type.end(), shape_type.begin(), [](unsigned char character) {
    return static_cast<char>(std::tolower(character));
  });
  return shape_type;
}

}  // namespace

namespace picknik_009_ur5e_behaviors
{

CreateSolidPrimitiveFromShape::CreateSolidPrimitiveFromShape(const std::string& name,
                                                             const BT::NodeConfiguration& config)
  : BT::SyncActionNode(name, config)
{
}

BT::PortsList CreateSolidPrimitiveFromShape::providedPorts()
{
  return {
    BT::InputPort<std::string>(kPortIDShapeType, "{shape_type}",
                               "Shape type to create: 'cube', 'sphere', or 'cylinder'."),
    BT::InputPort<std::vector<double>>(kPortIDDimensions, "{dimensions}",
                                       "Dimensions in meters. Expects a 3-element vector. "
                                       "Cube uses [x;y;z], sphere uses the first element as [radius], "
                                       "and cylinder uses the first two elements as [radius;height]."),
    BT::InputPort<geometry_msgs::msg::PoseStamped>(kPortIDPoseStamped, "{pose_stamped}",
                                                   "Pose associated with the primitive."),
    BT::OutputPort<shape_msgs::msg::SolidPrimitive>(kPortIDSolidPrimitive, "{solid_primitive}",
                                                    "Constructed SolidPrimitive message."),
  };
}

BT::KeyValueVector CreateSolidPrimitiveFromShape::metadata()
{
  return { { "subcategory", "Planning Scene" }, { "description", kDescription } };
}

BT::NodeStatus CreateSolidPrimitiveFromShape::tick()
{
  const auto shape_type_input = getInput<std::string>(kPortIDShapeType);
  const auto dimensions_input = getInput<std::vector<double>>(kPortIDDimensions);
  const auto pose_stamped_input = getInput<geometry_msgs::msg::PoseStamped>(kPortIDPoseStamped);

  if (!shape_type_input.has_value())
  {
    spdlog::error("CreateSolidPrimitiveFromShape: failed to get 'shape_type' input port.");
    return BT::NodeStatus::FAILURE;
  }
  if (!dimensions_input.has_value())
  {
    spdlog::error("CreateSolidPrimitiveFromShape: failed to get 'dimensions' input port.");
    return BT::NodeStatus::FAILURE;
  }
  if (!pose_stamped_input.has_value())
  {
    spdlog::error("CreateSolidPrimitiveFromShape: failed to get 'pose_stamped' input port.");
    return BT::NodeStatus::FAILURE;
  }

  const std::string shape_type = normalizeShapeType(shape_type_input.value());
  const auto& dimensions = dimensions_input.value();

  if (dimensions.size() != 3)
  {
    spdlog::error("CreateSolidPrimitiveFromShape: 'dimensions' must contain exactly 3 elements, got {}.",
                  dimensions.size());
    return BT::NodeStatus::FAILURE;
  }

  if (!allPositive(dimensions))
  {
    spdlog::error("CreateSolidPrimitiveFromShape: all dimensions must be positive.");
    return BT::NodeStatus::FAILURE;
  }

  shape_msgs::msg::SolidPrimitive primitive;

  if (shape_type == "cube")
  {
    primitive.type = shape_msgs::msg::SolidPrimitive::BOX;
    primitive.dimensions.assign(dimensions.begin(), dimensions.end());
  }
  else if (shape_type == "sphere")
  {
    primitive.type = shape_msgs::msg::SolidPrimitive::SPHERE;
    primitive.dimensions = { dimensions[0] };
  }
  else if (shape_type == "cylinder")
  {
    primitive.type = shape_msgs::msg::SolidPrimitive::CYLINDER;
    primitive.dimensions.resize(2);
    primitive.dimensions[shape_msgs::msg::SolidPrimitive::CYLINDER_HEIGHT] = dimensions[1];
    primitive.dimensions[shape_msgs::msg::SolidPrimitive::CYLINDER_RADIUS] = dimensions[0];
  }
  else
  {
    spdlog::error(
        "CreateSolidPrimitiveFromShape: unsupported shape type '{}'. Must be 'cube', 'sphere', or 'cylinder'.",
        shape_type_input.value());
    return BT::NodeStatus::FAILURE;
  }

  setOutput(kPortIDSolidPrimitive, primitive);
  return BT::NodeStatus::SUCCESS;
}

}  // namespace picknik_009_ur5e_behaviors