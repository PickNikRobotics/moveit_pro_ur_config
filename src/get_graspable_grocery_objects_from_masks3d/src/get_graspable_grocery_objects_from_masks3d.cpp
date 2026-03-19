// Copyright 2026 PickNik Inc.
// All rights reserved.
//
// Unauthorized copying of this code base via any medium is strictly prohibited.
// Proprietary and confidential.

#include <get_graspable_grocery_objects_from_masks3d/get_graspable_grocery_objects_from_masks3d.hpp>
#include <get_graspable_grocery_objects_from_masks3d/shape_fitting.hpp>

#include <pcl_conversions/pcl_conversions.h>
#include <spdlog/spdlog.h>

#include <moveit_pro_behavior_interface/get_required_ports.hpp>
#include <moveit_studio_vision_msgs/msg/graspable_object.hpp>
#include <moveit_studio_vision_msgs/msg/mask3_d.hpp>
#include <moveit_studio_vision_msgs/msg/object_subframe.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_ros/buffer.h>

namespace
{
inline constexpr auto kDescriptionGetGraspableGroceryObjectsFromMasks3D = R"(
                <p>
                    Creates graspable objects from 3D masks by fitting the best primitive shape
                    (box, cylinder, or sphere) to each point cloud segment. The fitted shape type
                    is stored in the object_type field of GraspInfo.
                </p>
            )";

constexpr auto kPortIDPointCloud = "point_cloud";
constexpr auto kPortIDMasks3D = "masks3d";
constexpr auto kPortIDBaseFrame = "base_frame";
constexpr auto kPortIDPlaneInlierThreshold = "plane_inlier_threshold";
constexpr auto kPortIDMinimumFaceArea = "minimum_face_area";
constexpr auto kPortIDFaceSeparationThreshold = "face_separation_threshold";
constexpr auto kPortIDGraspableObjects = "graspable_objects";

constexpr auto kMarkerTopic = "/visual_markers";
constexpr auto kMarkerNamespace = "grocery_objects";

const auto kTFLookupTimeoutSeconds = std::chrono::duration<double>{ 0.1 };

/** @brief Create a visualization marker for a shape fit result. */
visualization_msgs::msg::Marker createShapeMarker(
    const get_graspable_grocery_objects_from_masks3d::ShapeFitResult& fit, const Eigen::Isometry3d& tform_base_to_cloud,
    const std::string& frame_id, int id)
{
  visualization_msgs::msg::Marker m;
  m.action = visualization_msgs::msg::Marker::ADD;
  m.header.frame_id = frame_id;
  m.id = id;
  m.ns = kMarkerNamespace;
  m.color.r = 0.2f;
  m.color.g = 1.0f;
  m.color.b = 0.4f;
  m.color.a = 0.5f;
  m.lifetime.sec = 15;
  m.pose = tf2::toMsg(tform_base_to_cloud * fit.origin);

  if (fit.primitive.type == shape_msgs::msg::SolidPrimitive::BOX)
  {
    m.type = visualization_msgs::msg::Marker::CUBE;
    m.scale.x = fit.primitive.dimensions[0];
    m.scale.y = fit.primitive.dimensions[1];
    m.scale.z = fit.primitive.dimensions[2];
  }
  else if (fit.primitive.type == shape_msgs::msg::SolidPrimitive::CYLINDER)
  {
    m.type = visualization_msgs::msg::Marker::CYLINDER;
    m.scale.x = fit.primitive.dimensions[1] * 2.0;  // diameter
    m.scale.y = fit.primitive.dimensions[1] * 2.0;  // diameter
    m.scale.z = fit.primitive.dimensions[0];         // height
  }
  else if (fit.primitive.type == shape_msgs::msg::SolidPrimitive::SPHERE)
  {
    m.type = visualization_msgs::msg::Marker::SPHERE;
    const double diameter = fit.primitive.dimensions[0] * 2.0;
    m.scale.x = diameter;
    m.scale.y = diameter;
    m.scale.z = diameter;
  }

  return m;
}

/** @brief Create a text marker labeling the shape type. */
visualization_msgs::msg::Marker createLabelMarker(const get_graspable_grocery_objects_from_masks3d::ShapeFitResult& fit,
                                                   const Eigen::Isometry3d& tform_base_to_cloud,
                                                   const std::string& frame_id, int id)
{
  visualization_msgs::msg::Marker m;
  m.action = visualization_msgs::msg::Marker::ADD;
  m.header.frame_id = frame_id;
  m.id = id;
  m.ns = kMarkerNamespace;
  m.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
  m.text = fit.shape_type;
  m.color.r = 1.0f;
  m.color.g = 1.0f;
  m.color.b = 1.0f;
  m.color.a = 1.0f;
  m.scale.z = 0.03;
  m.lifetime.sec = 15;

  auto pose = tform_base_to_cloud * fit.origin;
  pose.translation().z() += 0.05;
  m.pose = tf2::toMsg(pose);

  return m;
}
}  // namespace

namespace get_graspable_grocery_objects_from_masks3d
{

GetGraspableGroceryObjectsFromMasks3D::GetGraspableGroceryObjectsFromMasks3D(
    const std::string& name, const BT::NodeConfiguration& config,
    const std::shared_ptr<moveit_pro::behaviors::BehaviorContext>& shared_resources)
  : AsyncBehaviorBase(name, config, shared_resources)
{
  marker_pub_ = shared_resources->node->create_publisher<visualization_msgs::msg::MarkerArray>(
      kMarkerTopic, rclcpp::QoS(1).transient_local());
}

BT::PortsList GetGraspableGroceryObjectsFromMasks3D::providedPorts()
{
  return {
    BT::InputPort<sensor_msgs::msg::PointCloud2>(kPortIDPointCloud, "{point_cloud}", "Input point cloud."),
    BT::InputPort<std::vector<moveit_studio_vision_msgs::msg::Mask3D>>(kPortIDMasks3D, "{masks3d}",
                                                                       "3D masks selecting subsets of points."),
    BT::InputPort<std::string>(kPortIDBaseFrame, "world",
                               "Calculate poses of graspable objects relative to this frame."),
    BT::InputPort<double>(kPortIDPlaneInlierThreshold, 0.01,
                          "Distance threshold in meters for RANSAC shape fitting."),
    BT::InputPort<double>(kPortIDMinimumFaceArea, 0.000625,
                          "Minimum area in meters^2 for a face to be considered graspable."),
    BT::InputPort<double>(kPortIDFaceSeparationThreshold, 0.01,
                          "Distance in meters between coplanar clusters to split them into separate faces."),
    BT::OutputPort<std::vector<moveit_studio_vision_msgs::msg::GraspableObject>>(
        kPortIDGraspableObjects, "{graspable_objects}",
        "Vector of graspable objects with best-fit primitive shapes."),
  };
}

BT::KeyValueVector GetGraspableGroceryObjectsFromMasks3D::metadata()
{
  return { { "subcategory", "Perception - 3D" },
           { "description", kDescriptionGetGraspableGroceryObjectsFromMasks3D } };
}

tl::expected<bool, std::string> GetGraspableGroceryObjectsFromMasks3D::doWork()
{
  const auto ports = moveit_pro::behaviors::getRequiredInputs(
      getInput<sensor_msgs::msg::PointCloud2>(kPortIDPointCloud),
      getInput<std::vector<moveit_studio_vision_msgs::msg::Mask3D>>(kPortIDMasks3D),
      getInput<std::string>(kPortIDBaseFrame), getInput<double>(kPortIDPlaneInlierThreshold),
      getInput<double>(kPortIDMinimumFaceArea), getInput<double>(kPortIDFaceSeparationThreshold));

  if (!ports.has_value())
  {
    return tl::make_unexpected("Failed to get required value from input data port: " + ports.error());
  }

  const auto& [point_cloud_msg, masks, base_frame, plane_inlier_threshold, minimum_face_area,
               face_separation_threshold] = ports.value();

  if (masks.empty())
  {
    return tl::make_unexpected("There are no input masks to create graspable objects.");
  }

  // Convert ROS point cloud to PCL.
  const auto point_cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();
  pcl::fromROSMsg(point_cloud_msg, *point_cloud);

  // Look up transform from base frame to cloud frame.
  Eigen::Isometry3d tform_base_to_cloud;
  try
  {
    tform_base_to_cloud = tf2::transformToEigen(shared_resources_->transform_buffer_ptr->lookupTransform(
        base_frame, point_cloud_msg.header.frame_id, tf2_ros::fromMsg(point_cloud_msg.header.stamp),
        tf2::durationFromSec(kTFLookupTimeoutSeconds.count())));
  }
  catch (const std::exception& e)
  {
    return tl::make_unexpected(
        std::string("Failed to look up transform from base frame to point cloud origin frame: ") + e.what());
  }

  // Clear previous markers.
  visualization_msgs::msg::MarkerArray marker_array;
  visualization_msgs::msg::Marker delete_marker;
  delete_marker.action = visualization_msgs::msg::Marker::DELETEALL;
  delete_marker.ns = kMarkerNamespace;
  marker_array.markers.push_back(delete_marker);

  std::vector<moveit_studio_vision_msgs::msg::GraspableObject> graspable_objects;
  graspable_objects.reserve(masks.size());
  int marker_id = 0;

  for (std::size_t i = 0; i < masks.size(); ++i)
  {
    const auto& mask = masks[i];
    const auto mask_indices = std::make_shared<pcl::PointIndices>();
    mask_indices->header = point_cloud->header;
    mask_indices->indices = mask.point_indices;

    spdlog::info("Fitting shapes to mask {} ({} points)...", i, mask.point_indices.size());

    const auto fit_result = fitBestShape(
        std::const_pointer_cast<const pcl::PointCloud<pcl::PointXYZRGB>>(point_cloud),
        std::const_pointer_cast<const pcl::PointIndices>(mask_indices), plane_inlier_threshold);

    if (!fit_result.has_value())
    {
      spdlog::warn("Unable to fit shape to mask {}: {}", i, fit_result.error());
      continue;
    }

    const auto& fit = fit_result.value();

    // Build the GraspableObject message.
    moveit_studio_vision_msgs::msg::GraspableObject graspable_object;
    const std::string label = std::string("object_").append(std::to_string(i));
    graspable_object.object.id = label;
    graspable_object.object.header.frame_id = base_frame;
    graspable_object.object.header.stamp = builtin_interfaces::msg::Time();
    graspable_object.object.pose = tf2::toMsg(tform_base_to_cloud * fit.origin);

    // Set the bounding volume to the fitted primitive.
    graspable_object.grasp_info.bounding_volume = fit.primitive;

    // Also store as a collision primitive.
    graspable_object.object.primitives.push_back(fit.primitive);
    graspable_object.object.primitive_poses.push_back(tf2::toMsg(Eigen::Isometry3d::Identity()));

    // Store the shape type for downstream behaviors to use.
    graspable_object.grasp_info.object_type = fit.shape_type;

    // Add a subframe named after the shape type.
    moveit_studio_vision_msgs::msg::ObjectSubframe subframe;
    subframe.id = fit.shape_type;
    subframe.pose = tf2::toMsg(Eigen::Isometry3d::Identity());
    graspable_object.grasp_info.subframes.push_back(subframe);

    graspable_objects.push_back(graspable_object);

    // Create visualization markers.
    marker_array.markers.push_back(createShapeMarker(fit, tform_base_to_cloud, base_frame, marker_id++));
    marker_array.markers.push_back(createLabelMarker(fit, tform_base_to_cloud, base_frame, marker_id++));

    spdlog::info("Object {}: {} (volume={:.6f} m^3)", i, fit.shape_type, fit.volume);
  }

  marker_pub_->publish(marker_array);

  if (graspable_objects.empty())
  {
    return tl::make_unexpected("Could not create any graspable objects from the masked point cloud fragments.");
  }

  setOutput(kPortIDGraspableObjects, graspable_objects);

  spdlog::info("GetGraspableGroceryObjectsFromMasks3D: created {} graspable objects.", graspable_objects.size());
  return true;
}

}  // namespace get_graspable_grocery_objects_from_masks3d
