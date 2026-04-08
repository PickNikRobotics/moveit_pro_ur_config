// Copyright 2026 PickNik Inc.
// All rights reserved.
//
// Unauthorized copying of this code base via any medium is strictly prohibited.
// Proprietary and confidential.

#include <get_graspable_grocery_objects_from_masks3d/get_graspable_grocery_objects_from_masks3d.hpp>
#include <get_graspable_grocery_objects_from_masks3d/shape_fitting.hpp>

#include <pcl/common/centroid.h>
#include <pcl_conversions/pcl_conversions.h>
#include <spdlog/spdlog.h>

#include <tf2_ros/buffer.h>
#include <moveit_pro_behavior_interface/get_required_ports.hpp>
#include <moveit_studio_vision_msgs/msg/graspable_object.hpp>
#include <moveit_studio_vision_msgs/msg/mask3_d.hpp>
#include <moveit_studio_vision_msgs/msg/object_subframe.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <tf2_eigen/tf2_eigen.hpp>

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
constexpr auto kPortIDBinFloorFrame = "bin_floor_frame";
constexpr auto kPortIDShapeType = "shape_type";
constexpr auto kPortIDFusedCloud = "fused_point_cloud";
constexpr auto kPortIDMaxObjects = "max_objects";

constexpr auto kMarkerTopic = "/visual_markers";
constexpr auto kMarkerNamespace = "grocery_objects";

const auto kTFLookupTimeoutSeconds = std::chrono::duration<double>{ 0.1 };

/** @brief Create a visualization marker for a shape fit result. */
visualization_msgs::msg::Marker createShapeMarker(const get_graspable_grocery_objects_from_masks3d::ShapeFitResult& fit,
                                                  const Eigen::Isometry3d& tform_base_to_cloud,
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
    m.scale.z = fit.primitive.dimensions[0];        // height
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
  segment_cloud_pub_ = shared_resources->node->create_publisher<sensor_msgs::msg::PointCloud2>(
      "/grocery_segment_cloud", rclcpp::QoS(1).transient_local());
}

BT::PortsList GetGraspableGroceryObjectsFromMasks3D::providedPorts()
{
  return {
    BT::InputPort<sensor_msgs::msg::PointCloud2>(kPortIDPointCloud, "{point_cloud}", "Input point cloud."),
    BT::InputPort<std::vector<moveit_studio_vision_msgs::msg::Mask3D>>(kPortIDMasks3D, "{masks3d}",
                                                                       "3D masks selecting subsets of points."),
    BT::InputPort<std::string>(kPortIDBaseFrame, "world",
                               "Calculate poses of graspable objects relative to this frame."),
    BT::InputPort<double>(kPortIDPlaneInlierThreshold, 0.01, "Distance threshold in meters for RANSAC shape fitting."),
    BT::InputPort<double>(kPortIDMinimumFaceArea, 0.000625,
                          "Minimum area in meters^2 for a face to be considered graspable."),
    BT::InputPort<double>(kPortIDFaceSeparationThreshold, 0.01,
                          "Distance in meters between coplanar clusters to split them into separate faces."),
    BT::OutputPort<std::vector<moveit_studio_vision_msgs::msg::GraspableObject>>(
        kPortIDGraspableObjects, "{graspable_objects}", "Vector of graspable objects with best-fit primitive shapes."),
    BT::InputPort<std::string>(kPortIDBinFloorFrame, "",
                               "TF frame at the bin floor. The Z coordinate of this frame in the base_frame "
                               "is used to clamp cylinder height. Leave empty to disable."),
    BT::InputPort<std::string>(kPortIDShapeType, "",
                               "Force a specific shape type: 'cylinder', 'box', or 'sphere'. "
                               "Leave empty to auto-detect best fit."),
    BT::InputPort<sensor_msgs::msg::PointCloud2>(
        kPortIDFusedCloud, "",
        "Optional fused multi-view point cloud. When provided, each mask's 3D bounding "
        "box is used to crop this cloud for shape fitting instead of the single-view cloud."),
    BT::OutputPort<sensor_msgs::msg::PointCloud2>(
        "segment_cloud", "{segment_cloud}", "Point cloud of the first mask segment, for debugging visualization."),
    BT::InputPort<int>(kPortIDMaxObjects, 1,
                       "Maximum number of masks to process. Set to 1 to only use the highest-confidence detection."),
  };
}

BT::KeyValueVector GetGraspableGroceryObjectsFromMasks3D::metadata()
{
  return { { "subcategory", "Perception - 3D" }, { "description", kDescriptionGetGraspableGroceryObjectsFromMasks3D } };
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

  // Look up the bin floor Z coordinate if a bin_floor_frame is provided.
  double bin_floor_z = std::numeric_limits<double>::quiet_NaN();
  const auto bin_floor_frame = getInput<std::string>(kPortIDBinFloorFrame);
  if (bin_floor_frame.has_value() && !bin_floor_frame->empty())
  {
    try
    {
      const auto tf = shared_resources_->transform_buffer_ptr->lookupTransform(
          point_cloud_msg.header.frame_id, bin_floor_frame.value(), tf2_ros::fromMsg(point_cloud_msg.header.stamp),
          tf2::durationFromSec(kTFLookupTimeoutSeconds.count()));
      bin_floor_z = tf.transform.translation.z;
      spdlog::info("Bin floor frame '{}' has Z={:.4f} in cloud frame '{}'", bin_floor_frame.value(), bin_floor_z,
                   point_cloud_msg.header.frame_id);
    }
    catch (const std::exception& e)
    {
      spdlog::warn("Could not look up bin floor frame '{}': {}. Cylinder height will not be clamped.",
                   bin_floor_frame.value(), e.what());
    }
  }

  // Load optional fused multi-view point cloud for richer shape fitting.
  auto fused_cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();
  bool use_fused_cloud = false;
  const auto fused_cloud_input = getInput<sensor_msgs::msg::PointCloud2>(kPortIDFusedCloud);
  if (fused_cloud_input.has_value() && !fused_cloud_input->data.empty())
  {
    pcl::fromROSMsg(fused_cloud_input.value(), *fused_cloud);
    use_fused_cloud = true;
    spdlog::info("Fused point cloud loaded: {} points. Will use for shape fitting.", fused_cloud->points.size());
  }

  // Read optional forced shape type.
  std::string forced_shape;
  const auto shape_type_input = getInput<std::string>(kPortIDShapeType);
  if (shape_type_input.has_value() && !shape_type_input->empty())
  {
    forced_shape = shape_type_input.value();
    spdlog::info("Forced shape type: '{}'", forced_shape);
  }

  // Clear previous markers.
  visualization_msgs::msg::MarkerArray marker_array;
  visualization_msgs::msg::Marker delete_marker;
  delete_marker.action = visualization_msgs::msg::Marker::DELETEALL;
  delete_marker.ns = kMarkerNamespace;
  marker_array.markers.push_back(delete_marker);

  // Limit number of masks to process.
  const int max_objects = getInput<int>(kPortIDMaxObjects).value_or(static_cast<int>(masks.size()));
  const std::size_t num_masks = std::min(masks.size(), static_cast<std::size_t>(std::max(max_objects, 1)));
  spdlog::info("Processing {} of {} masks (max_objects={})", num_masks, masks.size(), max_objects);

  std::vector<moveit_studio_vision_msgs::msg::GraspableObject> graspable_objects;
  graspable_objects.reserve(num_masks);
  int marker_id = 0;

  for (std::size_t i = 0; i < num_masks; ++i)
  {
    const auto& mask = masks[i];
    const auto mask_indices = std::make_shared<pcl::PointIndices>();
    mask_indices->header = point_cloud->header;
    mask_indices->indices = mask.point_indices;

    spdlog::info("Fitting shapes to mask {} ({} points)...", i, mask.point_indices.size());

    // Determine which cloud and indices to use for shape fitting.
    std::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB>> fit_cloud;
    auto fit_indices = std::make_shared<pcl::PointIndices>();

    if (use_fused_cloud)
    {
      // Get the AABB of the mask's single-view points in world frame.
      constexpr double kBboxPadding = 0.02;  // 2cm padding
      double min_x = std::numeric_limits<double>::max(), max_x = std::numeric_limits<double>::lowest();
      double min_y = std::numeric_limits<double>::max(), max_y = std::numeric_limits<double>::lowest();
      double min_z = std::numeric_limits<double>::max(), max_z = std::numeric_limits<double>::lowest();

      for (const int idx : mask.point_indices)
      {
        const auto& pt = point_cloud->points[idx];
        // Transform to base frame
        const Eigen::Vector3d p_base = tform_base_to_cloud * Eigen::Vector3d(pt.x, pt.y, pt.z);
        min_x = std::min(min_x, p_base.x());
        max_x = std::max(max_x, p_base.x());
        min_y = std::min(min_y, p_base.y());
        max_y = std::max(max_y, p_base.y());
        min_z = std::min(min_z, p_base.z());
        max_z = std::max(max_z, p_base.z());
      }
      min_x -= kBboxPadding;
      min_y -= kBboxPadding;
      min_z -= kBboxPadding;
      max_x += kBboxPadding;
      max_y += kBboxPadding;
      max_z += kBboxPadding;

      // Crop the fused cloud to this bounding box.
      // The fused cloud is assumed to be in the base (world) frame already.
      for (int fi = 0; fi < static_cast<int>(fused_cloud->points.size()); ++fi)
      {
        const auto& fp = fused_cloud->points[fi];
        if (fp.x >= min_x && fp.x <= max_x && fp.y >= min_y && fp.y <= max_y && fp.z >= min_z && fp.z <= max_z)
        {
          fit_indices->indices.push_back(fi);
        }
      }

      fit_cloud = fused_cloud;
      spdlog::info("Cropped fused cloud to mask {} bounding box: {} points (from {} total)", i,
                   fit_indices->indices.size(), fused_cloud->points.size());
    }
    else
    {
      fit_cloud = point_cloud;
      fit_indices->indices = mask.point_indices;
    }
    fit_indices->header = fit_cloud->header;

    // Output the first segment's point cloud for debugging visualization.
    if (i == 0)
    {
      auto segment = std::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();
      segment->header = fit_cloud->header;
      for (const int idx : fit_indices->indices)
      {
        segment->points.push_back(fit_cloud->points[idx]);
      }
      segment->width = segment->points.size();
      segment->height = 1;
      segment->is_dense = false;
      sensor_msgs::msg::PointCloud2 segment_msg;
      pcl::toROSMsg(*segment, segment_msg);
      segment_msg.header = point_cloud_msg.header;
      setOutput("segment_cloud", segment_msg);
      segment_cloud_pub_->publish(segment_msg);
      spdlog::info("Published segment point cloud for mask 0 ({} points) on /grocery_segment_cloud (transient_local)",
                   segment->points.size());
    }

    const auto fit_result = fitBestShape(std::const_pointer_cast<const pcl::PointCloud<pcl::PointXYZRGB>>(fit_cloud),
                                         std::const_pointer_cast<const pcl::PointIndices>(fit_indices),
                                         plane_inlier_threshold, bin_floor_z, forced_shape);

    if (!fit_result.has_value())
    {
      spdlog::warn("Unable to fit shape to mask {}: {}", i, fit_result.error());
      continue;
    }

    const auto& fit = fit_result.value();

    // Validate: reject if the fit center is too far from the point cloud centroid.
    // This catches bad RANSAC fits that produce shapes far from the actual points.
    {
      Eigen::Vector4f centroid;
      pcl::compute3DCentroid(*fit_cloud, fit_indices->indices, centroid);
      const Eigen::Vector3d centroid3d(centroid.x(), centroid.y(), centroid.z());
      const double center_distance = (fit.origin.translation() - centroid3d).norm();
      constexpr double kMaxCenterOffset = 0.15;  // 15cm max offset from centroid
      if (center_distance > kMaxCenterOffset)
      {
        spdlog::warn("Rejecting fit for mask {}: shape center is {:.4f}m from point centroid (max {:.4f}m)", i,
                     center_distance, kMaxCenterOffset);
        continue;
      }
    }

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
