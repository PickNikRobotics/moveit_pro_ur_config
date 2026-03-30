// Copyright 2026 PickNik Inc.
// All rights reserved.
//
// Unauthorized copying of this code base via any medium is strictly prohibited.
// Proprietary and confidential.

#include <get_graspable_grocery_objects_from_masks3d/shape_fitting.hpp>

#include <pcl/common/centroid.h>
#include <pcl/common/common.h>
#include <pcl/common/pca.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/sac_segmentation.h>

#include <cmath>
#include <spdlog/spdlog.h>

namespace get_graspable_grocery_objects_from_masks3d
{
namespace
{
constexpr int kMinPointsForFitting = 10;
constexpr int kRansacMaxIterations = 1000;
constexpr double kPi = 3.14159265358979323846;

// Max dimensions for grocery items
constexpr double kMaxCylinderRadius = 0.20;   // 20cm
constexpr double kMaxCylinderHeight = 0.40;   // 40cm
constexpr double kMaxBoxDimension = 0.40;     // 40cm
constexpr double kMaxSphereRadius = 0.10;     // 10cm

/** @brief Extract the subset of points from a cloud given indices, with statistical outlier removal. */
[[nodiscard]] pcl::PointCloud<pcl::PointXYZRGB>::Ptr
extractPoints(const std::shared_ptr<const pcl::PointCloud<pcl::PointXYZRGB>>& cloud,
              const std::shared_ptr<const pcl::PointIndices>& indices)
{
  auto subset = std::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();
  subset->points.reserve(indices->indices.size());
  for (const int idx : indices->indices)
  {
    subset->points.push_back(cloud->points[idx]);
  }
  subset->width = subset->points.size();
  subset->height = 1;
  subset->is_dense = false;

  // Remove statistical outliers (noisy depth points from the mask projection).
  if (subset->points.size() > 20)
  {
    auto filtered = std::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();
    pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor;
    sor.setInputCloud(subset);
    sor.setMeanK(20);
    sor.setStddevMulThresh(1.5);
    sor.filter(*filtered);
    spdlog::info("Outlier removal: {} -> {} points", subset->points.size(), filtered->points.size());
    return filtered;
  }

  return subset;
}
}  // namespace

tl::expected<ShapeFitResult, std::string>
fitBox(const std::shared_ptr<const pcl::PointCloud<pcl::PointXYZRGB>>& cloud,
       const std::shared_ptr<const pcl::PointIndices>& indices, double /*bin_floor_z*/)
{
  if (static_cast<int>(indices->indices.size()) < kMinPointsForFitting)
  {
    return tl::make_unexpected("Too few points for box fitting.");
  }

  const auto subset = extractPoints(cloud, indices);

  // Find axis-aligned (world frame) extents for bin floor clamping.
  double world_min_z = std::numeric_limits<double>::max();
  double world_max_z = std::numeric_limits<double>::lowest();
  double world_min_x = std::numeric_limits<double>::max();
  double world_max_x = std::numeric_limits<double>::lowest();
  double world_min_y = std::numeric_limits<double>::max();
  double world_max_y = std::numeric_limits<double>::lowest();

  for (const auto& pt : subset->points)
  {
    world_min_x = std::min(world_min_x, static_cast<double>(pt.x));
    world_max_x = std::max(world_max_x, static_cast<double>(pt.x));
    world_min_y = std::min(world_min_y, static_cast<double>(pt.y));
    world_max_y = std::max(world_max_y, static_cast<double>(pt.y));
    world_min_z = std::min(world_min_z, static_cast<double>(pt.z));
    world_max_z = std::max(world_max_z, static_cast<double>(pt.z));
  }

  // Build an axis-aligned bounding box in world frame from the observed points.
  const double size_x = world_max_x - world_min_x;
  const double size_y = world_max_y - world_min_y;
  const double size_z = world_max_z - world_min_z;

  Eigen::Isometry3d origin = Eigen::Isometry3d::Identity();
  origin.translation() = Eigen::Vector3d(
      (world_min_x + world_max_x) / 2.0,
      (world_min_y + world_max_y) / 2.0,
      (world_min_z + world_max_z) / 2.0);

  ShapeFitResult result;
  result.origin = origin;
  result.shape_type = "box";
  result.primitive.type = shape_msgs::msg::SolidPrimitive::BOX;
  const double clamped_x = std::min(size_x, kMaxBoxDimension);
  const double clamped_y = std::min(size_y, kMaxBoxDimension);
  const double clamped_z = std::min(size_z, kMaxBoxDimension);
  result.primitive.dimensions = { clamped_x, clamped_y, clamped_z };
  result.volume = clamped_x * clamped_y * clamped_z;

  return result;
}

tl::expected<ShapeFitResult, std::string>
fitCylinder(const std::shared_ptr<const pcl::PointCloud<pcl::PointXYZRGB>>& cloud,
            const std::shared_ptr<const pcl::PointIndices>& indices, double distance_threshold,
            double /*bin_floor_z*/)
{
  if (static_cast<int>(indices->indices.size()) < kMinPointsForFitting)
  {
    return tl::make_unexpected("Too few points for cylinder fitting.");
  }

  const auto subset = extractPoints(cloud, indices);

  // Estimate normals for cylinder fitting.
  // PCL's cylinder RANSAC requires normals, so we use PCA-based normal estimation.
  auto normals = std::make_shared<pcl::PointCloud<pcl::Normal>>();
  pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>());
  pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> ne;
  ne.setSearchMethod(tree);
  ne.setInputCloud(subset);
  ne.setKSearch(15);
  ne.compute(*normals);

  pcl::SACSegmentationFromNormals<pcl::PointXYZRGB, pcl::Normal> seg;
  seg.setOptimizeCoefficients(true);
  seg.setModelType(pcl::SACMODEL_CYLINDER);
  seg.setMethodType(pcl::SAC_RANSAC);
  seg.setMaxIterations(kRansacMaxIterations);
  seg.setDistanceThreshold(distance_threshold);
  seg.setRadiusLimits(0.005, 0.15);
  seg.setNormalDistanceWeight(0.1);
  seg.setInputCloud(subset);
  seg.setInputNormals(normals);

  pcl::PointIndices inlier_indices;
  pcl::ModelCoefficients coefficients;
  seg.segment(inlier_indices, coefficients);

  if (inlier_indices.indices.empty() || coefficients.values.size() < 7)
  {
    return tl::make_unexpected("Cylinder RANSAC failed to fit a model.");
  }

  // Cylinder coefficients: [point_on_axis.x, point_on_axis.y, point_on_axis.z, axis.x, axis.y, axis.z, radius].
  const Eigen::Vector3d axis_point(coefficients.values[0], coefficients.values[1], coefficients.values[2]);
  const Eigen::Vector3d axis_dir(coefficients.values[3], coefficients.values[4], coefficients.values[5]);
  const double radius = std::abs(coefficients.values[6]);
  const Eigen::Vector3d axis_normalized = axis_dir.normalized();

  spdlog::info("Cylinder RANSAC: axis=[{:.3f}, {:.3f}, {:.3f}], radius={:.4f}",
               axis_normalized.x(), axis_normalized.y(), axis_normalized.z(), radius);

  // Project all points onto the cylinder axis to find the height extent.
  double min_proj = std::numeric_limits<double>::max();
  double max_proj = std::numeric_limits<double>::lowest();

  for (const auto& pt : subset->points)
  {
    const Eigen::Vector3d p(pt.x, pt.y, pt.z);
    const double proj = (p - axis_point).dot(axis_normalized);
    min_proj = std::min(min_proj, proj);
    max_proj = std::max(max_proj, proj);
  }

  const double height = max_proj - min_proj;
  const double center_proj = (min_proj + max_proj) * 0.5;
  const Eigen::Vector3d center = axis_point + center_proj * axis_normalized;

  // Build the transform with the Z axis along the cylinder axis.
  Eigen::Isometry3d origin = Eigen::Isometry3d::Identity();
  origin.translation() = center;

  // Construct a rotation matrix where Z aligns with the cylinder axis.
  const Eigen::Vector3d z_axis = axis_normalized;
  Eigen::Vector3d x_axis = Eigen::Vector3d::UnitX();
  if (std::abs(z_axis.dot(x_axis)) > 0.9)
  {
    x_axis = Eigen::Vector3d::UnitY();
  }
  const Eigen::Vector3d y_axis = z_axis.cross(x_axis).normalized();
  x_axis = y_axis.cross(z_axis).normalized();

  Eigen::Matrix3d rotation;
  rotation.col(0) = x_axis;
  rotation.col(1) = y_axis;
  rotation.col(2) = z_axis;
  origin.linear() = rotation;

  ShapeFitResult result;
  result.origin = origin;
  result.shape_type = "cylinder";
  result.primitive.type = shape_msgs::msg::SolidPrimitive::CYLINDER;
  const double clamped_height = std::min(height, kMaxCylinderHeight);
  const double clamped_radius = std::min(radius, kMaxCylinderRadius);
  result.primitive.dimensions = { clamped_height, clamped_radius };
  result.volume = kPi * clamped_radius * clamped_radius * clamped_height;

  return result;
}

tl::expected<ShapeFitResult, std::string>
fitSphere(const std::shared_ptr<const pcl::PointCloud<pcl::PointXYZRGB>>& cloud,
          const std::shared_ptr<const pcl::PointIndices>& indices, double distance_threshold,
          double /*bin_floor_z*/)
{
  if (static_cast<int>(indices->indices.size()) < kMinPointsForFitting)
  {
    return tl::make_unexpected("Too few points for sphere fitting.");
  }

  const auto subset = extractPoints(cloud, indices);

  pcl::SACSegmentation<pcl::PointXYZRGB> seg;
  seg.setOptimizeCoefficients(true);
  seg.setModelType(pcl::SACMODEL_SPHERE);
  seg.setMethodType(pcl::SAC_RANSAC);
  seg.setMaxIterations(kRansacMaxIterations);
  seg.setDistanceThreshold(distance_threshold);
  seg.setRadiusLimits(0.01, 0.15);
  seg.setInputCloud(subset);

  pcl::PointIndices inlier_indices;
  pcl::ModelCoefficients coefficients;
  seg.segment(inlier_indices, coefficients);

  if (inlier_indices.indices.empty() || coefficients.values.size() < 4)
  {
    return tl::make_unexpected("Sphere RANSAC failed to fit a model.");
  }

  // Sphere coefficients: [center.x, center.y, center.z, radius].
  const Eigen::Vector3d center(coefficients.values[0], coefficients.values[1], coefficients.values[2]);
  const double radius = std::abs(coefficients.values[3]);

  ShapeFitResult result;
  result.origin = Eigen::Isometry3d::Identity();
  result.origin.translation() = center;
  result.shape_type = "sphere";
  result.primitive.type = shape_msgs::msg::SolidPrimitive::SPHERE;
  const double clamped_radius = std::min(radius, kMaxSphereRadius);
  result.primitive.dimensions = { clamped_radius };
  result.volume = (4.0 / 3.0) * kPi * clamped_radius * clamped_radius * clamped_radius;

  return result;
}

tl::expected<ShapeFitResult, std::string>
fitBestShape(const std::shared_ptr<const pcl::PointCloud<pcl::PointXYZRGB>>& cloud,
             const std::shared_ptr<const pcl::PointIndices>& indices, double distance_threshold,
             double bin_floor_z, const std::string& forced_shape)
{
  // If a specific shape is forced, only fit that one.
  if (forced_shape == "cylinder")
  {
    spdlog::info("  Forced shape: cylinder");
    return fitCylinder(cloud, indices, distance_threshold, bin_floor_z);
  }
  if (forced_shape == "box")
  {
    spdlog::info("  Forced shape: box");
    return fitBox(cloud, indices, bin_floor_z);
  }
  if (forced_shape == "sphere")
  {
    spdlog::info("  Forced shape: sphere");
    return fitSphere(cloud, indices, distance_threshold, bin_floor_z);
  }

  // Auto-detect: try all three and pick smallest volume.
  const auto box_result = fitBox(cloud, indices, bin_floor_z);
  const auto cylinder_result = fitCylinder(cloud, indices, distance_threshold, bin_floor_z);
  const auto sphere_result = fitSphere(cloud, indices, distance_threshold, bin_floor_z);

  if (box_result.has_value())
  {
    spdlog::info("  Box fit: volume={:.6f} m^3, dims=[{:.4f}, {:.4f}, {:.4f}]", box_result->volume,
                 box_result->primitive.dimensions[0], box_result->primitive.dimensions[1],
                 box_result->primitive.dimensions[2]);
  }
  else
  {
    spdlog::warn("  Box fit FAILED: {}", box_result.error());
  }
  if (cylinder_result.has_value())
  {
    spdlog::info("  Cylinder fit: volume={:.6f} m^3, height={:.4f}, radius={:.4f}", cylinder_result->volume,
                 cylinder_result->primitive.dimensions[0], cylinder_result->primitive.dimensions[1]);
  }
  else
  {
    spdlog::warn("  Cylinder fit FAILED: {}", cylinder_result.error());
  }
  if (sphere_result.has_value())
  {
    spdlog::info("  Sphere fit: volume={:.6f} m^3, radius={:.4f}", sphere_result->volume,
                 sphere_result->primitive.dimensions[0]);
  }
  else
  {
    spdlog::warn("  Sphere fit FAILED: {}", sphere_result.error());
  }

  std::vector<ShapeFitResult> candidates;
  if (box_result.has_value())
    candidates.push_back(box_result.value());
  if (cylinder_result.has_value())
    candidates.push_back(cylinder_result.value());
  if (sphere_result.has_value())
    candidates.push_back(sphere_result.value());

  if (candidates.empty())
  {
    return tl::make_unexpected("All shape fitting methods failed.");
  }

  auto best = std::min_element(candidates.begin(), candidates.end(),
                               [](const ShapeFitResult& a, const ShapeFitResult& b) { return a.volume < b.volume; });

  spdlog::info("  Best fit: {} (volume={:.6f} m^3)", best->shape_type, best->volume);
  return *best;
}

}  // namespace get_graspable_grocery_objects_from_masks3d
