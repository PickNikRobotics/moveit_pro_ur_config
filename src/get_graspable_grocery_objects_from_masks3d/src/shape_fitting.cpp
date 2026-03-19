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

/** @brief Extract the subset of points from a cloud given indices. */
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
  return subset;
}
}  // namespace

tl::expected<ShapeFitResult, std::string>
fitBox(const std::shared_ptr<const pcl::PointCloud<pcl::PointXYZRGB>>& cloud,
       const std::shared_ptr<const pcl::PointIndices>& indices)
{
  if (static_cast<int>(indices->indices.size()) < kMinPointsForFitting)
  {
    return tl::make_unexpected("Too few points for box fitting.");
  }

  const auto subset = extractPoints(cloud, indices);

  // Compute PCA to get principal axes.
  pcl::PCA<pcl::PointXYZRGB> pca;
  pca.setInputCloud(subset);

  const Eigen::Vector4f centroid_4f = pca.getMean();
  const Eigen::Vector3f centroid = centroid_4f.head<3>();
  const Eigen::Matrix3f eigenvectors = pca.getEigenVectors();

  // Build transform from cloud frame to PCA-aligned frame.
  Eigen::Isometry3d origin = Eigen::Isometry3d::Identity();
  origin.translation() = centroid.cast<double>();
  origin.linear() = eigenvectors.cast<double>();

  // Project points into PCA frame and find extents.
  Eigen::Vector3f min_pt(std::numeric_limits<float>::max(), std::numeric_limits<float>::max(),
                         std::numeric_limits<float>::max());
  Eigen::Vector3f max_pt(std::numeric_limits<float>::lowest(), std::numeric_limits<float>::lowest(),
                         std::numeric_limits<float>::lowest());

  for (const auto& pt : subset->points)
  {
    const Eigen::Vector3f local = eigenvectors.transpose() * (Eigen::Vector3f(pt.x, pt.y, pt.z) - centroid);
    min_pt = min_pt.cwiseMin(local);
    max_pt = max_pt.cwiseMax(local);
  }

  const Eigen::Vector3f size = max_pt - min_pt;
  const Eigen::Vector3f box_center_local = (min_pt + max_pt) * 0.5f;

  // Adjust origin to be at the center of the bounding box.
  origin.translation() += (eigenvectors * box_center_local).cast<double>();

  ShapeFitResult result;
  result.origin = origin;
  result.shape_type = "box";
  result.primitive.type = shape_msgs::msg::SolidPrimitive::BOX;
  result.primitive.dimensions = { static_cast<double>(size.x()), static_cast<double>(size.y()),
                                  static_cast<double>(size.z()) };
  result.volume = static_cast<double>(size.x()) * static_cast<double>(size.y()) * static_cast<double>(size.z());

  return result;
}

tl::expected<ShapeFitResult, std::string>
fitCylinder(const std::shared_ptr<const pcl::PointCloud<pcl::PointXYZRGB>>& cloud,
            const std::shared_ptr<const pcl::PointIndices>& indices, double distance_threshold)
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

  // Project all points onto the axis to find the height extent.
  const Eigen::Vector3d axis_normalized = axis_dir.normalized();
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
  // CYLINDER dimensions: [height, radius].
  result.primitive.dimensions = { height, radius };
  result.volume = kPi * radius * radius * height;

  return result;
}

tl::expected<ShapeFitResult, std::string>
fitSphere(const std::shared_ptr<const pcl::PointCloud<pcl::PointXYZRGB>>& cloud,
          const std::shared_ptr<const pcl::PointIndices>& indices, double distance_threshold)
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
  // SPHERE dimensions: [radius].
  result.primitive.dimensions = { radius };
  result.volume = (4.0 / 3.0) * kPi * radius * radius * radius;

  return result;
}

tl::expected<ShapeFitResult, std::string>
fitBestShape(const std::shared_ptr<const pcl::PointCloud<pcl::PointXYZRGB>>& cloud,
             const std::shared_ptr<const pcl::PointIndices>& indices, double distance_threshold)
{
  const auto box_result = fitBox(cloud, indices);
  const auto cylinder_result = fitCylinder(cloud, indices, distance_threshold);
  const auto sphere_result = fitSphere(cloud, indices, distance_threshold);

  // Collect successful fits.
  std::vector<ShapeFitResult> candidates;
  if (box_result.has_value())
  {
    spdlog::info("  Box fit: volume={:.6f} m^3, dims=[{:.4f}, {:.4f}, {:.4f}]", box_result->volume,
                 box_result->primitive.dimensions[0], box_result->primitive.dimensions[1],
                 box_result->primitive.dimensions[2]);
    candidates.push_back(box_result.value());
  }
  if (cylinder_result.has_value())
  {
    spdlog::info("  Cylinder fit: volume={:.6f} m^3, height={:.4f}, radius={:.4f}", cylinder_result->volume,
                 cylinder_result->primitive.dimensions[0], cylinder_result->primitive.dimensions[1]);
    candidates.push_back(cylinder_result.value());
  }
  if (sphere_result.has_value())
  {
    spdlog::info("  Sphere fit: volume={:.6f} m^3, radius={:.4f}", sphere_result->volume,
                 sphere_result->primitive.dimensions[0]);
    candidates.push_back(sphere_result.value());
  }

  if (candidates.empty())
  {
    return tl::make_unexpected("All shape fitting methods failed.");
  }

  // Select the candidate with the smallest bounding volume.
  auto best = std::min_element(candidates.begin(), candidates.end(),
                               [](const ShapeFitResult& a, const ShapeFitResult& b) { return a.volume < b.volume; });

  spdlog::info("  Best fit: {} (volume={:.6f} m^3)", best->shape_type, best->volume);
  return *best;
}

}  // namespace get_graspable_grocery_objects_from_masks3d
