// Copyright 2026 PickNik Inc.
// All rights reserved.
//
// Unauthorized copying of this code base via any medium is strictly prohibited.
// Proprietary and confidential.

#pragma once

#include <pcl/PointIndices.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <Eigen/Dense>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <string>
#include <tl_expected/expected.hpp>

namespace get_graspable_grocery_objects_from_masks3d
{

/**
 * @brief Result of fitting a primitive shape to a point cloud segment.
 */
struct ShapeFitResult
{
  /** @brief The fitted primitive (BOX, CYLINDER, or SPHERE). */
  shape_msgs::msg::SolidPrimitive primitive;

  /** @brief Transform from the cloud frame to the shape's principal-axis-aligned frame. */
  Eigen::Isometry3d origin;

  /** @brief Bounding volume of the fitted shape in cubic meters. */
  double volume;

  /** @brief Human-readable shape type: "box", "cylinder", or "sphere". */
  std::string shape_type;
};

/**
 * @brief Fit a principal-axis-aligned bounding box to the point cloud segment.
 * @param cloud Input point cloud.
 * @param indices Indices of points in this segment.
 * @return ShapeFitResult for a BOX, or error string.
 */
[[nodiscard]] tl::expected<ShapeFitResult, std::string>
fitBox(const std::shared_ptr<const pcl::PointCloud<pcl::PointXYZRGB>>& cloud,
       const std::shared_ptr<const pcl::PointIndices>& indices,
       double bin_floor_z = std::numeric_limits<double>::quiet_NaN());

/**
 * @brief Fit a cylinder to the point cloud segment using RANSAC.
 * @param cloud Input point cloud.
 * @param indices Indices of points in this segment.
 * @param distance_threshold RANSAC inlier distance threshold in meters.
 * @return ShapeFitResult for a CYLINDER, or error string.
 */
/**
 * @param bin_floor_z Z coordinate of the bin floor in the cloud frame. The cylinder bottom
 *                    is clamped to this value when the top-down view doesn't reveal the full height.
 *                    Set to NaN to disable floor clamping.
 */
[[nodiscard]] tl::expected<ShapeFitResult, std::string>
fitCylinder(const std::shared_ptr<const pcl::PointCloud<pcl::PointXYZRGB>>& cloud,
            const std::shared_ptr<const pcl::PointIndices>& indices, double distance_threshold,
            double bin_floor_z = std::numeric_limits<double>::quiet_NaN());

/**
 * @brief Fit a sphere to the point cloud segment using RANSAC.
 * @param cloud Input point cloud.
 * @param indices Indices of points in this segment.
 * @param distance_threshold RANSAC inlier distance threshold in meters.
 * @return ShapeFitResult for a SPHERE, or error string.
 */
[[nodiscard]] tl::expected<ShapeFitResult, std::string>
fitSphere(const std::shared_ptr<const pcl::PointCloud<pcl::PointXYZRGB>>& cloud,
          const std::shared_ptr<const pcl::PointIndices>& indices, double distance_threshold,
          double bin_floor_z = std::numeric_limits<double>::quiet_NaN());

/**
 * @brief Fit box, cylinder, and sphere to a point cloud segment and return the best fit.
 * @details The best fit is selected by smallest bounding volume. This assumes the tightest
 *          enclosing primitive is the most representative shape for the object.
 * @param cloud Input point cloud.
 * @param indices Indices of points in this segment.
 * @param distance_threshold RANSAC inlier distance threshold for cylinder/sphere fitting.
 * @return The ShapeFitResult with the smallest volume, or error string if all fits fail.
 */
/**
 * @param forced_shape If non-empty, only fit this shape type ("cylinder", "box", or "sphere").
 *                     If empty, tries all three and selects the best.
 */
[[nodiscard]] tl::expected<ShapeFitResult, std::string>
fitBestShape(const std::shared_ptr<const pcl::PointCloud<pcl::PointXYZRGB>>& cloud,
             const std::shared_ptr<const pcl::PointIndices>& indices, double distance_threshold,
             double bin_floor_z = std::numeric_limits<double>::quiet_NaN(), const std::string& forced_shape = "");

}  // namespace get_graspable_grocery_objects_from_masks3d
