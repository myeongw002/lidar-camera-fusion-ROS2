// Adapted from EPVelasco/lidar-camera-fusion for ROS2 Humble (2026).
#pragma once
#include <armadillo>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/range_image/range_image_spherical.h>

namespace lidar_camera_fusion
{
constexpr double pi = 3.14159265358979323846;
struct Settings
{
  double maxlen = 100.0, minlen = 0.01;
  double x_resolution = 0.5, ang_y_resolution = 2.1;
  int interpolation = 20;
  double ground_angle = 0.6 * pi / 180.0, max_var = 50.0;
  bool filter = true;
  double min_fov = 0.4, max_fov = 3.0;
  void validate() const;
};

// Mode preserves the different variance statistics in the two upstream nodes.
enum class Mode { Interpolation, Fusion };
struct Result
{
  arma::mat ranges;
  pcl::PointCloud<pcl::PointXYZ> cloud;
};
Result interpolate(const pcl::PointCloud<pcl::PointXYZ> & input,
  pcl::RangeImageSpherical & range_image, const Settings & settings, Mode mode);
}  // namespace lidar_camera_fusion
