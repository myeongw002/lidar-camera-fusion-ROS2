// Fixed-size ring range-image interpolation for ROS2 Humble.
#pragma once

#include <cstdint>
#include <vector>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace lidar_camera_fusion
{
constexpr double pi = 3.14159265358979323846;

struct RingPoint
{
  float x = 0.0F;
  float y = 0.0F;
  float z = 0.0F;
  std::uint16_t ring = 0;
};

struct Settings
{
  double maxlen = 100.0;
  double minlen = 0.01;

  int input_rows = 16;
  int output_rows = 64;
  double horizontal_resolution_deg = 0.2;

  // Angles are indexed by the incoming PointCloud2 ring value. The standard
  // ROS Velodyne VLP-16 cloud uses elevation-sorted ring indices 0..15.
  std::vector<double> vertical_angles_deg{
    -15.0, -13.0, -11.0, -9.0, -7.0, -5.0, -3.0, -1.0,
      1.0,   3.0,   5.0,  7.0,  9.0, 11.0, 13.0, 15.0};

  // Interpolate only between two valid neighboring rings whose ranges are
  // sufficiently similar. This prevents bridging obvious depth boundaries.
  double max_interpolation_range_gap_m = 2.0;

  // Preserve the existing optional ground correction and fusion FOV.
  double ground_angle = 0.6 * pi / 180.0;
  double min_fov = 0.4;
  double max_fov = 3.0;

  void validate() const;
  int horizontal_columns() const;
};

enum class Mode { Interpolation, Fusion };

struct Timing
{
  double setup_ms = 0.0;
  double raw_range_ms = 0.0;
  double directional_interpolation_ms = 0.0;
  double xyz_reconstruction_ms = 0.0;
  double total_ms = 0.0;
};

struct Result
{
  int raw_rows = 0;
  int interpolated_rows = 0;
  int cols = 0;

  // Row-major metric ranges in metres. Invalid cells are quiet NaN.
  std::vector<float> raw_ranges;
  std::vector<float> interpolated_ranges;

  pcl::PointCloud<pcl::PointXYZ> cloud;
  Timing timing;
};

Result interpolate(
  const std::vector<RingPoint> & input,
  const Settings & settings,
  Mode mode);

}  // namespace lidar_camera_fusion
