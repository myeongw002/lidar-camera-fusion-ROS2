// ROS2 adaptation of the upstream spherical interpolation algorithm.
#include "interpolation.hpp"
#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <Eigen/Geometry>
#include <pcl/common/angles.h>

namespace lidar_camera_fusion
{
void Settings::validate() const
{
  if (!std::isfinite(minlen) || !std::isfinite(maxlen) || minlen < 0 || maxlen <= minlen ||
    maxlen > std::numeric_limits<float>::max())
    throw std::invalid_argument("maxlen must be finite, float-representable and > minlen >= 0");
  if (!std::isfinite(x_resolution) || !std::isfinite(ang_y_resolution) ||
    x_resolution <= 0 || ang_y_resolution <= 0 || x_resolution > 360 || ang_y_resolution > 180)
    throw std::invalid_argument("x_resolution and ang_Y_resolution must be positive degrees (<=360, <=180)");
  if (interpolation <= 0) throw std::invalid_argument("y_interpolation must be a positive integer");
  if (!std::isfinite(ground_angle) || !std::isfinite(max_var) || max_var < 0)
    throw std::invalid_argument("ang_ground must be finite; max_var must be finite and nonnegative");
  if (!std::isfinite(min_fov) || !std::isfinite(max_fov) || min_fov >= max_fov ||
    min_fov < -pi / 2 || max_fov > 3 * pi / 2)
    throw std::invalid_argument("FOV must satisfy -pi/2 <= min_ang_FOV < max_ang_FOV <= 3*pi/2");
  // PCL uses signed image dimensions; reject settings that could overflow them.
  const double cells = (std::ceil(360 / x_resolution) + 2) *
    (std::ceil(180 / ang_y_resolution) + 2) * interpolation;
  if (!std::isfinite(cells) || cells > std::numeric_limits<int>::max())
    throw std::invalid_argument("angular resolution/y_interpolation would overflow image dimensions");
}

Result interpolate(const pcl::PointCloud<pcl::PointXYZ> & input,
  pcl::RangeImageSpherical & range_image, const Settings & s, Mode mode)
{
  Result result;
  pcl::PointCloud<pcl::PointXYZ> filtered;
  for (const auto & p : input) {
    if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) continue;
    const double distance = std::hypot(p.x, p.y);
    if (distance >= s.minlen && distance <= s.maxlen) filtered.push_back(p);
  }
  if (filtered.empty()) return result;
  range_image.pcl::RangeImage::createFromPointCloud(filtered,
    pcl::deg2rad(static_cast<float>(s.x_resolution)),
    pcl::deg2rad(static_cast<float>(s.ang_y_resolution)),
    pcl::deg2rad(360.0f), pcl::deg2rad(180.0f), Eigen::Affine3f::Identity(),
    pcl::RangeImage::LASER_FRAME, 0.0f, 0.0f, 0);
  if (range_image.width < 2 || range_image.height < 2) return result;
  arma::mat ranges(range_image.height, range_image.width, arma::fill::zeros);
  arma::mat heights(ranges.n_rows, ranges.n_cols, arma::fill::zeros);
  bool usable = false;
  for (arma::uword j = 0; j < ranges.n_cols; ++j) {
    for (arma::uword i = 0; i < ranges.n_rows; ++i) {
      const auto & p = range_image.getPoint(static_cast<int>(j), static_cast<int>(i));
      if (!std::isfinite(p.range) || !std::isfinite(p.z) ||
        p.range < s.minlen || p.range > s.maxlen) continue;
      ranges(i, j) = p.range;
      heights(i, j) = p.z;
      usable = usable || p.range > 0;
    }
  }
  if (!usable) return result;
  const arma::vec x = arma::regspace(1, ranges.n_cols);
  const arma::vec y = arma::regspace(1, ranges.n_rows);
  const arma::vec xi = arma::regspace(x.min(), 1.0, x.max());
  const arma::vec yi = arma::regspace(y.min(), 1.0 / s.interpolation, y.max());
  arma::mat zi, zzi;
  arma::interp2(x, y, ranges, xi, yi, zi, "linear");
  arma::interp2(x, y, heights, xi, yi, zzi, "linear");
  for (arma::uword n = 0; n < zi.n_elem; ++n) {
    if (!std::isfinite(zi[n]) || !std::isfinite(zzi[n])) {zi[n] = 0; zzi[n] = 0;}
  }
  arma::mat out = zi;
  const auto factor = static_cast<arma::uword>(s.interpolation);
  for (arma::uword i = 0; i < zi.n_rows; ++i) {
    for (arma::uword j = 0; j < zi.n_cols; ++j) {
      if (zi(i, j) != 0) continue;
      // Preserve the upstream strict boundaries and non-cascading mask.
      if (i + factor < zi.n_rows)
        for (arma::uword k = 1; k <= factor; ++k) out(i + k, j) = 0;
      if (i > factor)
        for (arma::uword k = 1; k <= factor; ++k) out(i - k, j) = 0;
    }
  }
  if (mode == Mode::Interpolation) zi = out;
  if (s.filter && zi.n_cols > 5) {
    for (arma::uword i = 0; i < (zi.n_rows - 1) / factor; ++i) {
      // Upstream intentionally leaves the last five columns unfiltered.
      for (arma::uword j = 0; j < zi.n_cols - 5; ++j) {
        double mean = 0, variance = 0;
        for (arma::uword k = 0; k < factor; ++k) mean += zi(i * factor + k, j);
        mean /= factor;
        for (arma::uword k = 0; k < factor; ++k)
          variance += std::pow(zi(i * factor + k, j) - mean, 2.0);
        if (mode == Mode::Interpolation) variance = std::sqrt(variance / factor);
        if (variance > s.max_var)
          for (arma::uword k = 0; k < factor; ++k) out(i * factor + k, j) = 0;
      }
    }
  }
  result.ranges = out;
  const float angle = static_cast<float>(s.ground_angle);
  Eigen::Matrix3f correction;
  correction << std::cos(angle), 0, std::sin(angle), 0, 1, 0,
    -std::sin(angle), 0, std::cos(angle);
  if (out.n_rows <= factor) return result;
  // Preserve upstream omission of the final interpolation block.
  for (arma::uword i = 0; i < out.n_rows - factor; ++i) {
    for (arma::uword j = 0; j < out.n_cols; ++j) {
      const float azimuth = pi - 2.0 * pi * j / out.n_cols;
      if (mode == Mode::Fusion &&
        (azimuth < s.min_fov - pi / 2 || azimuth > s.max_fov - pi / 2)) continue;
      const float range = out(i, j);
      const double z = zzi(i, j);
      if (!std::isfinite(range) || range <= 0 || !std::isfinite(z)) continue;
      double radial_squared = double(range) * range - z * z;
      const double tolerance = 8 * std::numeric_limits<float>::epsilon() *
        std::max(double(range) * range, z * z);
      if (!std::isfinite(radial_squared) || radial_squared < -tolerance) continue;
      radial_squared = std::max(0.0, radial_squared);
      const Eigen::Vector3f p = correction * Eigen::Vector3f(
        std::sqrt(radial_squared) * std::cos(azimuth),
        std::sqrt(radial_squared) * std::sin(azimuth), z);
      if (p.allFinite()) result.cloud.push_back(pcl::PointXYZ(p.x(), p.y(), p.z()));
    }
  }
  result.cloud.is_dense = true;
  return result;
}
}  // namespace lidar_camera_fusion
