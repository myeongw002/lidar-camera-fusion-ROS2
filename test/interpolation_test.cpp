#include "interpolation.hpp"

#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <vector>

using namespace lidar_camera_fusion;

namespace
{
void require(bool condition, const char * message)
{
  if (!condition) throw std::runtime_error(message);
}

std::vector<RingPoint> make_synthetic_scan(const Settings & s, float range)
{
  std::vector<RingPoint> points;
  const int cols = s.horizontal_columns();
  points.reserve(static_cast<std::size_t>(s.input_rows) * cols);

  for (int ring = 0; ring < s.input_rows; ++ring) {
    const double elevation = s.vertical_angles_deg[ring] * pi / 180.0;
    for (int col = 0; col < cols; ++col) {
      const double azimuth = 2.0 * pi * (static_cast<double>(col) + 0.5) / cols;
      points.push_back(RingPoint{
        static_cast<float>(range * std::cos(elevation) * std::cos(azimuth)),
        static_cast<float>(range * std::cos(elevation) * std::sin(azimuth)),
        static_cast<float>(range * std::sin(elevation)),
        static_cast<std::uint16_t>(ring)});
    }
  }
  return points;
}
}  // namespace

int main()
{
  Settings s;
  s.minlen = 0.3;
  s.maxlen = 50.0;
  s.input_rows = 16;
  s.output_rows = 64;
  s.horizontal_resolution_deg = 1.0;
  s.max_interpolation_range_gap_m = 2.0;
  s.ground_angle = 0.0;
  s.validate();

  require(s.horizontal_columns() == 360, "horizontal width");

  const auto empty = interpolate({}, s, Mode::Interpolation);
  require(empty.raw_rows == 16 && empty.interpolated_rows == 64 && empty.cols == 360,
    "fixed dimensions on empty input");
  require(empty.cloud.empty(), "empty input cloud");

  auto scan = make_synthetic_scan(s, 10.0F);
  const auto dense = interpolate(scan, s, Mode::Interpolation);
  require(dense.raw_ranges.size() == 16u * 360u, "raw 16xW range image");
  require(dense.interpolated_ranges.size() == 64u * 360u, "dense 64xW range image");
  require(dense.cloud.size() == 64u * 360u, "dense point reconstruction");

  for (float value : dense.raw_ranges)
    require(std::isfinite(value) && std::abs(value - 10.0F) < 1e-3F, "raw metric range");
  for (float value : dense.interpolated_ranges)
    require(std::isfinite(value) && std::abs(value - 10.0F) < 1e-3F, "interpolated metric range");

  for (const auto & p : dense.cloud) {
    const double range = std::sqrt(p.x*p.x + p.y*p.y + p.z*p.z);
    require(std::isfinite(range) && std::abs(range - 10.0) < 1e-3, "XYZ range preservation");
  }

  const auto fusion = interpolate(scan, s, Mode::Fusion);
  require(!fusion.cloud.empty() && fusion.cloud.size() < dense.cloud.size(), "camera FOV crop");

  // A depth discontinuity between adjacent raw rings must not be bridged.
  auto discontinuity = scan;
  const int test_col = 20;
  const int cols = s.horizontal_columns();
  const std::size_t point_index = static_cast<std::size_t>(8 * cols + test_col);
  const double elevation = s.vertical_angles_deg[8] * pi / 180.0;
  const double azimuth = 2.0 * pi * (static_cast<double>(test_col) + 0.5) / cols;
  discontinuity[point_index] = RingPoint{
    static_cast<float>(20.0 * std::cos(elevation) * std::cos(azimuth)),
    static_cast<float>(20.0 * std::cos(elevation) * std::sin(azimuth)),
    static_cast<float>(20.0 * std::sin(elevation)), 8};
  const auto discontinuous = interpolate(discontinuity, s, Mode::Interpolation);
  bool found_invalid_dense_cell = false;
  for (int row = 0; row < s.output_rows; ++row) {
    const float value = discontinuous.interpolated_ranges[
      static_cast<std::size_t>(row) * cols + test_col];
    if (!std::isfinite(value)) {
      found_invalid_dense_cell = true;
      break;
    }
  }
  require(found_invalid_dense_cell, "range-gap boundary suppression");

  // Closest return wins when multiple points map to the same ring/azimuth cell.
  auto collision = scan;
  collision.push_back(RingPoint{5.0F, 0.0F, 0.0F, 7});
  const auto collided = interpolate(collision, s, Mode::Interpolation);
  require(std::isfinite(collided.raw_ranges[7u * cols]) &&
    collided.raw_ranges[7u * cols] < 6.0F, "nearest return per cell");

  s.ground_angle = 0.1;
  const auto rotated = interpolate(scan, s, Mode::Interpolation);
  require(rotated.cloud.size() == dense.cloud.size(), "ground correction point count");

  s.horizontal_resolution_deg = 0.0;
  bool rejected = false;
  try {s.validate();} catch (const std::invalid_argument &) {rejected = true;}
  require(rejected, "invalid horizontal resolution rejected");

  std::cout << "Fixed ring range-image interpolation checks passed\n";
  return 0;
}
