// Ring-based fixed-size range-image interpolation.
#include "interpolation.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <chrono>
#include <iterator>
#include <limits>
#include <stdexcept>
#include <utility>

#include <Eigen/Geometry>

namespace lidar_camera_fusion
{
namespace
{
constexpr float nan_f = std::numeric_limits<float>::quiet_NaN();
constexpr int directional_search_radius_cols = 3;
constexpr int directional_candidate_count = 2 * directional_search_radius_cols + 1;
constexpr double directional_offset_penalty_m = 0.05;

struct RowInterpolationInfo
{
  int lower_row = -1;
  int upper_row = -1;
  bool same_ring = false;
  double alpha = 0.0;
  std::array<int, directional_candidate_count> lower_offsets{};
  std::array<int, directional_candidate_count> upper_offsets{};
};

inline std::size_t index_of(int row, int col, int cols)
{
  return static_cast<std::size_t>(row) * static_cast<std::size_t>(cols) +
         static_cast<std::size_t>(col);
}

inline int wrap_column(int col, int cols)
{
  // Directional interpolation shifts a valid column by only a few bins.
  // Handle the common single-wrap case without integer modulo; keep a
  // fallback so the helper remains correct even for unusually small widths.
  if (col >= 0 && col < cols) return col;
  if (col < 0 && col >= -cols) return col + cols;
  if (col >= cols && col < 2 * cols) return col - cols;

  col %= cols;
  if (col < 0) col += cols;
  return col;
}

inline bool valid_range(float value)
{
  return std::isfinite(value) && value > 0.0F;
}

inline double deg_to_rad(double degrees)
{
  return degrees * pi / 180.0;
}

using SteadyClock = std::chrono::steady_clock;

inline double elapsed_ms(
  const SteadyClock::time_point & begin,
  const SteadyClock::time_point & end)
{
  return std::chrono::duration<double, std::milli>(end - begin).count();
}

// Horizontal convention:
//   col = 0       -> -pi (rear seam)
//   col = cols/2  ->  0   (forward +X)
//   increasing col follows increasing atan2(y, x).
inline int azimuth_to_column(double azimuth, int cols)
{
  const double scaled = azimuth * static_cast<double>(cols) / (2.0 * pi);
  long col = std::lround(scaled) + cols / 2;
  col %= cols;
  if (col < 0) col += cols;
  return static_cast<int>(col);
}

inline double column_to_azimuth(int col, int cols)
{
  return 2.0 * pi *
    (static_cast<double>(col) - static_cast<double>(cols / 2)) /
    static_cast<double>(cols);
}
}  // namespace

int Settings::horizontal_columns() const
{
  return static_cast<int>(std::lround(360.0 / horizontal_resolution_deg));
}

void Settings::validate() const
{
  if (!std::isfinite(minlen) || !std::isfinite(maxlen) || minlen < 0.0 || maxlen <= minlen)
    throw std::invalid_argument("maxlen must be finite and > minlen >= 0");

  if (input_rows < 2 || output_rows < 2)
    throw std::invalid_argument("input_rows and output_rows must both be >= 2");

  if (!std::isfinite(horizontal_resolution_deg) || horizontal_resolution_deg <= 0.0 ||
      horizontal_resolution_deg > 360.0)
    throw std::invalid_argument("horizontal_resolution_deg must be in (0, 360]");

  const int cols = horizontal_columns();
  if (cols < 2 || cols > 1000000)
    throw std::invalid_argument("horizontal_resolution_deg produces an invalid range-image width");

  if (vertical_angles_deg.size() != static_cast<std::size_t>(input_rows))
    throw std::invalid_argument("vertical_angles_deg length must equal input_rows");

  for (double angle : vertical_angles_deg)
    if (!std::isfinite(angle) || angle <= -90.0 || angle >= 90.0)
      throw std::invalid_argument("vertical_angles_deg values must be finite and inside (-90, 90) degrees");

  std::vector<double> sorted_angles = vertical_angles_deg;
  std::sort(sorted_angles.begin(), sorted_angles.end());
  for (std::size_t i = 1; i < sorted_angles.size(); ++i)
    if (!(sorted_angles[i] > sorted_angles[i - 1]))
      throw std::invalid_argument("vertical_angles_deg must contain unique elevations");

  if (!std::isfinite(max_interpolation_range_gap_m) || max_interpolation_range_gap_m < 0.0)
    throw std::invalid_argument("max_interpolation_range_gap_m must be finite and >= 0");

  if (!std::isfinite(ground_angle))
    throw std::invalid_argument("ang_ground must be finite");

  if (!std::isfinite(min_fov) || !std::isfinite(max_fov) || min_fov >= max_fov ||
      min_fov < -pi / 2.0 || max_fov > 3.0 * pi / 2.0)
    throw std::invalid_argument("FOV must satisfy -pi/2 <= min_ang_FOV < max_ang_FOV <= 3*pi/2");

  const double cells = static_cast<double>(output_rows) * static_cast<double>(cols);
  if (!std::isfinite(cells) || cells > static_cast<double>(std::numeric_limits<int>::max()))
    throw std::invalid_argument("range-image dimensions are too large");
}

Result interpolate(
  const std::vector<RingPoint> & input,
  const Settings & s,
  Mode mode)
{
  const auto total_begin = SteadyClock::now();
  s.validate();

  Result result;
  result.raw_rows = s.input_rows;
  result.interpolated_rows = s.output_rows;
  result.cols = s.horizontal_columns();
  result.raw_ranges.assign(
    static_cast<std::size_t>(result.raw_rows) * result.cols, nan_f);
  result.interpolated_ranges.assign(
    static_cast<std::size_t>(result.interpolated_rows) * result.cols, nan_f);

  // Keep the physical ring calibration separate from image row numbering.
  // Image convention is conventional image coordinates:
  //   row 0              = highest elevation
  //   row input_rows - 1 = lowest elevation
  std::vector<std::pair<double, int>> rings_ascending;
  rings_ascending.reserve(s.input_rows);
  for (int ring = 0; ring < s.input_rows; ++ring)
    rings_ascending.emplace_back(s.vertical_angles_deg[ring], ring);
  std::sort(rings_ascending.begin(), rings_ascending.end(),
    [](const auto & a, const auto & b) {return a.first < b.first;});

  std::vector<int> ring_to_image_row(static_cast<std::size_t>(s.input_rows));
  for (int image_row = 0; image_row < s.input_rows; ++image_row) {
    const int ascending_index = s.input_rows - 1 - image_row;
    const int ring = rings_ascending[static_cast<std::size_t>(ascending_index)].second;
    ring_to_image_row[static_cast<std::size_t>(ring)] = image_row;
  }

  const auto setup_end = SteadyClock::now();
  result.timing.setup_ms = elapsed_ms(total_begin, setup_end);

  // Build a fixed HxW raw range image directly from ring and azimuth.
  // Horizontal centre is forward +X; vertical top is the highest laser ring.
  const auto raw_begin = SteadyClock::now();
  for (const auto & p : input) {
    if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) continue;
    if (p.ring >= static_cast<std::uint16_t>(s.input_rows)) continue;

    const double range = std::sqrt(
      static_cast<double>(p.x) * p.x +
      static_cast<double>(p.y) * p.y +
      static_cast<double>(p.z) * p.z);
    if (!std::isfinite(range) || range < s.minlen || range > s.maxlen || range <= 0.0) continue;

    const double azimuth = std::atan2(static_cast<double>(p.y), static_cast<double>(p.x));
    const int col = azimuth_to_column(azimuth, result.cols);
    const int row = ring_to_image_row[static_cast<std::size_t>(p.ring)];

    const std::size_t idx = index_of(row, col, result.cols);
    const float range_f = static_cast<float>(range);
    if (!valid_range(result.raw_ranges[idx]) || range_f < result.raw_ranges[idx])
      result.raw_ranges[idx] = range_f;
  }
  const auto raw_end = SteadyClock::now();
  result.timing.raw_range_ms = elapsed_ms(raw_begin, raw_end);

  const auto interpolation_begin = SteadyClock::now();
  const double min_elevation = rings_ascending.front().first;
  const double max_elevation = rings_ascending.back().first;
  const double dense_step = (max_elevation - min_elevation) /
    static_cast<double>(s.output_rows - 1);

  // Directional edge-aware vertical interpolation. All geometry that depends
  // only on the output row is precomputed once per frame. In particular, the
  // support rows, interpolation fraction, and rounded directional column
  // offsets do not depend on the horizontal column and should not be recomputed
  // for every candidate cell.
  std::vector<RowInterpolationInfo> row_info(static_cast<std::size_t>(s.output_rows));
  for (int out_row = 0; out_row < s.output_rows; ++out_row) {
    const double target_elevation = max_elevation - dense_step * out_row;

    auto upper = std::lower_bound(
      rings_ascending.begin(), rings_ascending.end(), target_elevation,
      [](const auto & entry, double value) {return entry.first < value;});

    int lower_ring = -1;
    int upper_ring = -1;
    double alpha = 0.0;

    if (upper == rings_ascending.begin()) {
      lower_ring = upper_ring = upper->second;
    } else if (upper == rings_ascending.end()) {
      lower_ring = upper_ring = rings_ascending.back().second;
    } else if (std::abs(upper->first - target_elevation) < 1e-12) {
      lower_ring = upper_ring = upper->second;
    } else {
      const auto lower = std::prev(upper);
      lower_ring = lower->second;
      upper_ring = upper->second;
      alpha = (target_elevation - lower->first) / (upper->first - lower->first);
    }

    auto & info = row_info[static_cast<std::size_t>(out_row)];
    info.lower_row = ring_to_image_row[static_cast<std::size_t>(lower_ring)];
    info.upper_row = ring_to_image_row[static_cast<std::size_t>(upper_ring)];
    info.same_ring = lower_ring == upper_ring;
    info.alpha = alpha;

    for (int direction = -directional_search_radius_cols;
      direction <= directional_search_radius_cols; ++direction)
    {
      const std::size_t candidate = static_cast<std::size_t>(
        direction + directional_search_radius_cols);
      info.lower_offsets[candidate] =
        static_cast<int>(std::lround(alpha * direction));
      info.upper_offsets[candidate] =
        static_cast<int>(std::lround((1.0 - alpha) * direction));
    }
  }

  // For a target cell between two physical rings, evaluate the vertical
  // support and a small set of diagonal supports. The candidate with the
  // smallest range discontinuity is selected, with a small penalty for
  // horizontal displacement so smooth interior surfaces still prefer the
  // vertical direction. Candidates crossing a large depth jump are rejected.
  for (int out_row = 0; out_row < s.output_rows; ++out_row) {
    const auto & info = row_info[static_cast<std::size_t>(out_row)];

    for (int col = 0; col < result.cols; ++col) {
      float dense = nan_f;

      if (info.same_ring) {
        const float raw = result.raw_ranges[index_of(info.lower_row, col, result.cols)];
        if (valid_range(raw)) dense = raw;
      } else {
        double best_score = std::numeric_limits<double>::infinity();

        for (int direction = -directional_search_radius_cols;
          direction <= directional_search_radius_cols; ++direction)
        {
          const std::size_t candidate = static_cast<std::size_t>(
            direction + directional_search_radius_cols);
          const int lower_col = wrap_column(
            col - info.lower_offsets[candidate], result.cols);
          const int upper_col = wrap_column(
            col + info.upper_offsets[candidate], result.cols);

          const float r0 =
            result.raw_ranges[index_of(info.lower_row, lower_col, result.cols)];
          const float r1 =
            result.raw_ranges[index_of(info.upper_row, upper_col, result.cols)];
          if (!valid_range(r0) || !valid_range(r1)) continue;

          const double gap = std::abs(static_cast<double>(r1) - r0);
          if (gap > s.max_interpolation_range_gap_m) continue;

          const double score = gap +
            directional_offset_penalty_m * static_cast<double>(std::abs(direction));
          if (score < best_score) {
            best_score = score;
            dense = static_cast<float>(
              (1.0 - info.alpha) * r0 + info.alpha * r1);
          }
        }
      }

      result.interpolated_ranges[index_of(out_row, col, result.cols)] = dense;
    }
  }
  const auto interpolation_end = SteadyClock::now();
  result.timing.directional_interpolation_ms =
    elapsed_ms(interpolation_begin, interpolation_end);

  const auto xyz_begin = SteadyClock::now();
  const float ground = static_cast<float>(s.ground_angle);
  Eigen::Matrix3f ground_correction;
  ground_correction <<
    std::cos(ground), 0.0F, std::sin(ground),
    0.0F,             1.0F, 0.0F,
   -std::sin(ground), 0.0F, std::cos(ground);

  // Reconstruct XYZ using the same top-to-bottom elevation convention.
  for (int row = 0; row < s.output_rows; ++row) {
    const double elevation_deg = max_elevation - dense_step * row;
    const double elevation = deg_to_rad(elevation_deg);
    const double cos_elevation = std::cos(elevation);
    const double sin_elevation = std::sin(elevation);

    for (int col = 0; col < result.cols; ++col) {
      const float range = result.interpolated_ranges[index_of(row, col, result.cols)];
      if (!valid_range(range)) continue;

      const double azimuth = column_to_azimuth(col, result.cols);

      if (mode == Mode::Fusion &&
          (azimuth < s.min_fov - pi / 2.0 ||
           azimuth > s.max_fov - pi / 2.0))
        continue;

      const float radial = static_cast<float>(range * cos_elevation);
      Eigen::Vector3f point(
        radial * static_cast<float>(std::cos(azimuth)),
        radial * static_cast<float>(std::sin(azimuth)),
        static_cast<float>(range * sin_elevation));
      point = ground_correction * point;

      if (point.allFinite())
        result.cloud.push_back(pcl::PointXYZ(point.x(), point.y(), point.z()));
    }
  }

  result.cloud.is_dense = true;
  result.cloud.height = 1;
  result.cloud.width = static_cast<std::uint32_t>(result.cloud.size());

  const auto xyz_end = SteadyClock::now();
  result.timing.xyz_reconstruction_ms = elapsed_ms(xyz_begin, xyz_end);
  result.timing.total_ms = elapsed_ms(total_begin, xyz_end);
  return result;
}

}  // namespace lidar_camera_fusion
