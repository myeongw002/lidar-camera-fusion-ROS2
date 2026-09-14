#include "interpolation.hpp"
#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>

using namespace lidar_camera_fusion;
void require(bool condition, const char * message)
{
  if (!condition) throw std::runtime_error(message);
}
int main()
{
  Settings s;
  s.x_resolution = 1.0;
  s.ang_y_resolution = 2.0;
  s.interpolation = 4;
  s.filter = false;
  s.ground_angle = 0;
  s.validate();
  pcl::RangeImageSpherical range_image;
  pcl::PointCloud<pcl::PointXYZ> cloud;
  require(interpolate(cloud, range_image, s, Mode::Interpolation).ranges.is_empty(), "empty input");
  cloud.push_back(pcl::PointXYZ(std::numeric_limits<float>::quiet_NaN(), 0, 0));
  cloud.push_back(pcl::PointXYZ(std::numeric_limits<float>::infinity(), 0, 0));
  cloud.push_back(pcl::PointXYZ(1000, 0, 0));
  require(interpolate(cloud, range_image, s, Mode::Interpolation).ranges.is_empty(), "invalid input");
  cloud.clear();
  // Complete synthetic VLP16-like rings at constant 10 m spherical range.
  for (int elevation = -15; elevation <= 15; elevation += 2) {
    for (int azimuth = 0; azimuth < 360; ++azimuth) {
      const double el = elevation * pi / 180, az = azimuth * pi / 180;
      cloud.push_back(pcl::PointXYZ(10 * std::cos(el) * std::cos(az),
        10 * std::cos(el) * std::sin(az), 10 * std::sin(el)));
    }
  }
  const auto dense = interpolate(cloud, range_image, s, Mode::Interpolation);
  require(dense.cloud.size() > cloud.size(), "vertical densification");
  for (const auto & p : dense.cloud) {
    require(std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z), "finite XYZ");
    require(std::abs(std::sqrt(p.x*p.x+p.y*p.y+p.z*p.z) - 10) < 1e-4, "range preserved");
  }
  const auto fusion = interpolate(cloud, range_image, s, Mode::Fusion);
  require(!fusion.cloud.empty() && fusion.cloud.size() < dense.cloud.size(), "camera FOV");
  for (const auto & p : fusion.cloud) {
    const double az = std::atan2(p.y, p.x);
    require(az >= s.min_fov - pi/2 - 1e-6 && az <= s.max_fov - pi/2 + 1e-6, "FOV bounds");
  }
  s.ground_angle = 0.1;
  const auto rotated = interpolate(cloud, range_image, s, Mode::Interpolation);
  require(rotated.cloud.size() == dense.cloud.size(), "rotation count");
  for (std::size_t i = 0; i < dense.cloud.size(); ++i) {
    const auto & a = dense.cloud[i]; const auto & b = rotated.cloud[i];
    require(std::abs(b.x - (std::cos(0.1)*a.x + std::sin(0.1)*a.z)) < 1e-5, "ground rotation");
  }
  s.filter = true;
  require(!interpolate(cloud, range_image, s, Mode::Fusion).cloud.empty(), "variance filtering");
  s.interpolation = 0;
  bool rejected = false;
  try {s.validate();} catch (const std::invalid_argument &) {rejected = true;}
  require(rejected, "invalid settings rejected");
  std::cout << "Interpolation, finite input, FOV, ground correction checks passed\n";
}
