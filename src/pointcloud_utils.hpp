#pragma once

#include "interpolation.hpp"

#include <cstdint>
#include <stdexcept>
#include <string>
#include <vector>

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

namespace lidar_camera_fusion
{
inline const sensor_msgs::msg::PointField & require_field(
  const sensor_msgs::msg::PointCloud2 & msg,
  const std::string & name)
{
  for (const auto & field : msg.fields)
    if (field.name == name) return field;
  throw std::invalid_argument("PointCloud2 is missing required field '" + name + "'");
}

inline std::vector<RingPoint> ring_points_from_ros(
  const sensor_msgs::msg::PointCloud2 & msg)
{
  const auto & ring_field = require_field(msg, "ring");
  if (ring_field.datatype != sensor_msgs::msg::PointField::UINT16)
    throw std::invalid_argument("PointCloud2 field 'ring' must be UINT16 for this VLP-16 pipeline");

  require_field(msg, "x");
  require_field(msg, "y");
  require_field(msg, "z");

  std::vector<RingPoint> points;
  points.reserve(static_cast<std::size_t>(msg.width) * msg.height);

  sensor_msgs::PointCloud2ConstIterator<float> x(msg, "x");
  sensor_msgs::PointCloud2ConstIterator<float> y(msg, "y");
  sensor_msgs::PointCloud2ConstIterator<float> z(msg, "z");
  sensor_msgs::PointCloud2ConstIterator<std::uint16_t> ring(msg, "ring");

  for (; x != x.end(); ++x, ++y, ++z, ++ring)
    points.push_back(RingPoint{*x, *y, *z, *ring});

  return points;
}

}  // namespace lidar_camera_fusion
