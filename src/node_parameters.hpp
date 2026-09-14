#pragma once
#include "interpolation.hpp"
#include <rclcpp/rclcpp.hpp>
#include <rcl_interfaces/msg/parameter_descriptor.hpp>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

namespace lidar_camera_fusion
{
template<typename T>
T parameter(rclcpp::Node & node, const std::string & name, const T & value)
{
  rcl_interfaces::msg::ParameterDescriptor descriptor;
  descriptor.read_only = true;
  return node.declare_parameter<T>(name, value, descriptor);
}
inline Settings read_settings(rclcpp::Node & node)
{
  Settings s;
  s.maxlen = parameter(node, "maxlen", s.maxlen);
  s.minlen = parameter(node, "minlen", s.minlen);
  s.x_resolution = parameter(node, "x_resolution", s.x_resolution);
  s.ang_y_resolution = parameter(node, "ang_Y_resolution", s.ang_y_resolution);
  const auto factor = parameter<int64_t>(node, "y_interpolation", s.interpolation);
  if (factor <= 0 || factor > std::numeric_limits<int>::max())
    throw std::invalid_argument("y_interpolation must be a positive int32 integer");
  s.interpolation = static_cast<int>(factor);
  s.ground_angle = parameter(node, "ang_ground", s.ground_angle);
  s.max_var = parameter(node, "max_var", s.max_var);
  s.filter = parameter(node, "filter_output_pc", s.filter);
  s.min_fov = parameter(node, "min_ang_FOV", s.min_fov);
  s.max_fov = parameter(node, "max_ang_FOV", s.max_fov);
  s.validate();
  return s;
}
inline std::string topic(rclcpp::Node & node, const std::string & name, const std::string & fallback)
{
  auto value = parameter(node, name, fallback);
  if (value.empty()) throw std::invalid_argument(name + " must not be empty");
  return value;
}
inline std::vector<double> calibration(rclcpp::Node & node, const std::string & name,
  std::size_t length)
{
  auto values = parameter(node, name, std::vector<double>{});
  if (values.size() != length)
    throw std::invalid_argument(name + " must contain exactly " + std::to_string(length) + " doubles; load config/calibration.yaml");
  for (double value : values)
    if (!std::isfinite(value) || std::abs(value) > std::numeric_limits<float>::max())
      throw std::invalid_argument(name + " values must be finite and float-representable");
  return values;
}
template<typename Node>
int run(int argc, char ** argv, const char * name)
{
  rclcpp::init(argc, argv);
  int status = 0;
  try {rclcpp::spin(std::make_shared<Node>());}
  catch (const std::exception & e) {
    RCLCPP_FATAL(rclcpp::get_logger(name), "%s", e.what());
    status = 1;
  }
  rclcpp::shutdown();
  return status;
}
}  // namespace lidar_camera_fusion
