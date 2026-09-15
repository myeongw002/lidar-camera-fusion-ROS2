// Native ROS2 Humble LiDAR-camera fusion consuming the interpolated cloud.
#include "node_parameters.hpp"

#include <algorithm>
#include <cmath>
#include <functional>
#include <limits>
#include <vector>

#include <Eigen/Core>
#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <opencv2/imgproc.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

namespace lidar_camera_fusion
{
namespace
{
struct ProjectedSample
{
  pcl::PointXYZ point;
  int px;
  int py;
  double range_m;
};

double percentile_from_sorted(const std::vector<double> & values, double percentile)
{
  if (values.empty()) return 0.0;
  if (values.size() == 1) return values.front();

  const double position = std::clamp(percentile, 0.0, 1.0) *
    static_cast<double>(values.size() - 1);
  const auto lower = static_cast<std::size_t>(std::floor(position));
  const auto upper = static_cast<std::size_t>(std::ceil(position));
  const double alpha = position - static_cast<double>(lower);
  return (1.0 - alpha) * values[lower] + alpha * values[upper];
}

cv::Scalar distance_color(double range_m, double min_range_m, double max_range_m)
{
  // Jet-like distance coloring:
  // near -> blue -> cyan -> green -> yellow -> red -> far.
  const double span = max_range_m - min_range_m;
  const double t = span > 1e-6 ?
    std::clamp((range_m - min_range_m) / span, 0.0, 1.0) : 0.5;

  const auto channel = [](double x) {
    return std::clamp(1.5 - std::abs(x), 0.0, 1.0);
  };

  const double r = channel(4.0 * t - 3.0);
  const double g = channel(4.0 * t - 2.0);
  const double b = channel(4.0 * t - 1.0);

  // OpenCV uses BGR channel order.
  return cv::Scalar(255.0 * b, 255.0 * g, 255.0 * r);
}
}  // namespace

class LidarCameraNode : public rclcpp::Node
{
  using Cloud = sensor_msgs::msg::PointCloud2;
  using Image = sensor_msgs::msg::Image;
  using Policy = message_filters::sync_policies::ApproximateTime<Cloud, Image>;

public:
  LidarCameraNode() : Node("lidar_camera_node")
  {
    const auto translation = calibration(*this, "matrix_file.tlc", 3);
    const auto rotation = calibration(*this, "matrix_file.rlc", 9);
    const auto camera = calibration(*this, "matrix_file.camera_matrix", 12);

    transform_.setIdentity();
    for (int row = 0; row < 3; ++row) {
      transform_(row, 3) = static_cast<float>(translation[row]);
      for (int col = 0; col < 3; ++col)
        transform_(row, col) = static_cast<float>(rotation[3 * row + col]);
      for (int col = 0; col < 4; ++col)
        camera_(row, col) = static_cast<float>(camera[4 * row + col]);
    }

    const auto queue = parameter<int64_t>(*this, "sync_queue_size", 10);
    if (queue <= 0 || queue > std::numeric_limits<int>::max())
      throw std::invalid_argument("sync_queue_size must be a positive int32 integer");

    overlay_percentile_low_ = parameter<double>(*this, "overlay_percentile_low", 0.05);
    overlay_percentile_high_ = parameter<double>(*this, "overlay_percentile_high", 0.95);
    overlay_range_smoothing_alpha_ =
      parameter<double>(*this, "overlay_range_smoothing_alpha", 0.2);

    if (!std::isfinite(overlay_percentile_low_) ||
        !std::isfinite(overlay_percentile_high_) ||
        overlay_percentile_low_ < 0.0 || overlay_percentile_high_ > 1.0 ||
        overlay_percentile_low_ >= overlay_percentile_high_)
      throw std::invalid_argument(
              "overlay percentiles must satisfy 0 <= low < high <= 1");

    if (!std::isfinite(overlay_range_smoothing_alpha_) ||
        overlay_range_smoothing_alpha_ <= 0.0 || overlay_range_smoothing_alpha_ > 1.0)
      throw std::invalid_argument(
              "overlay_range_smoothing_alpha must be in (0, 1]");

    cloud_pub_ = create_publisher<Cloud>(
      topic(*this, "output_cloud_topic", "/points2"), 1);
    image_pub_ = create_publisher<Image>(
      topic(*this, "output_image_topic", "/pcOnImage_image"), 1);

    const auto qos = rclcpp::SensorDataQoS().get_rmw_qos_profile();
    cloud_sub_.subscribe(this, topic(*this, "pcTopic", "/pc_interpoled"), qos);
    image_sub_.subscribe(this, topic(*this, "imgTopic", "/camera/color/image_raw"), qos);
    sync_ = std::make_shared<message_filters::Synchronizer<Policy>>(
      Policy(static_cast<uint32_t>(queue)), cloud_sub_, image_sub_);
    sync_->registerCallback(std::bind(
      &LidarCameraNode::callback, this, std::placeholders::_1, std::placeholders::_2));

    RCLCPP_INFO(
      get_logger(), "Waiting for synchronized interpolated LiDAR cloud and camera image");
  }

private:
  void callback(const Cloud::ConstSharedPtr & cloud, const Image::ConstSharedPtr & image)
  {
    try {
      const auto colors = cv_bridge::toCvCopy(image, "bgr8");
      if (colors->image.empty()) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "Empty camera image");
        return;
      }

      pcl::PointCloud<pcl::PointXYZ> dense_cloud;
      pcl::fromROSMsg(*cloud, dense_cloud);

      cv::Mat overlay = colors->image.clone();
      pcl::PointCloud<pcl::PointXYZRGB> colored;
      colored.reserve(dense_cloud.size());

      std::vector<ProjectedSample> projected_samples;
      projected_samples.reserve(dense_cloud.size());
      std::vector<double> visible_ranges;
      visible_ranges.reserve(dense_cloud.size());

      for (const auto & p : dense_cloud) {
        if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) continue;

        // The calibration extrinsic is interpreted directly as:
        //   P_camera = T_lidar_to_camera * [x_lidar, y_lidar, z_lidar, 1]^T
        // No implicit axis remapping is performed here.
        const Eigen::Vector4f lidar_point(p.x, p.y, p.z, 1.0f);
        const Eigen::Vector3f projected = camera_ * (transform_ * lidar_point);
        if (!projected.allFinite() || projected.z() <= 1e-6f) continue;

        const float u = projected.x() / projected.z();
        const float v = projected.y() / projected.z();
        if (!std::isfinite(u) || !std::isfinite(v) ||
            u < 0.0f || v < 0.0f || u >= overlay.cols || v >= overlay.rows)
          continue;

        const double range = std::sqrt(
          static_cast<double>(p.x) * p.x +
          static_cast<double>(p.y) * p.y +
          static_cast<double>(p.z) * p.z);
        if (!std::isfinite(range)) continue;

        projected_samples.push_back(ProjectedSample{
          p, static_cast<int>(u), static_cast<int>(v), range});
        visible_ranges.push_back(range);
      }

      if (!visible_ranges.empty()) {
        std::sort(visible_ranges.begin(), visible_ranges.end());
        double current_min = percentile_from_sorted(
          visible_ranges, overlay_percentile_low_);
        double current_max = percentile_from_sorted(
          visible_ranges, overlay_percentile_high_);

        // Avoid a degenerate color span in nearly planar/equidistant scenes.
        if (current_max - current_min < 1e-3) {
          const double center = 0.5 * (current_min + current_max);
          current_min = std::max(0.0, center - 0.5);
          current_max = center + 0.5;
        }

        if (!overlay_range_initialized_) {
          overlay_min_range_m_ = current_min;
          overlay_max_range_m_ = current_max;
          overlay_range_initialized_ = true;
        } else {
          const double a = overlay_range_smoothing_alpha_;
          overlay_min_range_m_ =
            (1.0 - a) * overlay_min_range_m_ + a * current_min;
          overlay_max_range_m_ =
            (1.0 - a) * overlay_max_range_m_ + a * current_max;
        }
      }

      for (const auto & sample : projected_samples) {
        const auto color = colors->image.at<cv::Vec3b>(sample.py, sample.px);

        pcl::PointXYZRGB point;
        point.x = sample.point.x;
        point.y = sample.point.y;
        point.z = sample.point.z;
        point.r = color[2];
        point.g = color[1];
        point.b = color[0];
        colored.push_back(point);

        cv::circle(
          overlay, cv::Point(sample.px, sample.py), 1,
          distance_color(sample.range_m, overlay_min_range_m_, overlay_max_range_m_),
          cv::FILLED);
      }

      colored.is_dense = true;
      colored.height = 1;
      colored.width = static_cast<uint32_t>(colored.size());

      Cloud output;
      pcl::toROSMsg(colored, output);
      output.header = cloud->header;
      cloud_pub_->publish(output);
      image_pub_->publish(*cv_bridge::CvImage(image->header, "bgr8", overlay).toImageMsg());

      if (dense_cloud.empty()) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 5000, "Interpolated LiDAR cloud is empty");
      } else if (projected_samples.empty()) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 5000, "No LiDAR points project inside the camera image");
      }
    } catch (const std::exception & e) {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 5000, "Fusion processing failed: %s", e.what());
    }
  }

  double overlay_percentile_low_ = 0.05;
  double overlay_percentile_high_ = 0.95;
  double overlay_range_smoothing_alpha_ = 0.2;
  bool overlay_range_initialized_ = false;
  double overlay_min_range_m_ = 0.0;
  double overlay_max_range_m_ = 1.0;

  Eigen::Matrix4f transform_;
  Eigen::Matrix<float, 3, 4> camera_;

  rclcpp::Publisher<Cloud>::SharedPtr cloud_pub_;
  rclcpp::Publisher<Image>::SharedPtr image_pub_;
  message_filters::Subscriber<Cloud> cloud_sub_;
  message_filters::Subscriber<Image> image_sub_;
  std::shared_ptr<message_filters::Synchronizer<Policy>> sync_;
};
}  // namespace lidar_camera_fusion

int main(int argc, char ** argv)
{
  return lidar_camera_fusion::run<lidar_camera_fusion::LidarCameraNode>(
    argc, argv, "lidar_camera_node");
}
