// Native ROS2 Humble LiDAR-camera fusion using fixed ring range images.
#include "node_parameters.hpp"
#include "pointcloud_utils.hpp"

#include <algorithm>
#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <opencv2/imgproc.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

namespace lidar_camera_fusion
{
class LidarCameraNode : public rclcpp::Node
{
  using Cloud = sensor_msgs::msg::PointCloud2;
  using Image = sensor_msgs::msg::Image;
  using Policy = message_filters::sync_policies::ApproximateTime<Cloud, Image>;

public:
  LidarCameraNode() : Node("lidar_camera_node"), settings_(read_settings(*this))
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

    cloud_pub_ = create_publisher<Cloud>(
      topic(*this, "output_cloud_topic", "/points2"), 1);
    image_pub_ = create_publisher<Image>(
      topic(*this, "output_image_topic", "/pcOnImage_image"), 1);
    raw_range_pub_ = create_publisher<Image>(
      topic(*this, "raw_range_image_topic", "/range_image_raw"), 1);
    interpolated_range_pub_ = create_publisher<Image>(
      topic(*this, "interpolated_range_image_topic", "/range_image_interpolated"), 1);

    const auto qos = rclcpp::SensorDataQoS().get_rmw_qos_profile();
    cloud_sub_.subscribe(this, topic(*this, "pcTopic", "/velodyne_points"), qos);
    image_sub_.subscribe(this, topic(*this, "imgTopic", "/camera/color/image_raw"), qos);
    sync_ = std::make_shared<message_filters::Synchronizer<Policy>>(
      Policy(static_cast<uint32_t>(queue)), cloud_sub_, image_sub_);
    sync_->registerCallback(std::bind(
      &LidarCameraNode::callback, this, std::placeholders::_1, std::placeholders::_2));

    RCLCPP_INFO(
      get_logger(),
      "Waiting for synchronized LiDAR/camera; raw range image %dx%d -> interpolated %dx%d",
      settings_.input_rows, settings_.horizontal_columns(),
      settings_.output_rows, settings_.horizontal_columns());
  }

private:
  void publish_range_image(
    const std::vector<float> & values, int rows, int cols,
    const std_msgs::msg::Header & header,
    const rclcpp::Publisher<Image>::SharedPtr & publisher)
  {
    cv::Mat image(rows, cols, CV_32FC1, const_cast<float *>(values.data()));
    publisher->publish(*cv_bridge::CvImage(header, "32FC1", image).toImageMsg());
  }

  void callback(const Cloud::ConstSharedPtr & cloud, const Image::ConstSharedPtr & image)
  {
    try {
      const auto colors = cv_bridge::toCvCopy(image, "bgr8");
      if (colors->image.empty()) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "Empty camera image");
        return;
      }

      cv::Mat overlay = colors->image.clone();
      const auto input = ring_points_from_ros(*cloud);
      const auto result = interpolate(input, settings_, Mode::Fusion);

      publish_range_image(
        result.raw_ranges, result.raw_rows, result.cols, cloud->header, raw_range_pub_);
      publish_range_image(
        result.interpolated_ranges, result.interpolated_rows, result.cols,
        cloud->header, interpolated_range_pub_);

      pcl::PointCloud<pcl::PointXYZRGB> colored;
      colored.reserve(result.cloud.size());

      for (const auto & p : result.cloud) {
        const Eigen::Vector4f remapped(-p.y, -p.z, p.x, 1.0f);
        const Eigen::Vector3f projected = camera_ * (transform_ * remapped);
        if (!projected.allFinite() || projected.z() <= 1e-6f) continue;

        const float u = projected.x() / projected.z();
        const float v = projected.y() / projected.z();
        if (!std::isfinite(u) || !std::isfinite(v) ||
            u < 0.0f || v < 0.0f || u >= overlay.cols || v >= overlay.rows)
          continue;

        const int px = static_cast<int>(u);
        const int py = static_cast<int>(v);
        const auto color = colors->image.at<cv::Vec3b>(py, px);

        pcl::PointXYZRGB point;
        point.x = p.x;
        point.y = p.y;
        point.z = p.z;
        point.r = color[2];
        point.g = color[1];
        point.b = color[0];
        colored.push_back(point);

        const int distance_x = static_cast<int>(std::clamp(
          255.0 * p.x / settings_.maxlen, 0.0, 255.0));
        const int distance_z = static_cast<int>(std::clamp(
          255.0 * p.x / 10.0, 0.0, 255.0));
        cv::circle(
          overlay, cv::Point(px, py), 1,
          cv::Scalar(distance_x, distance_z, 255 - distance_x), cv::FILLED);
      }

      colored.is_dense = true;
      colored.height = 1;
      colored.width = static_cast<uint32_t>(colored.size());

      Cloud output;
      pcl::toROSMsg(colored, output);
      output.header = cloud->header;
      cloud_pub_->publish(output);
      image_pub_->publish(*cv_bridge::CvImage(image->header, "bgr8", overlay).toImageMsg());

      if (result.cloud.empty())
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 5000, "No usable LiDAR points in camera FOV");
    } catch (const std::exception & e) {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 5000, "Fusion processing failed: %s", e.what());
    }
  }

  Settings settings_;
  Eigen::Matrix4f transform_;
  Eigen::Matrix<float, 3, 4> camera_;

  rclcpp::Publisher<Cloud>::SharedPtr cloud_pub_;
  rclcpp::Publisher<Image>::SharedPtr image_pub_;
  rclcpp::Publisher<Image>::SharedPtr raw_range_pub_;
  rclcpp::Publisher<Image>::SharedPtr interpolated_range_pub_;
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
