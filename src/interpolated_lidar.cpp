// Native ROS2 Humble port of EPVelasco/lidar-camera-fusion.
#include "node_parameters.hpp"
#include <cv_bridge/cv_bridge.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

namespace lidar_camera_fusion
{
class InterpolatedLidarNode : public rclcpp::Node
{
public:
  InterpolatedLidarNode() : Node("interpolated_node"), settings_(read_settings(*this))
  {
    cloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      topic(*this, "output_cloud_topic", "/pc_interpoled"), 10);
    image_pub_ = create_publisher<sensor_msgs::msg::Image>(
      topic(*this, "output_image_topic", "/pc2imageInterpol"), 10);
    subscription_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      topic(*this, "pcTopic", "/velodyne_points"), rclcpp::SensorDataQoS(),
      std::bind(&InterpolatedLidarNode::callback, this, std::placeholders::_1));
    RCLCPP_INFO(get_logger(), "Waiting for LiDAR messages");
  }
private:
  void callback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg)
  {
    try {
      pcl::PointCloud<pcl::PointXYZ> input;
      pcl::fromROSMsg(*msg, input);
      auto result = interpolate(input, range_image_, settings_, Mode::Interpolation);
      sensor_msgs::msg::PointCloud2 output;
      pcl::toROSMsg(result.cloud, output);
      output.header = msg->header;
      cloud_pub_->publish(output);
      if (result.ranges.is_empty()) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "No usable interpolated LiDAR points");
        return;
      }
      cv::Mat image(result.ranges.n_rows, result.ranges.n_cols, CV_16UC1);
      for (int i = 0; i < image.rows; ++i) {
        for (int j = 0; j < image.cols; ++j) {
          // Preserve the legacy range visualization's truncation/wrapping, but
          // perform it explicitly: negative float -> ushort was undefined.
          const double value = 1.0 - (65536.0 / (settings_.maxlen - settings_.minlen)) *
            (result.ranges(i, j) - settings_.minlen);
          double wrapped = std::isfinite(value) ? std::fmod(std::trunc(value), 65536.0) : 0;
          if (wrapped < 0) wrapped += 65536.0;
          image.at<uint16_t>(i, j) = static_cast<uint16_t>(wrapped);
        }
      }
      image_pub_->publish(*cv_bridge::CvImage(msg->header, "mono16", image).toImageMsg());
    } catch (const std::exception & e) {
      RCLCPP_ERROR(get_logger(), "LiDAR processing failed: %s", e.what());
    }
  }
  Settings settings_;
  pcl::RangeImageSpherical range_image_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
};
}  // namespace lidar_camera_fusion
int main(int argc, char ** argv)
{
  return lidar_camera_fusion::run<lidar_camera_fusion::InterpolatedLidarNode>(
    argc, argv, "interpolated_node");
}
