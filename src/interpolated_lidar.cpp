// Fixed-size ring range-image interpolation node for ROS2 Humble.
#include "node_parameters.hpp"
#include "pointcloud_utils.hpp"

#include <cv_bridge/cv_bridge.h>
#include <chrono>
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
    raw_range_pub_ = create_publisher<sensor_msgs::msg::Image>(
      topic(*this, "raw_range_image_topic", "/range_image_raw"), 10);
    interpolated_range_pub_ = create_publisher<sensor_msgs::msg::Image>(
      topic(*this, "interpolated_range_image_topic", "/range_image_interpolated"), 10);

    subscription_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      topic(*this, "pcTopic", "/velodyne_points"), rclcpp::SensorDataQoS(),
      std::bind(&InterpolatedLidarNode::callback, this, std::placeholders::_1));

    RCLCPP_INFO(
      get_logger(),
      "Waiting for LiDAR messages; raw range image %dx%d -> interpolated %dx%d (%.6f deg/bin)",
      settings_.input_rows, settings_.horizontal_columns(),
      settings_.output_rows, settings_.horizontal_columns(),
      settings_.horizontal_resolution_deg);
  }

private:
  void publish_range_image(
    const std::vector<float> & values, int rows, int cols,
    const std_msgs::msg::Header & header,
    const rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr & publisher)
  {
    cv::Mat image(rows, cols, CV_32FC1, const_cast<float *>(values.data()));
    publisher->publish(*cv_bridge::CvImage(header, "32FC1", image).toImageMsg());
  }

  void callback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg)
  {
    try {
      using Clock = std::chrono::steady_clock;
      const auto elapsed_ms = [](const Clock::time_point & begin, const Clock::time_point & end) {
          return std::chrono::duration<double, std::milli>(end - begin).count();
        };

      const auto callback_begin = Clock::now();

      const auto parse_begin = Clock::now();
      const auto input = ring_points_from_ros(*msg);
      const auto parse_end = Clock::now();

      auto result = interpolate(input, settings_, Mode::Interpolation);

      const auto ros_begin = Clock::now();
      sensor_msgs::msg::PointCloud2 output;
      pcl::toROSMsg(result.cloud, output);
      output.header = msg->header;
      const auto ros_end = Clock::now();

      const auto cloud_publish_begin = Clock::now();
      cloud_pub_->publish(output);
      const auto cloud_publish_end = Clock::now();

      const auto image_publish_begin = Clock::now();
      publish_range_image(
        result.raw_ranges, result.raw_rows, result.cols, msg->header, raw_range_pub_);
      publish_range_image(
        result.interpolated_ranges, result.interpolated_rows, result.cols,
        msg->header, interpolated_range_pub_);
      const auto callback_end = Clock::now();

      const double parse_ms = elapsed_ms(parse_begin, parse_end);
      const double ros_ms = elapsed_ms(ros_begin, ros_end);
      const double cloud_publish_ms = elapsed_ms(cloud_publish_begin, cloud_publish_end);
      const double image_publish_ms = elapsed_ms(image_publish_begin, callback_end);
      const double callback_total_ms = elapsed_ms(callback_begin, callback_end);

      RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Interpolation timing [ms] parse=%.3f core=%.3f "
        "(setup=%.3f raw=%.3f dense=%.3f xyz=%.3f) "
        "ros=%.3f cloud_pub=%.3f image_pub=%.3f total=%.3f "
        "points_in=%llu points_out=%llu",
        parse_ms,
        result.timing.total_ms,
        result.timing.setup_ms,
        result.timing.raw_range_ms,
        result.timing.directional_interpolation_ms,
        result.timing.xyz_reconstruction_ms,
        ros_ms,
        cloud_publish_ms,
        image_publish_ms,
        callback_total_ms,
        static_cast<unsigned long long>(input.size()),
        static_cast<unsigned long long>(result.cloud.size()));
    } catch (const std::exception & e) {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 5000, "LiDAR processing failed: %s", e.what());
    }
  }

  Settings settings_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr raw_range_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr interpolated_range_pub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
};
}  // namespace lidar_camera_fusion

int main(int argc, char ** argv)
{
  return lidar_camera_fusion::run<lidar_camera_fusion::InterpolatedLidarNode>(
    argc, argv, "interpolated_node");
}
