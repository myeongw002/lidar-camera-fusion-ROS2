// Fixed-size ring range-image interpolation node for ROS2 Humble.
#include "node_parameters.hpp"
#include "pointcloud_utils.hpp"

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
      const auto input = ring_points_from_ros(*msg);
      auto result = interpolate(input, settings_, Mode::Interpolation);

      sensor_msgs::msg::PointCloud2 output;
      pcl::toROSMsg(result.cloud, output);
      output.header = msg->header;
      cloud_pub_->publish(output);

      publish_range_image(
        result.raw_ranges, result.raw_rows, result.cols, msg->header, raw_range_pub_);
      publish_range_image(
        result.interpolated_ranges, result.interpolated_rows, result.cols,
        msg->header, interpolated_range_pub_);
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
