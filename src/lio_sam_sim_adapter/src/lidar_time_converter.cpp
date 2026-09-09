#include <cmath>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/point_field.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"

class LidarTimeConverter : public rclcpp::Node
{
public:
  LidarTimeConverter()
  : Node("lidar_time_converter")
  {
    scan_period_ = this->declare_parameter<double>("scan_period", 0.1);

    sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "/lidar/points",
      rclcpp::SensorDataQoS(),
      std::bind(
        &LidarTimeConverter::cloudCallback,
        this,
        std::placeholders::_1));

    pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
      "/lio_sam/points",
      rclcpp::SensorDataQoS());

    RCLCPP_INFO(
      this->get_logger(),
      "LiDAR time converter started. scan_period = %.3f s",
      scan_period_);
  }

private:
  void cloudCallback(
    const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    sensor_msgs::msg::PointCloud2 out;

    out.header = msg->header;
    out.header.frame_id = "lidar_link";
    out.height = msg->height;
    out.width = msg->width;
    out.is_bigendian = msg->is_bigendian;

    const size_t lidar_width = msg->width;
    out.is_dense = msg->is_dense;

    sensor_msgs::PointCloud2Modifier modifier(out);

    modifier.setPointCloud2Fields(
      6,
      "x", 1, sensor_msgs::msg::PointField::FLOAT32,
      "y", 1, sensor_msgs::msg::PointField::FLOAT32,
      "z", 1, sensor_msgs::msg::PointField::FLOAT32,
      "intensity", 1, sensor_msgs::msg::PointField::FLOAT32,
      "ring", 1, sensor_msgs::msg::PointField::UINT16,
      "time", 1, sensor_msgs::msg::PointField::FLOAT32);

    modifier.resize(
      static_cast<size_t>(msg->width) *
      static_cast<size_t>(msg->height));

    sensor_msgs::PointCloud2ConstIterator<float> in_x(*msg, "x");
    sensor_msgs::PointCloud2ConstIterator<float> in_y(*msg, "y");
    sensor_msgs::PointCloud2ConstIterator<float> in_z(*msg, "z");
    sensor_msgs::PointCloud2ConstIterator<float> in_intensity(*msg, "intensity");
    sensor_msgs::PointCloud2ConstIterator<uint16_t> in_ring(*msg, "ring");

    sensor_msgs::PointCloud2Iterator<float> out_x(out, "x");
    sensor_msgs::PointCloud2Iterator<float> out_y(out, "y");
    sensor_msgs::PointCloud2Iterator<float> out_z(out, "z");
    sensor_msgs::PointCloud2Iterator<float> out_intensity(out, "intensity");
    sensor_msgs::PointCloud2Iterator<uint16_t> out_ring(out, "ring");
    sensor_msgs::PointCloud2Iterator<float> out_time(out, "time");

    const size_t count =
      static_cast<size_t>(msg->width) *
      static_cast<size_t>(msg->height);

    for (size_t i = 0; i < count; ++i,
         ++in_x, ++in_y, ++in_z, ++in_intensity, ++in_ring,
         ++out_x, ++out_y, ++out_z,
         ++out_intensity, ++out_ring, ++out_time)
    {
      *out_x = *in_x;
      *out_y = *in_y;
      *out_z = *in_z;
      *out_intensity = *in_intensity;
      *out_ring = static_cast<uint16_t>(i / lidar_width);

      const size_t col = i % static_cast<size_t>(msg->width);

      *out_time = static_cast<float>(
        static_cast<double>(col) /
        static_cast<double>(lidar_width) *
        scan_period_);
    }

    pub_->publish(out);
  }

  double scan_period_;

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LidarTimeConverter>());
  rclcpp::shutdown();

  return 0;
}
