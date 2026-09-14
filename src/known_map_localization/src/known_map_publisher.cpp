#include <chrono>
#include <memory>
#include <string>
#include <stdexcept>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

using namespace std::chrono_literals;

class KnownMapPublisher : public rclcpp::Node
{
public:
    KnownMapPublisher()
    : Node("known_map_publisher")
    {
        this->declare_parameter<std::string>(
            "map_path",
            "/home/yez/lio_sam_maps/floor1/GlobalMap.pcd");

        map_path_ = this->get_parameter("map_path").as_string();

        RCLCPP_INFO(
            this->get_logger(),
            "Loading known map: %s",
            map_path_.c_str());

        map_cloud_.reset(
    		new pcl::PointCloud<pcl::PointXYZI>());

        if (pcl::io::loadPCDFile<pcl::PointXYZI>(
                map_path_, *map_cloud_) == -1)
        {
            RCLCPP_FATAL(
                this->get_logger(),
                "Failed to load PCD map: %s",
                map_path_.c_str());

            throw std::runtime_error("Failed to load known map");
        }

        RCLCPP_INFO(
            this->get_logger(),
            "Known map loaded successfully.");

        RCLCPP_INFO(
            this->get_logger(),
            "Map points: %zu",
            map_cloud_->size());

        pcl::toROSMsg(*map_cloud_, map_msg_);

        map_msg_.header.frame_id = "map";

        map_pub_ =
            this->create_publisher<sensor_msgs::msg::PointCloud2>(
                "/known_map",
                1);

        timer_ = this->create_wall_timer(
            1s,
            std::bind(
                &KnownMapPublisher::publishMap,
                this));
    }

private:
    void publishMap()
    {
        map_msg_.header.stamp = this->now();
        map_pub_->publish(map_msg_);
    }

    std::string map_path_;

    pcl::PointCloud<pcl::PointXYZI>::Ptr map_cloud_;

    sensor_msgs::msg::PointCloud2 map_msg_;

    rclcpp::Publisher<
        sensor_msgs::msg::PointCloud2>::SharedPtr map_pub_;

    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);

    try
    {
        rclcpp::spin(
            std::make_shared<KnownMapPublisher>());
    }
    catch (const std::exception & e)
    {
        RCLCPP_FATAL(
            rclcpp::get_logger("known_map_publisher"),
            "%s",
            e.what());
    }

    rclcpp::shutdown();

    return 0;
}
