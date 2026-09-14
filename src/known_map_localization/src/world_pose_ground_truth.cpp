// Read SceneBroadcaster model world poses with the Pose_V source timestamp.
// This publishes evaluation data only; it never broadcasts ROS TF.
#include <memory>
#include <string>
#include <ignition/msgs/pose_v.pb.h>
#include <ignition/transport/Node.hh>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

class WorldPoseGroundTruth : public rclcpp::Node
{
public:
  WorldPoseGroundTruth() : Node("world_pose_ground_truth")
  {
    model_ = declare_parameter<std::string>("model", "thesis_robot");
    const auto topic = declare_parameter<std::string>(
      "ign_topic", "/world/floor1/dynamic_pose/info");
    publisher_ = create_publisher<geometry_msgs::msg::PoseStamped>(
      "/ground_truth/world_pose", rclcpp::QoS(200));
    if (!transport_.Subscribe(topic, &WorldPoseGroundTruth::onPose, this)) {
      throw std::runtime_error("Cannot subscribe to Gazebo world poses");
    }
    RCLCPP_INFO(get_logger(), "Ground truth: %s model=%s, original Pose_V stamp", topic.c_str(), model_.c_str());
  }

private:
  void onPose(const ignition::msgs::Pose_V & msg)
  {
    if (!msg.has_header() || !msg.header().has_stamp()) return;
    for (const auto & pose : msg.pose()) {
      if (pose.name() != model_) continue;
      geometry_msgs::msg::PoseStamped out;
      out.header.frame_id = "world";
      out.header.stamp.sec = msg.header().stamp().sec();
      out.header.stamp.nanosec = msg.header().stamp().nsec();
      out.pose.position.x = pose.position().x();
      out.pose.position.y = pose.position().y();
      out.pose.position.z = pose.position().z();
      out.pose.orientation.x = pose.orientation().x();
      out.pose.orientation.y = pose.orientation().y();
      out.pose.orientation.z = pose.orientation().z();
      out.pose.orientation.w = pose.orientation().w();
      publisher_->publish(out);
      return;
    }
  }
  std::string model_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr publisher_;
  ignition::transport::Node transport_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<WorldPoseGroundTruth>());
  rclcpp::shutdown();
}
