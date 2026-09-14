#include <algorithm>
#include <cmath>
#include <cstdint>
#include <deque>
#include <limits>
#include <memory>
#include <string>
#include <fstream>
#include <iomanip>
#include <stdexcept>

#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_msgs/msg/tf_message.hpp>

class LocalizationEvaluator : public rclcpp::Node
{
public:
    LocalizationEvaluator()
    : Node("localization_evaluator")
    {
        this->declare_parameter<std::string>(
            "gt_parent_frame",
            "thesis_robot/odom");

        this->declare_parameter<std::string>(
            "gt_child_frame",
            "thesis_robot/chassis");

        this->declare_parameter<double>(
            "max_time_difference",
            0.10);

        gt_parent_frame_ =
            this->get_parameter("gt_parent_frame").as_string();

        gt_child_frame_ =
            this->get_parameter("gt_child_frame").as_string();

        max_time_difference_ =
            this->get_parameter("max_time_difference").as_double();

        const auto csv_path = declare_parameter<std::string>("csv_path", "");
        log_every_n_ = declare_parameter<int>("log_every_n", 10);
        const auto world_topic = declare_parameter<std::string>("world_pose_topic", "");
        if (!csv_path.empty()) {
            // Do not silently overwrite a previous experiment.
            if (std::ifstream(csv_path).good()) throw std::runtime_error("CSV already exists: " + csv_path);
            csv_.open(csv_path);
            if (!csv_) throw std::runtime_error("Cannot open CSV: " + csv_path);
            csv_ << "timestamp,gt_timestamp,gt_x,gt_y,gt_yaw,ndt_x,ndt_y,ndt_yaw,ex,ey,xy_error,yaw_error,time_difference\n";
            csv_ << std::setprecision(17);
        }

        if (world_topic.empty()) {
        gt_sub_ =
            this->create_subscription<tf2_msgs::msg::TFMessage>(
                "/ground_truth/tf",
                100,
                std::bind(
                    &LocalizationEvaluator::gtCallback,
                    this,
                    std::placeholders::_1));
        } else {
            world_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
                world_topic, 200,
                [this](geometry_msgs::msg::PoseStamped::ConstSharedPtr msg) {
                    if (msg->header.frame_id != "world") return;
                    GroundTruthSample sample;
                    sample.stamp_ns = stampToNs(msg->header.stamp);
                    sample.x = msg->pose.position.x;
                    sample.y = msg->pose.position.y;
                    const auto & q = msg->pose.orientation;
                    sample.yaw = quaternionToYaw(q.x, q.y, q.z, q.w);
                    gt_buffer_.push_back(sample);
                    while (gt_buffer_.size() > 3000) gt_buffer_.pop_front();
                });
            RCLCPP_INFO(get_logger(), "Using simulator model world pose: %s", world_topic.c_str());
        }

        localization_sub_ =
            this->create_subscription<geometry_msgs::msg::PoseStamped>(
                "/localization/pose",
                10,
                std::bind(
                    &LocalizationEvaluator::localizationCallback,
                    this,
                    std::placeholders::_1));

        RCLCPP_INFO(
            this->get_logger(),
            "Localization evaluator ready.");

        RCLCPP_INFO(
            this->get_logger(),
            "GT: %s -> %s",
            gt_parent_frame_.c_str(),
            gt_child_frame_.c_str());

        RCLCPP_INFO(
            this->get_logger(),
            "Comparing /ground_truth/tf with /localization/pose");

        RCLCPP_INFO(
            this->get_logger(),
            "Evaluation: x, y, yaw");
    }

private:

    struct GroundTruthSample
    {
        int64_t stamp_ns;

        double x;
        double y;
        double yaw;
    };

    static double quaternionToYaw(
        double x,
        double y,
        double z,
        double w)
    {
        const double siny_cosp =
            2.0 * (w * z + x * y);

        const double cosy_cosp =
            1.0 - 2.0 * (y * y + z * z);

        return std::atan2(
            siny_cosp,
            cosy_cosp);
    }

    static double normalizeAngle(double angle)
    {
        while (angle > M_PI)
            angle -= 2.0 * M_PI;

        while (angle < -M_PI)
            angle += 2.0 * M_PI;

        return angle;
    }

    static int64_t stampToNs(
        const builtin_interfaces::msg::Time & stamp)
    {
        return
            static_cast<int64_t>(stamp.sec) *
            1000000000LL +
            static_cast<int64_t>(stamp.nanosec);
    }

    void gtCallback(
        const tf2_msgs::msg::TFMessage::SharedPtr msg)
    {
        for (const auto & tf : msg->transforms)
        {
            if (tf.header.frame_id != gt_parent_frame_)
                continue;

            if (tf.child_frame_id != gt_child_frame_)
                continue;

            GroundTruthSample sample;

            sample.stamp_ns =
                stampToNs(tf.header.stamp);

            sample.x =
                tf.transform.translation.x;

            sample.y =
                tf.transform.translation.y;

            sample.yaw =
                quaternionToYaw(
                    tf.transform.rotation.x,
                    tf.transform.rotation.y,
                    tf.transform.rotation.z,
                    tf.transform.rotation.w);

            gt_buffer_.push_back(sample);

            // Ground truth is high frequency.
            // Keep only recent samples.
            while (gt_buffer_.size() > 1000)
            {
                gt_buffer_.pop_front();
            }
        }
    }

    void localizationCallback(
        const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        if (msg->header.frame_id != "map") return;
        if (gt_buffer_.empty())
        {
            RCLCPP_WARN_THROTTLE(
                this->get_logger(),
                *this->get_clock(),
                2000,
                "Waiting for Ground Truth...");
            return;
        }

        const int64_t loc_stamp_ns =
            stampToNs(msg->header.stamp);

        // ============================================================
        // Find nearest Ground Truth sample in time
        // ============================================================
        const GroundTruthSample * best_gt = nullptr;

        int64_t best_dt_ns =
            std::numeric_limits<int64_t>::max();

        for (const auto & gt : gt_buffer_)
        {
            const int64_t dt_ns =
                std::llabs(
                    gt.stamp_ns -
                    loc_stamp_ns);

            if (dt_ns < best_dt_ns)
            {
                best_dt_ns = dt_ns;
                best_gt = &gt;
            }
        }

        if (best_gt == nullptr)
            return;

        const double dt =
            static_cast<double>(best_dt_ns) /
            1e9;

        if (dt > max_time_difference_)
        {
            RCLCPP_WARN_THROTTLE(
                this->get_logger(),
                *this->get_clock(),
                2000,
                "No synchronized GT sample. "
                "Nearest dt=%.3f s",
                dt);
            return;
        }

        // ============================================================
        // NDT pose
        // ============================================================
        const double ndt_x =
            msg->pose.position.x;

        const double ndt_y =
            msg->pose.position.y;

        const double ndt_yaw =
            quaternionToYaw(
                msg->pose.orientation.x,
                msg->pose.orientation.y,
                msg->pose.orientation.z,
                msg->pose.orientation.w);

        // ============================================================
        // Errors
        //
        // Current experiment assumes:
        // map XY/yaw axes were initialized consistently with
        // Gazebo thesis_robot/odom during mapping.
        // ============================================================
        const double ex =
            ndt_x - best_gt->x;

        const double ey =
            ndt_y - best_gt->y;

        const double xy_error =
            std::sqrt(
                ex * ex +
                ey * ey);

        const double yaw_error =
            normalizeAngle(
                ndt_yaw -
                best_gt->yaw);

        const double abs_yaw_error =
            std::fabs(yaw_error);

        // Every matched sample is persisted, independent of console throttling.
        // yaw values/errors are radians; time_difference is absolute seconds.
        if (csv_.is_open()) {
            csv_ << loc_stamp_ns / 1e9 << ',' << best_gt->stamp_ns / 1e9 << ','
                 << best_gt->x << ',' << best_gt->y << ',' << best_gt->yaw << ','
                 << ndt_x << ',' << ndt_y << ',' << ndt_yaw << ','
                 << ex << ',' << ey << ',' << xy_error << ',' << yaw_error << ',' << dt << '\n';
            csv_.flush();
        }

        // ============================================================
        // Running statistics
        // ============================================================
        count_++;

        sum_xy_error_ += xy_error;

        sum_xy_error_squared_ +=
            xy_error * xy_error;

        max_xy_error_ =
            std::max(
                max_xy_error_,
                xy_error);

        sum_abs_yaw_error_ +=
            abs_yaw_error;

        sum_yaw_error_squared_ +=
            yaw_error * yaw_error;

        max_abs_yaw_error_ =
            std::max(
                max_abs_yaw_error_,
                abs_yaw_error);

        const double mean_xy =
            sum_xy_error_ /
            static_cast<double>(count_);

        const double rmse_xy =
            std::sqrt(
                sum_xy_error_squared_ /
                static_cast<double>(count_));

        const double mean_yaw_deg =
            (sum_abs_yaw_error_ /
             static_cast<double>(count_)) *
            180.0 / M_PI;

        const double rmse_yaw_deg =
            std::sqrt(
                sum_yaw_error_squared_ /
                static_cast<double>(count_)) *
            180.0 / M_PI;

        const double yaw_error_deg =
            yaw_error *
            180.0 / M_PI;

        const double max_yaw_deg =
            max_abs_yaw_error_ *
            180.0 / M_PI;

        // Every 10 localization samples
        if (log_every_n_ > 0 && count_ % static_cast<size_t>(log_every_n_) == 0)
        {
            RCLCPP_INFO(
                this->get_logger(),
                "EVAL #%zu | dt=%.3f s | "
                "GT=(%.3f %.3f) | "
                "NDT=(%.3f %.3f) | "
                "ex=%.3f ey=%.3f | "
                "XY=%.3f m | yaw_err=%.3f deg | "
                "RMSE_XY=%.3f m | mean_XY=%.3f m | "
                "max_XY=%.3f m | "
                "RMSE_yaw=%.3f deg | "
                "mean_yaw=%.3f deg | "
                "max_yaw=%.3f deg",
                count_,
                dt,
                best_gt->x,
                best_gt->y,
                ndt_x,
                ndt_y,
                ex,
                ey,
                xy_error,
                yaw_error_deg,
                rmse_xy,
                mean_xy,
                max_xy_error_,
                rmse_yaw_deg,
                mean_yaw_deg,
                max_yaw_deg);
        }
    }

    std::string gt_parent_frame_;
    std::ofstream csv_;
    int log_every_n_{10};
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr world_sub_;
    std::string gt_child_frame_;

    double max_time_difference_;

    std::deque<GroundTruthSample>
        gt_buffer_;

    rclcpp::Subscription<
        tf2_msgs::msg::TFMessage>::SharedPtr
        gt_sub_;

    rclcpp::Subscription<
        geometry_msgs::msg::PoseStamped>::SharedPtr
        localization_sub_;

    size_t count_{0};

    double sum_xy_error_{0.0};
    double sum_xy_error_squared_{0.0};
    double max_xy_error_{0.0};

    double sum_abs_yaw_error_{0.0};
    double sum_yaw_error_squared_{0.0};
    double max_abs_yaw_error_{0.0};
};

int main(
    int argc,
    char ** argv)
{
    rclcpp::init(argc, argv);

    rclcpp::spin(
        std::make_shared<
            LocalizationEvaluator>());

    rclcpp::shutdown();

    return 0;
}
