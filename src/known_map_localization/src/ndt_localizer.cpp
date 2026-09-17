#include <chrono>
#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstdint>
#include <deque>
#include <functional>
#include <limits>
#include <memory>
#include <mutex>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <diagnostic_msgs/msg/key_value.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/float32.hpp>

#include <tf2_ros/transform_broadcaster.h>

#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/registration/ndt.h>
#include <pcl_conversions/pcl_conversions.h>

#include <Eigen/Core>
#include <Eigen/Geometry>

class NDTLocalizer : public rclcpp::Node
{
public:
    using PointT = pcl::PointXYZI;
    using PointCloudT = pcl::PointCloud<PointT>;

    NDTLocalizer()
    : Node("ndt_localizer")
    {
        // ============================================================
        // Parameters
        // ============================================================
        this->declare_parameter<std::string>(
            "map_path",
            "/home/yez/lio_sam_maps/floor1/GlobalMap.pcd");

        this->declare_parameter<double>("map_voxel_size", 0.30);
        this->declare_parameter<double>("scan_voxel_size", 0.50);

        this->declare_parameter<double>("ndt_resolution", 1.0);
        this->declare_parameter<double>("ndt_step_size", 0.1);
        this->declare_parameter<double>("ndt_epsilon", 0.01);
        this->declare_parameter<int>("ndt_max_iterations", 15);

        // Initial guess
        this->declare_parameter<double>("initial_x", 0.0);
        this->declare_parameter<double>("initial_y", 0.0);
        this->declare_parameter<double>("initial_z", 0.0);
        this->declare_parameter<double>("initial_yaw", 0.0);

        // true: 启动后先等待 RViz /initialpose
        this->declare_parameter<bool>("wait_for_initialpose", false);

        // base_link -> lidar_link
        this->declare_parameter<double>("base_to_lidar_x", 0.0);
        this->declare_parameter<double>("base_to_lidar_y", 0.0);
        this->declare_parameter<double>("base_to_lidar_z", 0.14);

        // v0.3.5 robust tracking. The fitness threshold is derived from the
        // recorded 2026-09-10 baseline experiment; all limits remain tunable.
        fitness_accept_threshold_ =
            this->declare_parameter<double>("fitness_accept_threshold", 0.017);
        max_position_jump_ =
            this->declare_parameter<double>("max_position_jump", 0.15);
        max_yaw_jump_ =
            this->declare_parameter<double>("max_yaw_jump", 0.20);
        max_prediction_position_error_ = this->declare_parameter<double>(
            "max_prediction_position_error", 0.15);
        max_prediction_yaw_error_ = this->declare_parameter<double>(
            "max_prediction_yaw_error", 0.15);
        use_odom_prior_ =
            this->declare_parameter<bool>("use_odom_prior", true);
        odom_topic_ =
            this->declare_parameter<std::string>("odom_topic", "/odom");
        max_odom_time_difference_ = this->declare_parameter<double>(
            "max_odom_time_difference", 0.10);

        if (!(fitness_accept_threshold_ > 0.0) ||
            !(max_position_jump_ > 0.0) ||
            !(max_yaw_jump_ > 0.0) ||
            !(max_prediction_position_error_ > 0.0) ||
            !(max_prediction_yaw_error_ > 0.0) ||
            !(max_odom_time_difference_ > 0.0))
        {
            throw std::runtime_error("NDT quality-gate limits must be positive");
        }

        map_path_ =
            this->get_parameter("map_path").as_string();

        map_voxel_size_ =
            this->get_parameter("map_voxel_size").as_double();

        scan_voxel_size_ =
            this->get_parameter("scan_voxel_size").as_double();

        // ============================================================
        // LiDAR extrinsic
        // ============================================================
        T_base_lidar_ = Eigen::Matrix4f::Identity();

        T_base_lidar_(0, 3) =
            static_cast<float>(
                this->get_parameter("base_to_lidar_x").as_double());

        T_base_lidar_(1, 3) =
            static_cast<float>(
                this->get_parameter("base_to_lidar_y").as_double());

        T_base_lidar_(2, 3) =
            static_cast<float>(
                this->get_parameter("base_to_lidar_z").as_double());

        T_lidar_base_ = T_base_lidar_.inverse();

        RCLCPP_INFO(
            this->get_logger(),
            "Extrinsic base_link -> lidar_link: "
            "x=%.3f y=%.3f z=%.3f",
            T_base_lidar_(0, 3),
            T_base_lidar_(1, 3),
            T_base_lidar_(2, 3));

        // ============================================================
        // Load map
        // ============================================================
        PointCloudT::Ptr map_raw(new PointCloudT);
        map_cloud_.reset(new PointCloudT);

        RCLCPP_INFO(
            this->get_logger(),
            "Loading NDT target map: %s",
            map_path_.c_str());

        if (pcl::io::loadPCDFile<PointT>(
                map_path_, *map_raw) == -1)
        {
            RCLCPP_FATAL(
                this->get_logger(),
                "Failed to load map: %s",
                map_path_.c_str());

            throw std::runtime_error(
                "Failed to load NDT target map");
        }

        std::vector<int> indices;

        pcl::removeNaNFromPointCloud(
            *map_raw,
            *map_raw,
            indices);

        RCLCPP_INFO(
            this->get_logger(),
            "Raw map points: %zu",
            map_raw->size());

        // ============================================================
        // Downsample map
        // ============================================================
        pcl::VoxelGrid<PointT> map_voxel;

        map_voxel.setLeafSize(
            static_cast<float>(map_voxel_size_),
            static_cast<float>(map_voxel_size_),
            static_cast<float>(map_voxel_size_));

        map_voxel.setInputCloud(map_raw);
        map_voxel.filter(*map_cloud_);

        RCLCPP_INFO(
            this->get_logger(),
            "Downsampled map points: %zu",
            map_cloud_->size());

        // ============================================================
        // Configure NDT
        // ============================================================
        ndt_.setTransformationEpsilon(
            this->get_parameter("ndt_epsilon").as_double());

        ndt_.setStepSize(
            this->get_parameter("ndt_step_size").as_double());

        ndt_.setResolution(
            this->get_parameter("ndt_resolution").as_double());

        ndt_.setMaximumIterations(
            static_cast<int>(
                this->get_parameter(
                    "ndt_max_iterations").as_int()));

        ndt_.setInputTarget(map_cloud_);

        // ============================================================
        // Initial guess for T_map_lidar
        // ============================================================
        const float initial_x =
            static_cast<float>(
                this->get_parameter("initial_x").as_double());

        const float initial_y =
            static_cast<float>(
                this->get_parameter("initial_y").as_double());

        const float initial_z =
            static_cast<float>(
                this->get_parameter("initial_z").as_double());

        const float initial_yaw =
            static_cast<float>(
                this->get_parameter("initial_yaw").as_double());

        current_lidar_pose_ =
            Eigen::Matrix4f::Identity();

        Eigen::AngleAxisf yaw_rotation(
            initial_yaw,
            Eigen::Vector3f::UnitZ());

        current_lidar_pose_.block<3, 3>(0, 0) =
            yaw_rotation.toRotationMatrix();

        current_lidar_pose_(0, 3) = initial_x;
        current_lidar_pose_(1, 3) = initial_y;
        current_lidar_pose_(2, 3) = initial_z;

        last_valid_lidar_pose_ = current_lidar_pose_;
        last_valid_base_pose_ = current_lidar_pose_ * T_lidar_base_;
        have_last_valid_pose_ = true;

        RCLCPP_INFO(
            this->get_logger(),
            "Initial NDT guess: "
            "x=%.3f y=%.3f z=%.3f yaw=%.3f rad",
            initial_x,
            initial_y,
            initial_z,
            initial_yaw);

        wait_for_initialpose_ =
            this->get_parameter("wait_for_initialpose").as_bool();

        initial_pose_received_ =
            !wait_for_initialpose_;

        // ============================================================
        // Publishers
        // ============================================================
        aligned_pub_ =
            this->create_publisher<
                sensor_msgs::msg::PointCloud2>(
                "/localization/aligned_scan",
                10);

        base_pose_pub_ =
            this->create_publisher<
                geometry_msgs::msg::PoseStamped>(
                "/localization/pose",
                10);

        lidar_pose_pub_ =
            this->create_publisher<
                geometry_msgs::msg::PoseStamped>(
                "/localization/lidar_pose",
                10);

        fitness_pub_ =
            this->create_publisher<
                std_msgs::msg::Float32>(
                "/localization/fitness_score",
                10);

        raw_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
            "/localization/raw_pose", 10);
        predicted_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
            "/localization/predicted_pose", 10);
        status_pub_ = this->create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
            "/localization/status", 10);

        tf_broadcaster_ =
            std::make_unique<
                tf2_ros::TransformBroadcaster>(*this);

        // ============================================================
        // LiDAR subscriber
        // ============================================================
        auto scan_qos = rclcpp::SensorDataQoS();
        scan_qos.keep_last(1);

        scan_callback_group_ = create_callback_group(
            rclcpp::CallbackGroupType::MutuallyExclusive);

        rclcpp::SubscriptionOptions scan_options;
        scan_options.callback_group = scan_callback_group_;

        scan_sub_ =
            this->create_subscription<
                sensor_msgs::msg::PointCloud2>(
                "/lio_sam/points",
                scan_qos,
                std::bind(
                    &NDTLocalizer::scanCallback,
                    this,
                    std::placeholders::_1),
                scan_options);

        // RViz "2D Pose Estimate" publishes /initialpose
        initial_pose_sub_ =
            this->create_subscription<
                geometry_msgs::msg::PoseWithCovarianceStamped>(
                "/initialpose",
                10,
                std::bind(
                    &NDTLocalizer::initialPoseCallback,
                    this,
                    std::placeholders::_1));

        odom_callback_group_ = create_callback_group(
            rclcpp::CallbackGroupType::Reentrant);
        rclcpp::SubscriptionOptions odom_options;
        odom_options.callback_group = odom_callback_group_;
        auto odom_qos = rclcpp::SensorDataQoS();
        odom_qos.keep_last(200);
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            odom_topic_, odom_qos,
            std::bind(&NDTLocalizer::odomCallback, this, std::placeholders::_1),
            odom_options);

        RCLCPP_INFO(
            this->get_logger(),
            "NDT localizer ready.");

        RCLCPP_INFO(
            this->get_logger(),
            "/localization/lidar_pose = T_map_lidar");

        RCLCPP_INFO(
            this->get_logger(),
            "/localization/pose = T_map_base_link");

        RCLCPP_INFO(
            this->get_logger(),
            "Publishing dynamic TF: map -> odom");

        RCLCPP_INFO(
            this->get_logger(),
            "Robust tracking: fitness<%.6f position_jump<%.3f m "
            "yaw_jump<%.3f rad odom_prior=%s topic=%s",
            fitness_accept_threshold_, max_position_jump_, max_yaw_jump_,
            use_odom_prior_ ? "true" : "false", odom_topic_.c_str());

        // ============================================================
        // Runtime NDT map switching
        // ============================================================
        parameter_callback_handle_ =
            this->add_on_set_parameters_callback(
                [this](const std::vector<rclcpp::Parameter> & parameters)
                {
                    return this->onParametersSet(parameters);
                });

        RCLCPP_INFO(
            this->get_logger(),
            "Runtime map switching enabled through parameter 'map_path'.");

        if (wait_for_initialpose_)
        {
            RCLCPP_INFO(
                this->get_logger(),
                "Waiting for /initialpose before NDT starts...");
        }
        else
        {
            RCLCPP_INFO(
                this->get_logger(),
                "Waiting for /lio_sam/points ...");
        }
    }

private:

    bool reloadNdtMap(
        const std::string & new_map_path,
        std::string & reason)
    {
        RCLCPP_INFO(
            this->get_logger(),
            "Reloading NDT target map: %s",
            new_map_path.c_str());

        // 先加载到临时点云。
        // 只有整个过程成功后才替换当前地图，
        // 防止坏文件破坏正在工作的 NDT target。
        PointCloudT::Ptr map_raw(new PointCloudT);

        if (pcl::io::loadPCDFile<PointT>(
                new_map_path,
                *map_raw) == -1)
        {
            reason =
                "Failed to load PCD map: " +
                new_map_path;

            RCLCPP_ERROR(
                this->get_logger(),
                "%s",
                reason.c_str());

            return false;
        }

        std::vector<int> indices;

        pcl::removeNaNFromPointCloud(
            *map_raw,
            *map_raw,
            indices);

        if (map_raw->empty())
        {
            reason = "Loaded PCD map is empty";

            RCLCPP_ERROR(
                this->get_logger(),
                "%s",
                reason.c_str());

            return false;
        }

        PointCloudT::Ptr new_map_cloud(
            new PointCloudT);

        pcl::VoxelGrid<PointT> map_voxel;

        map_voxel.setLeafSize(
            static_cast<float>(map_voxel_size_),
            static_cast<float>(map_voxel_size_),
            static_cast<float>(map_voxel_size_));

        map_voxel.setInputCloud(map_raw);
        map_voxel.filter(*new_map_cloud);

        if (new_map_cloud->empty())
        {
            reason =
                "Downsampled NDT map is empty";

            RCLCPP_ERROR(
                this->get_logger(),
                "%s",
                reason.c_str());

            return false;
        }

        RCLCPP_INFO(
            this->get_logger(),
            "New raw map points: %zu",
            map_raw->size());

        RCLCPP_INFO(
            this->get_logger(),
            "New downsampled map points: %zu",
            new_map_cloud->size());

        // ------------------------------------------------------------
        // 真正切换 NDT target
        // ------------------------------------------------------------
        // Wait for any currently running scanCallback to finish.
        std::lock_guard<std::mutex> state_lock(localization_state_mutex_);

        {
            std::lock_guard<std::mutex> ndt_lock(ndt_mutex_);
            ndt_.setInputTarget(new_map_cloud);
        }

        map_cloud_ = new_map_cloud;
        map_path_ = new_map_path;

        // ------------------------------------------------------------
        // 清除上一楼层定位历史
        // ------------------------------------------------------------
        current_lidar_pose_ =
            Eigen::Matrix4f::Identity();

        last_valid_lidar_pose_ =
            Eigen::Matrix4f::Identity();

        last_valid_base_pose_ =
            Eigen::Matrix4f::Identity();

        have_last_valid_pose_ = false;
        have_last_accept_odom_ = false;

        // 清除旧 odometry prior 缓存。
        // odomCallback 会马上重新积累当前楼层/当前时刻的数据。
        {
            std::lock_guard<std::mutex> lock(
                odom_mutex_);

            odom_buffer_.clear();
        }

        // 清除上一层 map -> odom 修正，
        // 防止新楼层初始化前继续广播旧楼层全局修正。
        {
            std::lock_guard<std::mutex> lock(
                map_odom_tf_mutex_);

            latest_T_map_odom_ =
                Eigen::Matrix4f::Identity();

            have_map_odom_tf_ = false;
        }

        accepted_count_ = 0;
        rejected_count_ = 0;

        // 关键：
        // 换楼层后必须重新获得该楼层初始位姿，
        // 在此之前 scanCallback 不允许继续做 NDT。
        initial_pose_received_ = false;

        RCLCPP_INFO(
            this->get_logger(),
            "NDT target map switched successfully.");

        RCLCPP_INFO(
            this->get_logger(),
            "Localization state reset. Waiting for new /initialpose.");

        return true;
    }


    rcl_interfaces::msg::SetParametersResult
    onParametersSet(
        const std::vector<rclcpp::Parameter> & parameters)
    {
        rcl_interfaces::msg::SetParametersResult result;

        result.successful = true;
        result.reason = "success";

        for (const auto & parameter : parameters)
        {
            if (parameter.get_name() != "map_path")
            {
                continue;
            }

            if (parameter.get_type() !=
                rclcpp::ParameterType::PARAMETER_STRING)
            {
                result.successful = false;
                result.reason =
                    "map_path must be a string";

                return result;
            }

            const std::string new_map_path =
                parameter.as_string();

            if (new_map_path.empty())
            {
                result.successful = false;
                result.reason =
                    "map_path cannot be empty";

                return result;
            }

            if (new_map_path == map_path_)
            {
                RCLCPP_INFO(
                    this->get_logger(),
                    "Requested NDT map is already active: %s",
                    new_map_path.c_str());

                continue;
            }

            std::string reason;

            // Stop new scan callbacks from starting while the new map
            // is being prepared and committed.
            map_switch_requested_.store(true);

            const bool reload_ok =
                reloadNdtMap(
                    new_map_path,
                    reason);

            map_switch_requested_.store(false);

            if (!reload_ok)
            {
                result.successful = false;
                result.reason = reason;

                return result;
            }
        }

        return result;
    }


    void initialPoseCallback(
        const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
    {
        std::lock_guard<std::mutex> state_lock(localization_state_mutex_);
        if (!msg->header.frame_id.empty() &&
            msg->header.frame_id != "map")
        {
            RCLCPP_WARN(
                this->get_logger(),
                "Rejected /initialpose: frame_id is '%s', expected 'map'.",
                msg->header.frame_id.c_str());
            return;
        }

        const auto & p = msg->pose.pose;

        // ------------------------------------------------------------
        // /initialpose represents T_map_base_link.
        //
        // RViz 2D Pose Estimate mainly supplies x/y/yaw.
        // For the current floor we preserve the current base_link z,
        // because the LIO-SAM map origin is approximately at LiDAR height.
        // ------------------------------------------------------------
        Eigen::Matrix4f old_T_map_base =
            current_lidar_pose_ * T_lidar_base_;

        const float base_z =
            old_T_map_base(2, 3);

        Eigen::Quaternionf q(
            static_cast<float>(p.orientation.w),
            static_cast<float>(p.orientation.x),
            static_cast<float>(p.orientation.y),
            static_cast<float>(p.orientation.z));

        if (q.norm() < 1e-6f)
        {
            RCLCPP_WARN(
                this->get_logger(),
                "Rejected /initialpose: invalid quaternion.");
            return;
        }

        q.normalize();

        Eigen::Matrix4f T_map_base =
            Eigen::Matrix4f::Identity();

        T_map_base.block<3, 3>(0, 0) =
            q.toRotationMatrix();

        T_map_base(0, 3) =
            static_cast<float>(p.position.x);

        T_map_base(1, 3) =
            static_cast<float>(p.position.y);

        T_map_base(2, 3) =
            base_z;

        // Convert base_link initial pose to LiDAR initial pose:
        //
        // T_map_lidar =
        // T_map_base * T_base_lidar
        current_lidar_pose_ =
            T_map_base * T_base_lidar_;

        last_valid_lidar_pose_ = current_lidar_pose_;
        last_valid_base_pose_ = T_map_base;
        have_last_valid_pose_ = true;
        have_last_accept_odom_ = false;

        {
            std::lock_guard<std::mutex> lock(map_odom_tf_mutex_);
            have_map_odom_tf_ = false;
        }

        initial_pose_received_ = true;

        const float yaw =
            std::atan2(
                T_map_base(1, 0),
                T_map_base(0, 0));

        RCLCPP_INFO(
            this->get_logger(),
            "Received /initialpose | "
            "base=(%.3f %.3f %.3f) yaw=%.3f rad | "
            "lidar guess=(%.3f %.3f %.3f)",
            T_map_base(0, 3),
            T_map_base(1, 3),
            T_map_base(2, 3),
            yaw,
            current_lidar_pose_(0, 3),
            current_lidar_pose_(1, 3),
            current_lidar_pose_(2, 3));
    }

    geometry_msgs::msg::PoseStamped matrixToPose(
        const Eigen::Matrix4f & transform,
        const builtin_interfaces::msg::Time & stamp)
    {
        geometry_msgs::msg::PoseStamped pose_msg;

        pose_msg.header.stamp = stamp;
        pose_msg.header.frame_id = "map";

        pose_msg.pose.position.x =
            transform(0, 3);

        pose_msg.pose.position.y =
            transform(1, 3);

        pose_msg.pose.position.z =
            transform(2, 3);

        Eigen::Matrix3f rotation =
            transform.block<3, 3>(0, 0);

        Eigen::Quaternionf q(rotation);
        q.normalize();

        pose_msg.pose.orientation.x = q.x();
        pose_msg.pose.orientation.y = q.y();
        pose_msg.pose.orientation.z = q.z();
        pose_msg.pose.orientation.w = q.w();

        return pose_msg;
    }

    void sendMapOdomTf(
        const Eigen::Matrix4f & T_map_odom,
        const builtin_interfaces::msg::Time & stamp)
    {
        geometry_msgs::msg::TransformStamped tf_msg;

        tf_msg.header.stamp = stamp;
        tf_msg.header.frame_id = "map";
        tf_msg.child_frame_id = "odom";

        tf_msg.transform.translation.x = T_map_odom(0, 3);
        tf_msg.transform.translation.y = T_map_odom(1, 3);
        tf_msg.transform.translation.z = T_map_odom(2, 3);

        Eigen::Matrix3f rotation =
            T_map_odom.block<3, 3>(0, 0);

        Eigen::Quaternionf q(rotation);
        q.normalize();

        tf_msg.transform.rotation.x = q.x();
        tf_msg.transform.rotation.y = q.y();
        tf_msg.transform.rotation.z = q.z();
        tf_msg.transform.rotation.w = q.w();

        tf_broadcaster_->sendTransform(tf_msg);
    }

    void publishMapOdomTf(
        const Eigen::Matrix4f & T_map_base,
        const Eigen::Matrix4f & T_odom_base,
        const builtin_interfaces::msg::Time & stamp)
    {
        // ------------------------------------------------------------
        // Standard localization TF:
        //
        // T_map_base = T_map_odom * T_odom_base
        //
        // Therefore:
        //
        // T_map_odom = T_map_base * inverse(T_odom_base)
        //
        // NDT updates this correction at a relatively low rate.
        // Between NDT updates, /odom refreshes the same correction
        // with a fresh odometry timestamp for Nav2 / TF consumers.
        // ------------------------------------------------------------
        const Eigen::Matrix4f T_map_odom =
            T_map_base * T_odom_base.inverse();

        {
            std::lock_guard<std::mutex> lock(map_odom_tf_mutex_);
            latest_T_map_odom_ = T_map_odom;
            have_map_odom_tf_ = true;
        }

        // Do not publish here with the old LiDAR scan timestamp.
        // The high-rate /odom callback will publish the newly updated
        // map -> odom correction using a fresh odometry timestamp.
    }

    void publishLatestMapOdomTf(
        const builtin_interfaces::msg::Time & stamp)
    {
        Eigen::Matrix4f T_map_odom;

        {
            std::lock_guard<std::mutex> lock(map_odom_tf_mutex_);

            if (!have_map_odom_tf_) {
                return;
            }

            T_map_odom = latest_T_map_odom_;
        }

        sendMapOdomTf(T_map_odom, stamp);
    }

    struct OdomSample
    {
        int64_t stamp_ns{0};
        Eigen::Matrix4f T_odom_base{Eigen::Matrix4f::Identity()};
    };

    static int64_t stampToNs(const builtin_interfaces::msg::Time & stamp)
    {
        return static_cast<int64_t>(stamp.sec) * 1000000000LL +
            static_cast<int64_t>(stamp.nanosec);
    }

    static double normalizeAngle(double angle)
    {
        return std::atan2(std::sin(angle), std::cos(angle));
    }

    static double yawOf(const Eigen::Matrix4f & transform)
    {
        return std::atan2(transform(1, 0), transform(0, 0));
    }

    static double planarDistance(
        const Eigen::Matrix4f & first,
        const Eigen::Matrix4f & second)
    {
        return std::hypot(
            static_cast<double>(first(0, 3) - second(0, 3)),
            static_cast<double>(first(1, 3) - second(1, 3)));
    }

    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        const auto & p = msg->pose.pose;
        Eigen::Quaternionf q(
            static_cast<float>(p.orientation.w),
            static_cast<float>(p.orientation.x),
            static_cast<float>(p.orientation.y),
            static_cast<float>(p.orientation.z));
        if (q.norm() < 1e-6f) {
            RCLCPP_WARN_THROTTLE(
                get_logger(), *get_clock(), 2000, "Ignoring invalid /odom quaternion");
            return;
        }
        q.normalize();
        OdomSample sample;
        sample.stamp_ns = stampToNs(msg->header.stamp);
        sample.T_odom_base.block<3, 3>(0, 0) = q.toRotationMatrix();
        sample.T_odom_base(0, 3) = static_cast<float>(p.position.x);
        sample.T_odom_base(1, 3) = static_cast<float>(p.position.y);
        sample.T_odom_base(2, 3) = static_cast<float>(p.position.z);
        {
            std::lock_guard<std::mutex> lock(odom_mutex_);

            if (!odom_buffer_.empty() &&
                sample.stamp_ns < odom_buffer_.back().stamp_ns)
            {
                RCLCPP_WARN(
                    get_logger(),
                    "/odom time moved backwards; clearing motion-prior buffer");

                odom_buffer_.clear();
                have_last_accept_odom_ = false;
            }

            odom_buffer_.push_back(sample);

            while (odom_buffer_.size() > 1000) {
                odom_buffer_.pop_front();
            }
        }

        // Keep map -> odom temporally fresh between relatively slow
        // NDT corrections. The transform value stays equal to the
        // latest accepted NDT correction, while its timestamp follows
        // the current odometry sample.
        // Refresh map -> odom using the timestamp carried by the
        // current odometry sample. /odom remains temporally fresh even
        // while the expensive NDT scan callback is running.
        publishLatestMapOdomTf(msg->header.stamp);
    }

    bool findOdom(
        const builtin_interfaces::msg::Time & stamp,
        OdomSample & result,
        double & time_difference)
    {
        std::lock_guard<std::mutex> lock(odom_mutex_);
        if (odom_buffer_.empty()) return false;
        const int64_t target = stampToNs(stamp);
        const OdomSample * best = nullptr;
        int64_t best_dt = std::numeric_limits<int64_t>::max();
        for (const auto & sample : odom_buffer_) {
            const int64_t dt = std::llabs(sample.stamp_ns - target);
            if (dt < best_dt) {
                best_dt = dt;
                best = &sample;
            }
        }
        time_difference = static_cast<double>(best_dt) / 1e9;
        if (best == nullptr || time_difference > max_odom_time_difference_) return false;
        result = *best;
        return true;
    }

    static diagnostic_msgs::msg::KeyValue keyValue(
        const std::string & key,
        const std::string & value)
    {
        diagnostic_msgs::msg::KeyValue item;
        item.key = key;
        item.value = value;
        return item;
    }

    static std::string number(double value)
    {
        std::ostringstream stream;
        stream.precision(17);
        stream << value;
        return stream.str();
    }

    void publishStatus(
        const builtin_interfaces::msg::Time & stamp,
        bool accepted,
        const std::string & reason,
        bool converged,
        double fitness,
        double compute_ms,
        bool odom_used,
        double odom_dt,
        const Eigen::Matrix4f & predicted,
        const Eigen::Matrix4f & raw,
        const Eigen::Matrix4f & final_pose,
        double position_jump,
        double yaw_jump,
        double prediction_position_error,
        double prediction_yaw_error)
    {
        diagnostic_msgs::msg::DiagnosticArray array;
        array.header.stamp = stamp;
        array.header.frame_id = "map";
        diagnostic_msgs::msg::DiagnosticStatus status;
        status.level = accepted ? diagnostic_msgs::msg::DiagnosticStatus::OK :
            diagnostic_msgs::msg::DiagnosticStatus::WARN;
        status.name = "ndt_localizer/quality_gate";
        status.message = accepted ? "NDT ACCEPT" : "NDT REJECT";
        status.hardware_id = "simulation";
        status.values = {
            keyValue("accepted", accepted ? "true" : "false"),
            keyValue("rejection_reason", reason),
            keyValue("converged", converged ? "true" : "false"),
            keyValue("fitness", number(fitness)),
            keyValue("compute_time_ms", number(compute_ms)),
            keyValue("odom_prior_used", odom_used ? "true" : "false"),
            keyValue("odom_time_difference_s", number(odom_dt)),
            keyValue("predicted_x", number(predicted(0, 3))),
            keyValue("predicted_y", number(predicted(1, 3))),
            keyValue("predicted_yaw", number(yawOf(predicted))),
            keyValue("raw_x", number(raw(0, 3))),
            keyValue("raw_y", number(raw(1, 3))),
            keyValue("raw_yaw", number(yawOf(raw))),
            keyValue("final_x", number(final_pose(0, 3))),
            keyValue("final_y", number(final_pose(1, 3))),
            keyValue("final_yaw", number(yawOf(final_pose))),
            keyValue("position_jump_m", number(position_jump)),
            keyValue("yaw_jump_rad", number(yaw_jump)),
            keyValue("prediction_position_error_m", number(prediction_position_error)),
            keyValue("prediction_yaw_error_rad", number(prediction_yaw_error))};
        array.status.push_back(status);
        status_pub_->publish(array);
    }

    void scanCallback(
        const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        // A floor-map switch has priority over starting a new NDT match.
        if (map_switch_requested_.load())
        {
            return;
        }

        std::lock_guard<std::mutex> state_lock(localization_state_mutex_);

        // Re-check after acquiring the state lock.
        if (map_switch_requested_.load())
        {
            return;
        }

        // When requested, do not run NDT until an initial pose arrives.
        if (!initial_pose_received_)
        {
            return;
        }

        PointCloudT::Ptr scan_raw(new PointCloudT);
        PointCloudT::Ptr scan_filtered(new PointCloudT);
        PointCloudT::Ptr aligned_cloud(new PointCloudT);

        pcl::fromROSMsg(
            *msg,
            *scan_raw);

        std::vector<int> indices;

        pcl::removeNaNFromPointCloud(
            *scan_raw,
            *scan_raw,
            indices);

        if (scan_raw->empty())
        {
            RCLCPP_WARN(
                this->get_logger(),
                "Received empty LiDAR scan.");

            return;
        }

        pcl::VoxelGrid<PointT> scan_voxel;

        scan_voxel.setLeafSize(
            static_cast<float>(scan_voxel_size_),
            static_cast<float>(scan_voxel_size_),
            static_cast<float>(scan_voxel_size_));

        scan_voxel.setInputCloud(scan_raw);
        scan_voxel.filter(*scan_filtered);

        if (scan_filtered->size() < 100)
        {
            RCLCPP_WARN(
                this->get_logger(),
                "Too few scan points after filtering: %zu",
                scan_filtered->size());

            return;
        }

        // ============================================================
        // NDT
        // ============================================================
        std::unique_lock<std::mutex> ndt_lock(ndt_mutex_);
        ndt_.setInputSource(scan_filtered);

        // Predict the pose at this scan time from the odometry increment since
        // the last accepted NDT result. Rejections deliberately leave both the
        // map pose and its odometry reference unchanged.
        Eigen::Matrix4f predicted_base_pose = last_valid_base_pose_;
        OdomSample current_odom;
        double odom_time_difference = std::numeric_limits<double>::quiet_NaN();
        const bool have_current_odom =
            findOdom(
                msg->header.stamp,
                current_odom,
                odom_time_difference);
        if (use_odom_prior_ && !have_current_odom) {
            RCLCPP_WARN_THROTTLE(
                get_logger(), *get_clock(), 2000,
                "No /odom sample within %.3f s of scan; using last valid pose as NDT prior",
                max_odom_time_difference_);
        }
        bool odom_prior_used = false;
        if (
            use_odom_prior_ &&
            have_last_valid_pose_ &&
            have_last_accept_odom_ &&
            have_current_odom)
        {
            const Eigen::Matrix4f delta_T =
                last_accept_odom_.T_odom_base.inverse() * current_odom.T_odom_base;
            predicted_base_pose = last_valid_base_pose_ * delta_T;
            odom_prior_used = true;
        }
        const Eigen::Matrix4f predicted_lidar_pose =
            predicted_base_pose * T_base_lidar_;

        predicted_pose_pub_->publish(
            matrixToPose(predicted_base_pose, msg->header.stamp));

        const auto ndt_start =
            std::chrono::steady_clock::now();

        ndt_.align(
            *aligned_cloud,
            predicted_lidar_pose);

        const auto ndt_end =
            std::chrono::steady_clock::now();

        const double ndt_time_ms =
            std::chrono::duration<double, std::milli>(
                ndt_end - ndt_start).count();

        const bool converged =
            ndt_.hasConverged();

        const double fitness =
            ndt_.getFitnessScore();

        std_msgs::msg::Float32 fitness_msg;
        fitness_msg.data =
            static_cast<float>(fitness);

        fitness_pub_->publish(fitness_msg);

        const Eigen::Matrix4f raw_lidar_pose = ndt_.getFinalTransformation();

        // NDT内部状态读取完成，后续quality gate和TF发布不再占用NDT锁。
        ndt_lock.unlock();
        const Eigen::Matrix4f raw_base_pose = raw_lidar_pose * T_lidar_base_;
        raw_pose_pub_->publish(matrixToPose(raw_base_pose, msg->header.stamp));

        const double position_jump = have_last_valid_pose_ ?
            planarDistance(raw_base_pose, last_valid_base_pose_) : 0.0;
        const double yaw_jump = have_last_valid_pose_ ?
            std::abs(normalizeAngle(yawOf(raw_base_pose) - yawOf(last_valid_base_pose_))) : 0.0;
        const double prediction_position_error =
            planarDistance(raw_base_pose, predicted_base_pose);
        const double prediction_yaw_error = std::abs(normalizeAngle(
            yawOf(raw_base_pose) - yawOf(predicted_base_pose)));

        std::vector<std::string> rejection_reasons;
        if (!converged) rejection_reasons.emplace_back("not_converged");
        if (!std::isfinite(fitness)) rejection_reasons.emplace_back("fitness_nonfinite");
        else if (fitness >= fitness_accept_threshold_) rejection_reasons.emplace_back("fitness");
        // If a synchronized odom prior is available, validate the NDT
        // solution against the odom-predicted pose. Comparing it again
        // against the last accepted pose can create a reject latch when
        // the robot has moved since the last accepted NDT update.
        if (!odom_prior_used) {
            if (position_jump >= max_position_jump_)
                rejection_reasons.emplace_back("position_jump");
            if (yaw_jump >= max_yaw_jump_)
                rejection_reasons.emplace_back("yaw_jump");
        }

        if (odom_prior_used &&
            prediction_position_error >= max_prediction_position_error_)
            rejection_reasons.emplace_back("prediction_position_error");

        if (odom_prior_used &&
            prediction_yaw_error >= max_prediction_yaw_error_)
            rejection_reasons.emplace_back("prediction_yaw_error");

        std::ostringstream reason_stream;
        for (size_t i = 0; i < rejection_reasons.size(); ++i) {
            if (i) reason_stream << ',';
            reason_stream << rejection_reasons[i];
        }
        const bool accepted = rejection_reasons.empty();

        if (!accepted) {
            ++rejected_count_;
            publishStatus(
                msg->header.stamp, false, reason_stream.str(), converged, fitness,
                ndt_time_ms, odom_prior_used, odom_time_difference,
                predicted_base_pose, raw_base_pose, last_valid_base_pose_,
                position_jump, yaw_jump, prediction_position_error,
                prediction_yaw_error);
            RCLCPP_WARN(
                get_logger(),
                "NDT REJECT | reason=%s | fitness=%.6f | "
                "jump=%.3f m/%.3f rad | innovation=%.3f m/%.3f rad | prior=%s",
                reason_stream.str().c_str(), fitness, position_jump, yaw_jump,
                prediction_position_error, prediction_yaw_error,
                odom_prior_used ? "odom" : "last_valid");
            return;
        }

        // Commit state only after every quality check passes. This prevents a
        // rejected local optimum from becoming the next frame's seed.
        current_lidar_pose_ = raw_lidar_pose;
        last_valid_lidar_pose_ = raw_lidar_pose;
        last_valid_base_pose_ = raw_base_pose;
        have_last_valid_pose_ = true;
        if (have_current_odom) {
            last_accept_odom_ = current_odom;
            have_last_accept_odom_ = true;
        }
        ++accepted_count_;

        publishStatus(
            msg->header.stamp, true, "", converged, fitness, ndt_time_ms,
            odom_prior_used, odom_time_difference, predicted_base_pose,
            raw_base_pose, last_valid_base_pose_, position_jump, yaw_jump,
            prediction_position_error, prediction_yaw_error);

        // ============================================================
        // Publish aligned scan
        // ============================================================
        sensor_msgs::msg::PointCloud2 aligned_msg;

        pcl::toROSMsg(
            *aligned_cloud,
            aligned_msg);

        aligned_msg.header.stamp =
            msg->header.stamp;

        aligned_msg.header.frame_id =
            "map";

        aligned_pub_->publish(
            aligned_msg);

        // LiDAR pose
        auto lidar_pose_msg =
            matrixToPose(
                last_valid_lidar_pose_,
                msg->header.stamp);

        lidar_pose_pub_->publish(
            lidar_pose_msg);

        // base_link pose
        auto base_pose_msg =
            matrixToPose(
                last_valid_base_pose_,
                msg->header.stamp);

        base_pose_pub_->publish(
            base_pose_msg);

        // ============================================================
        // TF map -> odom
        //
        // NDT provides the globally corrected T_map_base.
        // /odom provides T_odom_base.
        //
        // Only an ACCEPTED NDT result is allowed to update map -> odom.
        // odom -> base_link is published separately by
        // odom_tf_broadcaster at 50 Hz.
        // ============================================================
        if (have_current_odom) {
            publishMapOdomTf(
                last_valid_base_pose_,
                current_odom.T_odom_base,
                msg->header.stamp);
        } else {
            RCLCPP_WARN_THROTTLE(
                get_logger(),
                *get_clock(),
                2000,
                "NDT ACCEPT but no synchronized /odom is available; "
                "map -> odom TF was not updated.");
        }

        RCLCPP_INFO(
            get_logger(),
            "NDT ACCEPT | time=%.1f ms | fitness=%.6f | prior=%s | "
            "base=(%.3f %.3f %.3f) | accepted=%zu rejected=%zu",
            ndt_time_ms, fitness, odom_prior_used ? "odom" : "last_valid",
            last_valid_base_pose_(0, 3), last_valid_base_pose_(1, 3),
            last_valid_base_pose_(2, 3), accepted_count_, rejected_count_);
    }

    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr
        parameter_callback_handle_;

    std::string map_path_;

    double map_voxel_size_;
    double scan_voxel_size_;

    PointCloudT::Ptr map_cloud_;

    // Protect runtime NDT target switching against scan matching.
    std::mutex ndt_mutex_;

    // Protect localization state shared by scan, initialpose and map switching.
    std::mutex localization_state_mutex_;
    std::atomic_bool map_switch_requested_{false};

    pcl::NormalDistributionsTransform<
        PointT,
        PointT> ndt_;

    Eigen::Matrix4f current_lidar_pose_;
    Eigen::Matrix4f last_valid_lidar_pose_{Eigen::Matrix4f::Identity()};
    Eigen::Matrix4f last_valid_base_pose_{Eigen::Matrix4f::Identity()};

    Eigen::Matrix4f T_base_lidar_;
    Eigen::Matrix4f T_lidar_base_;

    rclcpp::Subscription<
        sensor_msgs::msg::PointCloud2>::SharedPtr scan_sub_;

    rclcpp::CallbackGroup::SharedPtr scan_callback_group_;

    rclcpp::Subscription<
        geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr
        initial_pose_sub_;

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::CallbackGroup::SharedPtr odom_callback_group_;
    std::deque<OdomSample> odom_buffer_;
    std::mutex odom_mutex_;
    OdomSample last_accept_odom_;

    Eigen::Matrix4f latest_T_map_odom_{Eigen::Matrix4f::Identity()};
    std::mutex map_odom_tf_mutex_;
    bool have_map_odom_tf_{false};

    bool wait_for_initialpose_{false};
    bool initial_pose_received_{true};
    bool have_last_valid_pose_{false};
    std::atomic_bool have_last_accept_odom_{false};

    double fitness_accept_threshold_{0.017};
    double max_position_jump_{0.15};
    double max_yaw_jump_{0.20};
    double max_prediction_position_error_{0.15};
    double max_prediction_yaw_error_{0.15};
    double max_odom_time_difference_{0.10};
    bool use_odom_prior_{true};
    std::string odom_topic_{"/odom"};

    rclcpp::Publisher<
        sensor_msgs::msg::PointCloud2>::SharedPtr aligned_pub_;

    rclcpp::Publisher<
        geometry_msgs::msg::PoseStamped>::SharedPtr base_pose_pub_;

    rclcpp::Publisher<
        geometry_msgs::msg::PoseStamped>::SharedPtr lidar_pose_pub_;

    rclcpp::Publisher<
        std_msgs::msg::Float32>::SharedPtr fitness_pub_;

    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr raw_pose_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr predicted_pose_pub_;
    rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr status_pub_;

    std::unique_ptr<
        tf2_ros::TransformBroadcaster> tf_broadcaster_;

    size_t accepted_count_{0};
    size_t rejected_count_{0};
};

int main(
    int argc,
    char ** argv)
{
    rclcpp::init(argc, argv);

    try
    {
        auto node = std::make_shared<NDTLocalizer>();
        rclcpp::executors::MultiThreadedExecutor executor(
            rclcpp::ExecutorOptions(), 2);
        executor.add_node(node);
        executor.spin();
    }
    catch (const std::exception & e)
    {
        RCLCPP_FATAL(
            rclcpp::get_logger("ndt_localizer"),
            "%s",
            e.what());
    }

    rclcpp::shutdown();

    return 0;
}
