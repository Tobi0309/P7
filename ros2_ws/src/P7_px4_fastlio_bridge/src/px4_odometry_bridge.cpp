#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <memory>
#include <limits>
#include <cmath>
#include <cstdint>

class Px4OdometryBridge : public rclcpp::Node
{
public:
    Px4OdometryBridge()
        : Node("px4_odometry_bridge")
    {
        odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
            "/Odometry_base_link", 20,
            [this](nav_msgs::msg::Odometry::ConstSharedPtr msg)
            {
                odomCallback(msg);
            });

        px4_pub_ = create_publisher<px4_msgs::msg::VehicleOdometry>(
            "/fmu/in/vehicle_visual_odometry", 10);
        pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(
            "~/pose", 10);

        RCLCPP_INFO(
            get_logger(),
            "PX4 odometry bridge started (OFFLINE TEST MODE, ~50 Hz); "
            "publishing RViz pose on ~/pose");
    }

private:
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<px4_msgs::msg::VehicleOdometry>::SharedPtr px4_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;

    uint64_t last_publish_us_ = 0;

    void odomCallback(
        const nav_msgs::msg::Odometry::ConstSharedPtr msg)
    {
        // Convert ROS measurement timestamp to microseconds.
        const uint64_t stamp_us =
            static_cast<uint64_t>(msg->header.stamp.sec) * 1000000ULL +
            static_cast<uint64_t>(msg->header.stamp.nanosec) / 1000ULL;

        if (stamp_us == 0)
        {
            return;
        }

        // Handle simulation clock resets or rosbag restarts.
        if (stamp_us < last_publish_us_)
        {
            last_publish_us_ = 0;
        }

        // Publish at approximately 50 Hz.
        constexpr uint64_t publish_interval_us = 20000;

        if (last_publish_us_ != 0 &&
            stamp_us - last_publish_us_ < publish_interval_us)
        {
            return;
        }

        // Incoming odometry already represents base_link.
        const Eigen::Vector3d p_world_base(
            msg->pose.pose.position.x,
            msg->pose.pose.position.y,
            msg->pose.pose.position.z);

        Eigen::Quaterniond q_world_base(
            msg->pose.pose.orientation.w,
            msg->pose.pose.orientation.x,
            msg->pose.pose.orientation.y,
            msg->pose.pose.orientation.z);

        if (!q_world_base.coeffs().allFinite() ||
            q_world_base.norm() < 1e-6)
        {
            return;
        }

        q_world_base.normalize();

        // World Z-up -> world FRD (180 degrees around X).
        const Eigen::Quaterniond q_frd_world(
            Eigen::AngleAxisd(
                M_PI, Eigen::Vector3d::UnitX()));

        // Body FLU -> body FRD.
        const Eigen::Quaterniond q_flu_frd(
            Eigen::AngleAxisd(
                M_PI, Eigen::Vector3d::UnitX()));

        const Eigen::Vector3d p_frd =
            q_frd_world * p_world_base;

        const Eigen::Quaterniond q_frd_body =
            (q_frd_world * q_world_base * q_flu_frd).normalized();

        px4_msgs::msg::VehicleOdometry output{};

        // OFFLINE TEST ONLY:
        // Not yet synchronized with PX4's clock.
        output.timestamp = stamp_us;
        output.timestamp_sample = stamp_us;

        output.pose_frame =
            px4_msgs::msg::VehicleOdometry::POSE_FRAME_FRD;

        output.position = {
            static_cast<float>(p_frd.x()),
            static_cast<float>(p_frd.y()),
            static_cast<float>(p_frd.z())
        };

        // PX4 quaternion order: w, x, y, z.
        output.q = {
            static_cast<float>(q_frd_body.w()),
            static_cast<float>(q_frd_body.x()),
            static_cast<float>(q_frd_body.y()),
            static_cast<float>(q_frd_body.z())
        };

        // Velocity fusion disabled for this initial test.
        const float nan = std::numeric_limits<float>::quiet_NaN();

        output.velocity_frame =
            px4_msgs::msg::VehicleOdometry::VELOCITY_FRAME_UNKNOWN;

        output.velocity = {nan, nan, nan};
        output.angular_velocity = {nan, nan, nan};

        // Placeholder variances; validate before EKF2 fusion.
        output.position_variance = {0.25f, 0.25f, 0.25f};
        output.orientation_variance = {0.25f, 0.25f, 0.25f};
        output.velocity_variance = {nan, nan, nan};

        output.reset_counter = 0;
        output.quality = 1;

        geometry_msgs::msg::PoseStamped pose;
        pose.header = msg->header;
        pose.pose = msg->pose.pose;
        pose.pose.orientation.w = q_world_base.w();
        pose.pose.orientation.x = q_world_base.x();
        pose.pose.orientation.y = q_world_base.y();
        pose.pose.orientation.z = q_world_base.z();

        px4_pub_->publish(output);
        pose_pub_->publish(pose);

        last_publish_us_ = stamp_us;
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Px4OdometryBridge>());
    rclcpp::shutdown();
    return 0;
}