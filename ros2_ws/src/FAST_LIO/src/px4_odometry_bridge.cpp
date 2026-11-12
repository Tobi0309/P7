#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/exceptions.h>

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
        : Node("px4_odometry_bridge"),
          tf_buffer_(this->get_clock()),
          tf_listener_(tf_buffer_)
    {
        odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
            "/Odometry", 20,
            [this](nav_msgs::msg::Odometry::ConstSharedPtr msg)
            {
                odomCallback(msg);
            });

        px4_pub_ = create_publisher<px4_msgs::msg::VehicleOdometry>(
            "/p7/px4/vehicle_visual_odometry_test", 10);

        RCLCPP_INFO(
            get_logger(),
            "PX4 odometry bridge started (OFFLINE TEST MODE)");
    }

private:
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;

    rclcpp::Publisher<px4_msgs::msg::VehicleOdometry>::SharedPtr px4_pub_;

    void odomCallback(
        const nav_msgs::msg::Odometry::ConstSharedPtr msg)
    {
        geometry_msgs::msg::TransformStamped tf;

        try
        {
            // Fixed transform from Livox IMU to aircraft center
            tf = tf_buffer_.lookupTransform(
                "body_imu",
                "base_link",
                tf2::TimePointZero);
        }
        catch (const tf2::TransformException &ex)
        {
            RCLCPP_WARN_THROTTLE(
                get_logger(), *get_clock(), 2000,
                "Waiting for IMU-to-aircraft TF: %s",
                ex.what());
            return;
        }

        // FAST-LIO IMU position in camera_init
        const Eigen::Vector3d p_world_imu(
            msg->pose.pose.position.x,
            msg->pose.pose.position.y,
            msg->pose.pose.position.z);

        Eigen::Quaterniond q_world_imu(
            msg->pose.pose.orientation.w,
            msg->pose.pose.orientation.x,
            msg->pose.pose.orientation.y,
            msg->pose.pose.orientation.z);

        q_world_imu.normalize();

        // URDF mounting translation and rotation
        const Eigen::Vector3d r_imu_base(
            tf.transform.translation.x,
            tf.transform.translation.y,
            tf.transform.translation.z);

        Eigen::Quaterniond q_imu_base(
            tf.transform.rotation.w,
            tf.transform.rotation.x,
            tf.transform.rotation.y,
            tf.transform.rotation.z);

        q_imu_base.normalize();

        // Aircraft-centered pose in camera_init
        const Eigen::Vector3d p_world_base =
            p_world_imu + q_world_imu * r_imu_base;

        const Eigen::Quaterniond q_world_base =
            (q_world_imu * q_imu_base).normalized();

        // Convert Z-up world to Z-down FRD world.
        // Arbitrary heading is preserved.
        const Eigen::Quaterniond q_frd_world(
            Eigen::AngleAxisd(
                M_PI, Eigen::Vector3d::UnitX()));

        // Convert aircraft FLU to body FRD
        const Eigen::Quaterniond q_flu_frd(
            Eigen::AngleAxisd(
                M_PI, Eigen::Vector3d::UnitX()));

        const Eigen::Vector3d p_frd =
            q_frd_world * p_world_base;

        const Eigen::Quaterniond q_frd_body =
            (q_frd_world * q_world_base * q_flu_frd).normalized();

        px4_msgs::msg::VehicleOdometry output{};

        // OFFLINE TEST ONLY:
        // These are ROS measurement timestamps, not PX4-synchronized time.
        const uint64_t stamp_us =
            static_cast<uint64_t>(msg->header.stamp.sec) * 1000000ULL +
            static_cast<uint64_t>(msg->header.stamp.nanosec) / 1000ULL;

        output.timestamp = stamp_us;
        output.timestamp_sample = stamp_us;

        output.pose_frame =
            px4_msgs::msg::VehicleOdometry::POSE_FRAME_FRD;

        output.position = {
            static_cast<float>(p_frd.x()),
            static_cast<float>(p_frd.y()),
            static_cast<float>(p_frd.z())
        };

        // PX4 quaternion ordering: w, x, y, z
        output.q = {
            static_cast<float>(q_frd_body.w()),
            static_cast<float>(q_frd_body.x()),
            static_cast<float>(q_frd_body.y()),
            static_cast<float>(q_frd_body.z())
        };

        // Velocity not supplied in this initial conversion.
        const float nan = std::numeric_limits<float>::quiet_NaN();

        output.velocity_frame =
            px4_msgs::msg::VehicleOdometry::VELOCITY_FRAME_UNKNOWN;

        output.velocity = {nan, nan, nan};
        output.angular_velocity = {nan, nan, nan};

        // Conservative placeholder variances for offline testing.
        // These must be validated before EKF2 fusion.
        output.position_variance = {0.25f, 0.25f, 0.25f};
        output.orientation_variance = {0.25f, 0.25f, 0.25f};
        output.velocity_variance = {nan, nan, nan};

        output.reset_counter = 0;
        output.quality = 1;

        px4_pub_->publish(output);
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    rclcpp::spin(std::make_shared<Px4OdometryBridge>());

    rclcpp::shutdown();
    return 0;
}
