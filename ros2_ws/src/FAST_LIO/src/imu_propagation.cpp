#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <fast_lio/msg/fast_lio_imu_state.hpp>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <deque>
#include <memory>
#include <cmath>
#include <algorithm>

#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

class ImuPropagation : public rclcpp::Node
{
public:
    ImuPropagation() : Node("imu_propagation")
    {
        imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
            "/livox/imu",
            rclcpp::SensorDataQoS(),
            [this](sensor_msgs::msg::Imu::ConstSharedPtr msg) {
                imuCallback(msg);
            });

        state_sub_ =
            create_subscription<fast_lio::msg::FastLioImuState>(
                "/fast_lio/imu_state",
                20,
                [this](fast_lio::msg::FastLioImuState::ConstSharedPtr msg) {
                    stateCallback(msg);
                });

        odom_pub_ =
            create_publisher<nav_msgs::msg::Odometry>(
                "/Odometry_imu", 20);
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
        RCLCPP_INFO(get_logger(), "IMU propagation node started");
    }

private:
    struct State
    {
        Eigen::Vector3d position = Eigen::Vector3d::Zero();
        Eigen::Vector3d velocity = Eigen::Vector3d::Zero();

        Eigen::Quaterniond orientation =
            Eigen::Quaterniond::Identity();

        Eigen::Vector3d gyro_bias = Eigen::Vector3d::Zero();
        Eigen::Vector3d accel_bias = Eigen::Vector3d::Zero();
        Eigen::Vector3d gravity = Eigen::Vector3d::Zero();

        double accel_scale = 1.0;
        double timestamp = 0.0;
    };

    struct ImuSample
    {
        double timestamp;
        Eigen::Vector3d gyro;
        Eigen::Vector3d accel;
    };

    State state_;
    Eigen::Vector3d angular_velocity_body_ = Eigen::Vector3d::Zero();

    bool initialized_ = false;
    double last_correction_time_ = -1.0;

    std::deque<ImuSample> imu_buffer_;

    static constexpr double BUFFER_DURATION = 5.0;
    static constexpr double MAX_IMU_DT = 0.1;

    nav_msgs::msg::Odometry odom_template_;

    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;

    rclcpp::Subscription<
        fast_lio::msg::FastLioImuState>::SharedPtr state_sub_;

    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;

    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    
    static double stampToSeconds(const builtin_interfaces::msg::Time &stamp)
    {
        return static_cast<double>(stamp.sec) +
               static_cast<double>(stamp.nanosec) * 1e-9;
    }

    void propagate(const ImuSample &imu)
    {
        const double dt = imu.timestamp - state_.timestamp;

        if (dt <= 0.0)
            return;

        if (dt > MAX_IMU_DT)
        {
            RCLCPP_WARN_THROTTLE(
                get_logger(), *get_clock(), 2000,
                "Large IMU gap (%.3f s); skipping integration",
                dt);

            state_.timestamp = imu.timestamp;
            return;
        }

        // Remove estimated gyroscope bias
        const Eigen::Vector3d omega =
            imu.gyro - state_.gyro_bias;
            angular_velocity_body_ = omega;

        // Apply FAST-LIO acceleration scaling and bias correction
        const Eigen::Vector3d accel_body =
            imu.accel * state_.accel_scale - state_.accel_bias;

        // Integrate orientation using body-frame angular velocity
        const double omega_norm = omega.norm();

        Eigen::Quaterniond delta_half =
            Eigen::Quaterniond::Identity();

        Eigen::Quaterniond delta_full =
            Eigen::Quaterniond::Identity();

        if (omega_norm > 1e-12)
        {
            const Eigen::Vector3d axis = omega / omega_norm;

            delta_half = Eigen::Quaterniond(
                Eigen::AngleAxisd(omega_norm * dt * 0.5, axis));

            delta_full = Eigen::Quaterniond(
                Eigen::AngleAxisd(omega_norm * dt, axis));
        }

        // Midpoint orientation for acceleration transformation
        const Eigen::Quaterniond q_mid =
            state_.orientation * delta_half;

        // Acceleration in camera_init frame, including gravity
        const Eigen::Vector3d accel_world =
            q_mid * accel_body + state_.gravity;

        // Integrate position and velocity
        state_.position +=
            state_.velocity * dt + 0.5 * accel_world * dt * dt;

        state_.velocity += accel_world * dt;

        state_.orientation =
            (state_.orientation * delta_full).normalized();

        state_.timestamp = imu.timestamp;
    }

    void imuCallback(
        const sensor_msgs::msg::Imu::ConstSharedPtr msg)
    {
        ImuSample sample;

        sample.timestamp = stampToSeconds(msg->header.stamp);

        sample.gyro = Eigen::Vector3d(
            msg->angular_velocity.x,
            msg->angular_velocity.y,
            msg->angular_velocity.z);

        sample.accel = Eigen::Vector3d(
            msg->linear_acceleration.x,
            msg->linear_acceleration.y,
            msg->linear_acceleration.z);

        // Ignore duplicate or out-of-order IMU timestamps
        if (!imu_buffer_.empty() &&
            sample.timestamp <= imu_buffer_.back().timestamp)
        {
            return;
        }

        imu_buffer_.push_back(sample);

        // Keep recent IMU history for delayed corrections
        while (!imu_buffer_.empty() &&
               sample.timestamp - imu_buffer_.front().timestamp >
                   BUFFER_DURATION)
        {
            imu_buffer_.pop_front();
        }

        if (!initialized_)
            return;

        if (sample.timestamp <= state_.timestamp)
            return;

        propagate(sample);
        publishOdometry();
    }

    void stateCallback(
        const fast_lio::msg::FastLioImuState::ConstSharedPtr msg)
    {
        const auto &odom = msg->odometry;

        const double correction_time =
            stampToSeconds(odom.header.stamp);

        // Never replace a newer correction with an older one
        if (correction_time <= last_correction_time_)
            return;

        // Reject corrections if the required IMU history
        // has already been discarded.
        if (initialized_ && !imu_buffer_.empty() &&
            correction_time < imu_buffer_.front().timestamp - 0.02 &&
            correction_time < state_.timestamp)
        {
            RCLCPP_WARN(
                get_logger(),
                "Correction older than IMU buffer; skipping");
            return;
        }

        // FAST-LIO state is authoritative:
        // overwrite the predicted state completely.
        state_.position = Eigen::Vector3d(
            odom.pose.pose.position.x,
            odom.pose.pose.position.y,
            odom.pose.pose.position.z);

        state_.orientation = Eigen::Quaterniond(
            odom.pose.pose.orientation.w,
            odom.pose.pose.orientation.x,
            odom.pose.pose.orientation.y,
            odom.pose.pose.orientation.z).normalized();

        // FAST-LIO publishes linear velocity in body frame.
        // Convert it back to world frame for integration.
        const Eigen::Vector3d velocity_body(
            odom.twist.twist.linear.x,
            odom.twist.twist.linear.y,
            odom.twist.twist.linear.z);

        state_.velocity =
            state_.orientation * velocity_body;

        state_.gyro_bias = Eigen::Vector3d(
            msg->gyro_bias.x,
            msg->gyro_bias.y,
            msg->gyro_bias.z);

        state_.accel_bias = Eigen::Vector3d(
            msg->accel_bias.x,
            msg->accel_bias.y,
            msg->accel_bias.z);

        state_.gravity = Eigen::Vector3d(
            msg->gravity.x,
            msg->gravity.y,
            msg->gravity.z);

        state_.accel_scale = msg->accel_scale;
        state_.timestamp = correction_time;

        // Retain frame IDs and corrected covariance information.
        // Covariance is not propagated between corrections.
        odom_template_ = odom;

        last_correction_time_ = correction_time;
        initialized_ = true;

        // Update angular velocity using the corrected gyroscope bias.
        // Prefer the most recent IMU sample at or before the correction.
        angular_velocity_body_.setZero();

        for (auto it = imu_buffer_.rbegin(); it != imu_buffer_.rend(); ++it)
        {
            if (it->timestamp <= correction_time)
            {
                angular_velocity_body_ = it->gyro - state_.gyro_bias;
                break;
            }
        }
        
        // Replay IMU measurements newer than the correction.
        // This brings the corrected estimate back to the present.
        for (const auto &imu : imu_buffer_)
        {
            if (imu.timestamp > state_.timestamp)
                propagate(imu);
        }

        publishOdometry();
    }

    void publishOdometry()
    {
        if (!initialized_)
            return;

        nav_msgs::msg::Odometry odom = odom_template_;

        rclcpp::Time stamp(
            static_cast<int64_t>(std::floor(state_.timestamp)),
            static_cast<uint32_t>(
                std::llround(
                    (state_.timestamp - std::floor(state_.timestamp)) *
                    1e9)));

        odom.header.stamp = stamp;

        odom.pose.pose.position.x = state_.position.x();
        odom.pose.pose.position.y = state_.position.y();
        odom.pose.pose.position.z = state_.position.z();

        odom.pose.pose.orientation.w = state_.orientation.w();
        odom.pose.pose.orientation.x = state_.orientation.x();
        odom.pose.pose.orientation.y = state_.orientation.y();
        odom.pose.pose.orientation.z = state_.orientation.z();

        // Publish velocity in body frame, matching /Odometry
        const Eigen::Vector3d velocity_body =
            state_.orientation.conjugate() * state_.velocity;

        odom.twist.twist.linear.x = velocity_body.x();
        odom.twist.twist.linear.y = velocity_body.y();
        odom.twist.twist.linear.z = velocity_body.z();

        // Bias-corrected angular velocity in the IMU body frame
        odom.twist.twist.angular.x = angular_velocity_body_.x();
        odom.twist.twist.angular.y = angular_velocity_body_.y();
        odom.twist.twist.angular.z = angular_velocity_body_.z();

        odom_pub_->publish(odom);
	// Publish IMU-propagated transform
	geometry_msgs::msg::TransformStamped transform;

	transform.header = odom.header;
	transform.child_frame_id = "body_imu";

	transform.transform.translation.x = state_.position.x();
	transform.transform.translation.y = state_.position.y();
	transform.transform.translation.z = state_.position.z();

	transform.transform.rotation = odom.pose.pose.orientation;

	tf_broadcaster_->sendTransform(transform);
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    rclcpp::spin(std::make_shared<ImuPropagation>());

    rclcpp::shutdown();
    return 0;
}
