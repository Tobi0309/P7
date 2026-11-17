// FAST-LIO -> PX4 external vision bridge (PX4 Zenoh middleware).
//
// Subscribes to base_link odometry (nav_msgs/Odometry, world = FAST-LIO
// "camera_init" frame, Z up / FLU; body = base_link FLU) and publishes
// px4_msgs/VehicleOdometry on /fmu/in/vehicle_visual_odometry in the
// FRD world frame / FRD body frame that EKF2 expects.
//
// ---------------------------------------------------------------------------
// Time synchronisation
// ---------------------------------------------------------------------------
// PX4's Zenoh module does NOT do any time sync. On receive it overwrites
// `timestamp` with hrt_absolute_time() and passes `timestamp_sample` through
// untouched. EKF2 uses `timestamp_sample` (minus EKF2_EV_DELAY) to place the
// measurement in its delayed-time horizon, so it must be in PX4 boot time.
//
// The odometry stamps are in whatever clock the Livox driver uses (device
// time unless the lidar is PTP-synced, sim time in simulation, ...), so we
// do not assume any relation between that clock and PX4. Instead we track
// two offsets against the node clock (ROS time: system time on hardware,
// /clock when use_sim_time is true):
//
//   off_px4 = min_window( now_rx - px4_msg.timestamp )   (PX4 -> us)
//   off_src = min_window( now_rx - odom.header.stamp )   (FAST-LIO -> us)
//
// In simulation use_sim_time MUST be true: PX4 SITL runs in lockstep with the
// simulator, so if the sim runs slower than real time a wall/steady clock
// drifts against both PX4 and the odometry stamps and the offsets go wrong.
//
// Taking the windowed minimum removes transport / scheduling jitter (the
// fastest message in the window is the one closest to zero latency). Then
//
//   timestamp_sample = odom_stamp + off_src - off_px4
//
// The remaining *constant* pipeline latency (lidar/IMU -> FAST-LIO output
// for the fastest message) cannot be observed here; compensate it with the
// PX4 parameter EKF2_EV_DELAY.
// ---------------------------------------------------------------------------

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <px4_msgs/msg/vehicle_attitude.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/static_transform_broadcaster.h>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <chrono>
#include <cmath>
#include <cstdint>
#include <deque>
#include <limits>
#include <memory>
#include <string>
#include <utility>

namespace
{

int64_t stampToUs(const builtin_interfaces::msg::Time &stamp)
{
    return static_cast<int64_t>(stamp.sec) * 1000000LL +
           static_cast<int64_t>(stamp.nanosec) / 1000LL;
}

// Sliding-window minimum (monotonic deque, O(1) amortised).
class WindowedMin
{
public:
    explicit WindowedMin(int64_t window_us = 5000000) : window_us_(window_us) {}

    void setWindow(int64_t window_us) { window_us_ = window_us; }

    void add(int64_t t_us, int64_t value)
    {
        while (!q_.empty() && q_.back().second >= value)
        {
            q_.pop_back();
        }
        q_.emplace_back(t_us, value);
        while (q_.front().first < t_us - window_us_)
        {
            q_.pop_front();
        }
        last_add_us_ = t_us;
    }

    bool valid() const { return !q_.empty(); }
    int64_t min() const { return q_.front().second; }
    int64_t lastAddUs() const { return last_add_us_; }

    void reset()
    {
        q_.clear();
        last_add_us_ = 0;
    }

private:
    int64_t window_us_;
    int64_t last_add_us_{0};
    std::deque<std::pair<int64_t, int64_t>> q_;
};

}  // namespace

class Px4OdometryBridge : public rclcpp::Node
{
public:
    Px4OdometryBridge()
        : Node("px4_odometry_bridge")
    {
        const auto odom_topic =
            declare_parameter<std::string>("odom_topic", "/Odometry_base_link");
        const auto px4_odom_topic =
            declare_parameter<std::string>("px4_odom_topic", "/fmu/in/vehicle_visual_odometry");
        const auto px4_clock_topic =
            declare_parameter<std::string>("px4_clock_topic", "/fmu/out/vehicle_attitude");

        const double rate_hz = declare_parameter<double>("publish_rate_hz", 50.0);
        min_period_us_ = rate_hz > 0.0 ? static_cast<int64_t>(1e6 / rate_hz) : 0;

        send_velocity_ = declare_parameter<bool>("send_velocity", true);

        const double pos_std = declare_parameter<double>("position_std_m", 0.05);
        const double att_std = declare_parameter<double>("attitude_std_rad", 0.05);
        const double vel_std = declare_parameter<double>("velocity_std_mps", 0.10);
        pos_var_ = static_cast<float>(pos_std * pos_std);
        att_var_ = static_cast<float>(att_std * att_std);
        vel_var_ = static_cast<float>(vel_std * vel_std);

        const double window_s = declare_parameter<double>("sync_window_s", 5.0);
        px4_off_.setWindow(static_cast<int64_t>(window_s * 1e6));
        src_off_.setWindow(static_cast<int64_t>(window_s * 1e6));

        px4_timeout_us_ = static_cast<int64_t>(
            declare_parameter<double>("px4_clock_timeout_s", 1.0) * 1e6);
        stamp_jump_us_ = static_cast<int64_t>(
            declare_parameter<double>("stamp_jump_threshold_s", 1.0) * 1e6);

        publish_debug_pose_ = declare_parameter<bool>("publish_debug_pose", true);
        px4_frame_id_ = declare_parameter<std::string>("px4_frame_id", "px4_frd");
        px4_frame_yaw_rad_ =
            declare_parameter<double>("px4_frame_yaw_deg", 90.0) * M_PI / 180.0;

        odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
            odom_topic, rclcpp::SensorDataQoS(),
            [this](nav_msgs::msg::Odometry::ConstSharedPtr msg) { odomCallback(*msg); });

        px4_clock_sub_ = create_subscription<px4_msgs::msg::VehicleAttitude>(
            px4_clock_topic, rclcpp::SensorDataQoS(),
            [this](px4_msgs::msg::VehicleAttitude::ConstSharedPtr msg) { px4ClockCallback(*msg); });

        px4_pub_ = create_publisher<px4_msgs::msg::VehicleOdometry>(px4_odom_topic, 10);

        if (publish_debug_pose_)
        {
            pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>("~/px4_pose", 10);
            static_tf_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
        }

        // Wall timer: works even when use_sim_time is set and no /clock arrives.
        status_timer_ = create_wall_timer(std::chrono::seconds(2), [this]() { logStatus(); });

        RCLCPP_INFO(get_logger(),
                    "Bridge %s -> %s (clock ref: %s, %.0f Hz, velocity %s)",
                    odom_topic.c_str(), px4_odom_topic.c_str(), px4_clock_topic.c_str(),
                    rate_hz, send_velocity_ ? "on" : "off");
    }

private:
    // --------------------------------------------------------------- status
    void logStatus()
    {
        RCLCPP_INFO(get_logger(),
                    "last 2 s: odom rx %u, px4 clock rx %u, published %u | dropped: "
                    "zero stamp %u, duplicate stamp %u, no clock (px4 or /clock) %u, bad pose %u",
                    n_odom_, n_px4_, n_pub_, n_zero_stamp_, n_dup_, n_no_clock_, n_bad_pose_);
        if (n_odom_ == 0)
        {
            RCLCPP_WARN(get_logger(), "No odometry received - is %s being published?",
                        odom_sub_->get_topic_name());
        }
        if (n_px4_ == 0)
        {
            RCLCPP_WARN(get_logger(), "No PX4 messages on %s - is the PX4 Zenoh bridge up?",
                        px4_clock_sub_->get_topic_name());
        }
        n_odom_ = n_px4_ = n_pub_ = n_zero_stamp_ = n_dup_ = n_no_clock_ = n_bad_pose_ = 0;
    }

    // ---------------------------------------------------------------- clock
    // Node clock in microseconds, or 0 if not available yet (sim time before
    // the first /clock message). Resets the sync if the clock jumps back.
    int64_t nodeNowUs()
    {
        const int64_t now_us = get_clock()->now().nanoseconds() / 1000;
        if (now_us <= 0)
        {
            return 0;
        }
        if (now_us < last_now_us_)
        {
            RCLCPP_WARN(get_logger(), "Node clock went backwards (sim reset?), resetting sync");
            px4_off_.reset();
            src_off_.reset();
            last_px4_us_ = 0;
            last_src_us_ = 0;
            last_pub_src_us_ = 0;
        }
        last_now_us_ = now_us;
        return now_us;
    }

    void px4ClockCallback(const px4_msgs::msg::VehicleAttitude &msg)
    {
        const int64_t rx_us = nodeNowUs();
        if (rx_us == 0)
        {
            return;
        }
        ++n_px4_;
        const auto px4_us = static_cast<int64_t>(msg.timestamp);
        if (px4_us <= 0)
        {
            return;
        }

        // PX4 rebooted -> its clock restarted.
        if (px4_us < last_px4_us_)
        {
            RCLCPP_WARN(get_logger(), "PX4 clock went backwards (reboot?), resetting sync");
            px4_off_.reset();
        }
        last_px4_us_ = px4_us;

        px4_off_.add(rx_us, rx_us - px4_us);
    }

    bool px4ClockValid(int64_t now_us) const
    {
        return px4_off_.valid() && (now_us - px4_off_.lastAddUs()) < px4_timeout_us_;
    }

    // ------------------------------------------------------------- odometry
    void odomCallback(const nav_msgs::msg::Odometry &msg)
    {
        const int64_t rx_us = nodeNowUs();
        ++n_odom_;
        if (rx_us == 0)
        {
            ++n_no_clock_;
            return;
        }
        const int64_t src_us = stampToUs(msg.header.stamp);
        if (src_us <= 0)
        {
            ++n_zero_stamp_;
            return;
        }

        // Source clock discontinuity (FAST-LIO restart, bag loop, sim reset).
        if (last_src_us_ != 0)
        {
            const int64_t d = src_us - last_src_us_;
            if (d < 0 || d > stamp_jump_us_)
            {
                RCLCPP_WARN(get_logger(),
                            "Odometry stamp jumped by %.3f s, resetting source sync", d * 1e-6);
                src_off_.reset();
                last_pub_src_us_ = 0;
                if (d < 0)
                {
                    ++reset_counter_;  // pose estimate likely restarted
                }
            }
            else if (d == 0)
            {
                ++n_dup_;
                return;  // duplicate
            }
        }
        last_src_us_ = src_us;
        src_off_.add(rx_us, rx_us - src_us);

        // Rate limit on source time.
        if (min_period_us_ > 0 && last_pub_src_us_ != 0 &&
            src_us - last_pub_src_us_ < min_period_us_)
        {
            return;
        }

        if (!px4ClockValid(rx_us))
        {
            ++n_no_clock_;
            return;
        }

        // ---- time conversion ------------------------------------------------
        const int64_t off_px4 = px4_off_.min();
        const int64_t px4_now_us = rx_us - off_px4;           // never ahead of PX4
        int64_t sample_us = src_us + src_off_.min() - off_px4;  // in PX4 boot time
        if (sample_us > px4_now_us)
        {
            sample_us = px4_now_us;
        }
        if (sample_us <= 0)
        {
            return;
        }

        // ---- frame conversion ---------------------------------------------
        const Eigen::Vector3d p_w(msg.pose.pose.position.x,
                                  msg.pose.pose.position.y,
                                  msg.pose.pose.position.z);
        Eigen::Quaterniond q_wb(msg.pose.pose.orientation.w,
                                msg.pose.pose.orientation.x,
                                msg.pose.pose.orientation.y,
                                msg.pose.pose.orientation.z);
        if (!p_w.allFinite() || !q_wb.coeffs().allFinite() || q_wb.norm() < 1e-6)
        {
            ++n_bad_pose_;
            return;
        }
        q_wb.normalize();

        // World Z-up (FLU at start) -> world FRD, and body FLU -> body FRD
        // are both a 180 deg rotation about X.
        static const Eigen::Quaterniond q_x180(
            Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitX()));

        const Eigen::Vector3d p_frd = q_x180 * p_w;
        const Eigen::Quaterniond q_frd = (q_x180 * q_wb * q_x180).normalized();

        px4_msgs::msg::VehicleOdometry out{};
        out.timestamp = static_cast<uint64_t>(px4_now_us);  // overwritten by Zenoh anyway
        out.timestamp_sample = static_cast<uint64_t>(sample_us);

        out.pose_frame = px4_msgs::msg::VehicleOdometry::POSE_FRAME_FRD;
        out.position = {static_cast<float>(p_frd.x()),
                        static_cast<float>(p_frd.y()),
                        static_cast<float>(p_frd.z())};
        out.q = {static_cast<float>(q_frd.w()), static_cast<float>(q_frd.x()),
                 static_cast<float>(q_frd.y()), static_cast<float>(q_frd.z())};

        const float nan = std::numeric_limits<float>::quiet_NaN();

        // Twist in /Odometry_base_link is expressed in body FLU.
        const auto &v = msg.twist.twist.linear;
        const auto &w = msg.twist.twist.angular;
        const bool vel_ok = send_velocity_ &&
                            std::isfinite(v.x) && std::isfinite(v.y) && std::isfinite(v.z);

        if (vel_ok)
        {
            out.velocity_frame = px4_msgs::msg::VehicleOdometry::VELOCITY_FRAME_BODY_FRD;
            out.velocity = {static_cast<float>(v.x), static_cast<float>(-v.y),
                            static_cast<float>(-v.z)};
            out.velocity_variance = {vel_var_, vel_var_, vel_var_};
        }
        else
        {
            out.velocity_frame = px4_msgs::msg::VehicleOdometry::VELOCITY_FRAME_UNKNOWN;
            out.velocity = {nan, nan, nan};
            out.velocity_variance = {nan, nan, nan};
        }

        if (std::isfinite(w.x) && std::isfinite(w.y) && std::isfinite(w.z))
        {
            out.angular_velocity = {static_cast<float>(w.x), static_cast<float>(-w.y),
                                    static_cast<float>(-w.z)};
        }
        else
        {
            out.angular_velocity = {nan, nan, nan};
        }

        out.position_variance = {pos_var_, pos_var_, pos_var_};
        out.orientation_variance = {att_var_, att_var_, att_var_};
        out.reset_counter = reset_counter_;
        out.quality = 100;

        px4_pub_->publish(out);
        last_pub_src_us_ = src_us;
        ++n_pub_;

        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 5000,
                             "Publishing: sample age vs PX4 now = %.1f ms, "
                             "rx latency above min = %.1f ms",
                             (px4_now_us - sample_us) * 1e-3,
                             ((rx_us - src_us) - src_off_.min()) * 1e-3);

        if (pose_pub_)
        {
            publishDebugPose(msg.header, out);
        }
    }

    // Publish exactly the pose sent to PX4 (FRD world / FRD body) for RViz.
    // A static TF <odom frame> -> px4_frame_id (180 deg about X) lets RViz
    // draw it on top of the FAST-LIO odometry.
    void publishDebugPose(const std_msgs::msg::Header &odom_header,
                          const px4_msgs::msg::VehicleOdometry &out)
    {
        if (odom_header.frame_id.empty())
        {
            return;
        }
        if (static_tf_parent_ != odom_header.frame_id)
        {
            geometry_msgs::msg::TransformStamped tf;
            tf.header.stamp = odom_header.stamp;
            tf.header.frame_id = odom_header.frame_id;
            tf.child_frame_id = px4_frame_id_;
            // Yaw about the parent (Z-up) axis, then the 180 deg X flip to FRD.
            const Eigen::Quaterniond q =
                Eigen::AngleAxisd(px4_frame_yaw_rad_, Eigen::Vector3d::UnitZ()) *
                Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitX());
            tf.transform.rotation.w = q.w();
            tf.transform.rotation.x = q.x();
            tf.transform.rotation.y = q.y();
            tf.transform.rotation.z = q.z();
            static_tf_->sendTransform(tf);
            static_tf_parent_ = odom_header.frame_id;
        }

        geometry_msgs::msg::PoseStamped pose;
        pose.header.stamp = odom_header.stamp;
        pose.header.frame_id = px4_frame_id_;
        pose.pose.position.x = out.position[0];
        pose.pose.position.y = out.position[1];
        pose.pose.position.z = out.position[2];
        pose.pose.orientation.w = out.q[0];
        pose.pose.orientation.x = out.q[1];
        pose.pose.orientation.y = out.q[2];
        pose.pose.orientation.z = out.q[3];
        pose_pub_->publish(pose);
    }

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<px4_msgs::msg::VehicleAttitude>::SharedPtr px4_clock_sub_;
    rclcpp::Publisher<px4_msgs::msg::VehicleOdometry>::SharedPtr px4_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_;
    std::string px4_frame_id_{"px4_frd"};
    double px4_frame_yaw_rad_{M_PI / 2.0};
    std::string static_tf_parent_;

    rclcpp::TimerBase::SharedPtr status_timer_;

    // Diagnostics counters (reset every status print).
    uint32_t n_odom_{0}, n_px4_{0}, n_pub_{0};
    uint32_t n_zero_stamp_{0}, n_dup_{0}, n_no_clock_{0}, n_bad_pose_{0};

    WindowedMin px4_off_;
    WindowedMin src_off_;

    int64_t min_period_us_{0};
    int64_t px4_timeout_us_{1000000};
    int64_t stamp_jump_us_{1000000};
    int64_t last_px4_us_{0};
    int64_t last_now_us_{0};
    int64_t last_src_us_{0};
    int64_t last_pub_src_us_{0};

    float pos_var_{0.0025f};
    float att_var_{0.0025f};
    float vel_var_{0.01f};

    bool send_velocity_{true};
    bool publish_debug_pose_{true};
    uint8_t reset_counter_{0};
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Px4OdometryBridge>());
    rclcpp::shutdown();
    return 0;
}
