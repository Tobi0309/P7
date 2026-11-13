#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/exceptions.h>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <memory>

class OdometryTransformer : public rclcpp::Node
{
public:
    OdometryTransformer()
        : Node("odometry_transformer"),
          tf_buffer_(this->get_clock()),
          tf_listener_(tf_buffer_)
    {
        odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
            "/Odometry_imu",
            20,
            [this](nav_msgs::msg::Odometry::ConstSharedPtr msg)
            {
                odomCallback(msg);
            });

        odom_pub_ = create_publisher<nav_msgs::msg::Odometry>(
            "/Odometry_base_link", 20);

        RCLCPP_INFO(
            get_logger(),
            "Odometry transformer started: body_imu -> base_link");
    }

private:
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;

    void odomCallback(
        const nav_msgs::msg::Odometry::ConstSharedPtr msg)
    {
        // Retrieve the fixed IMU-to-aircraft transform from TF2.
        geometry_msgs::msg::TransformStamped tf;

        try
        {
            tf = tf_buffer_.lookupTransform(
                "body_imu",
                "base_link",
                tf2::TimePointZero);
        }
        catch (const tf2::TransformException &ex)
        {
            RCLCPP_WARN_THROTTLE(
                get_logger(),
                *get_clock(),
                2000,
                "Waiting for body_imu -> base_link TF: %s",
                ex.what());
            return;
        }

        // IMU pose in world frame
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

        // Fixed transform: IMU frame -> aircraft frame
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

        // Aircraft position in world frame
        const Eigen::Vector3d p_world_base =
            p_world_imu + q_world_imu * r_imu_base;

        // Aircraft orientation in world frame
        const Eigen::Quaterniond q_world_base =
            (q_world_imu * q_imu_base).normalized();

        // IMU linear velocity, expressed in IMU frame
        const Eigen::Vector3d v_imu(
            msg->twist.twist.linear.x,
            msg->twist.twist.linear.y,
            msg->twist.twist.linear.z);

        // Angular velocity, expressed in IMU frame
        const Eigen::Vector3d omega_imu(
            msg->twist.twist.angular.x,
            msg->twist.twist.angular.y,
            msg->twist.twist.angular.z);

        // Correct for velocity caused by the mounting offset.
        const Eigen::Vector3d v_base_imu =
            v_imu + omega_imu.cross(r_imu_base);

        // Convert velocities into aircraft FLU frame
        const Eigen::Vector3d v_base =
            q_imu_base.conjugate() * v_base_imu;

        const Eigen::Vector3d omega_base =
            q_imu_base.conjugate() * omega_imu;

        // Construct aircraft-centered odometry
        nav_msgs::msg::Odometry output;

        output.header = msg->header;
        output.child_frame_id = "base_link";

        output.pose.pose.position.x = p_world_base.x();
        output.pose.pose.position.y = p_world_base.y();
        output.pose.pose.position.z = p_world_base.z();

        output.pose.pose.orientation.w = q_world_base.w();
        output.pose.pose.orientation.x = q_world_base.x();
        output.pose.pose.orientation.y = q_world_base.y();
        output.pose.pose.orientation.z = q_world_base.z();

        output.twist.twist.linear.x = v_base.x();
        output.twist.twist.linear.y = v_base.y();
        output.twist.twist.linear.z = v_base.z();

        output.twist.twist.angular.x = omega_base.x();
        output.twist.twist.angular.y = omega_base.y();
        output.twist.twist.angular.z = omega_base.z();

        // Covariances are not copied because changing the
        // reference point requires transforming them.
        // They remain zero/unset for this initial planner output.

        odom_pub_->publish(output);
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    rclcpp::spin(std::make_shared<OdometryTransformer>());

    rclcpp::shutdown();
    return 0;
}
