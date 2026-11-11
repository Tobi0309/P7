#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>

class GoalConverter : public rclcpp::Node
{
public:
    GoalConverter() : Node("goal_converter")
    {
        odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
            "/Odometry_base_link", 10,
            [this](nav_msgs::msg::Odometry::ConstSharedPtr msg)
            {
                altitude_ = msg->pose.pose.position.z;
                odom_frame_ = msg->header.frame_id;
                has_odom_ = true;
            });

        goal_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
            "/goal", 10,
            [this](geometry_msgs::msg::PoseStamped::ConstSharedPtr msg)
            {
                if (!has_odom_)
                {
                    RCLCPP_WARN(get_logger(),
                                "No odometry received yet; ignoring goal");
                    return;
                }

                if (msg->header.frame_id != odom_frame_)
                {
                    RCLCPP_WARN(
                        get_logger(),
                        "Goal frame '%s' differs from odometry frame '%s'; ignoring goal",
                        msg->header.frame_id.c_str(),
                        odom_frame_.c_str());
                    return;
                }

                auto goal = *msg;
                goal.pose.position.z = altitude_;

                goal_pub_->publish(goal);

                RCLCPP_INFO(
                    get_logger(),
                    "Goal forwarded: x=%.2f y=%.2f z=%.2f",
                    goal.pose.position.x,
                    goal.pose.position.y,
                    goal.pose.position.z);
            });

        goal_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(
            "/move_base_simple/goal", 10);
    }

private:
    double altitude_ = 0.0;
    bool has_odom_ = false;
    std::string odom_frame_;

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pub_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GoalConverter>());
    rclcpp::shutdown();
    return 0;
}
