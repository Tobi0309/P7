#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>

// Forwards goals (e.g. from RViz) directly to ego-planner, which plans and follows the path by
// itself. The goal height is replaced with goal_height (> 0) or, if goal_height <= 0, with the
// drone's current altitude. x, y and orientation are kept; ego-planner turns to the goal yaw
// after it arrives. The goal must be in the odometry frame (ego-planner ignores frame_id).
class GoalConverterEgo : public rclcpp::Node
{
public:
    GoalConverterEgo() : Node("goal_converter_ego")
    {
        declare_parameter("goal_height", -1.0);
        const auto goal_topic = declare_parameter("goal_topic", std::string("/goal"));
        const auto odometry_topic = declare_parameter("odometry_topic", std::string("/Odometry_base_link"));
        const auto output_topic = declare_parameter("output_topic", std::string("/move_base_simple/goal"));

        odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
            odometry_topic, 10,
            [this](nav_msgs::msg::Odometry::ConstSharedPtr msg)
            {
                altitude_ = msg->pose.pose.position.z;
                odom_frame_ = msg->header.frame_id;
                has_odom_ = true;
            });

        goal_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
            goal_topic, 10,
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
                const double goal_height = get_parameter("goal_height").as_double();
                goal.pose.position.z = goal_height > 0.0 ? goal_height : altitude_;

                goal_pub_->publish(goal);

                RCLCPP_INFO(
                    get_logger(),
                    "Goal forwarded to ego-planner: x=%.2f y=%.2f z=%.2f",
                    goal.pose.position.x,
                    goal.pose.position.y,
                    goal.pose.position.z);
            });

        goal_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(output_topic, 10);
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
    rclcpp::spin(std::make_shared<GoalConverterEgo>());
    rclcpp::shutdown();
    return 0;
}
