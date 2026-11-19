#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <dsp_interfaces/srv/set_goal.hpp>

// Forwards goals (e.g. from RViz) to the DSP global planner through its set_goal service,
// replacing the goal height with a fixed altitude (goal_height). x, y, orientation and frame
// are kept; DSP transforms the goal into its own frame and forwards the orientation to
// ego-planner.
class GoalConverterDsp : public rclcpp::Node
{
public:
    using SetGoal = dsp_interfaces::srv::SetGoal;

    GoalConverterDsp() : Node("goal_converter_dsp")
    {
        declare_parameter("goal_height", 2.0);
        const auto goal_topic = declare_parameter("goal_topic", std::string("/goal"));
        const auto service_name = declare_parameter("set_goal_service", std::string("/dsp/set_goal"));

        client_ = create_client<SetGoal>(service_name);

        goal_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
            goal_topic, 10,
            [this](geometry_msgs::msg::PoseStamped::ConstSharedPtr msg)
            {
                if (!client_->service_is_ready())
                {
                    RCLCPP_WARN(get_logger(),
                                "Service '%s' not available (is dsp running?); ignoring goal",
                                client_->get_service_name());
                    return;
                }

                auto request = std::make_shared<SetGoal::Request>();
                request->goal = *msg;
                request->goal.pose.position.z = get_parameter("goal_height").as_double();

                RCLCPP_INFO(
                    get_logger(),
                    "Sending goal to DSP: x=%.2f y=%.2f z=%.2f (frame '%s')",
                    request->goal.pose.position.x,
                    request->goal.pose.position.y,
                    request->goal.pose.position.z,
                    request->goal.header.frame_id.c_str());

                client_->async_send_request(
                    request,
                    [this](rclcpp::Client<SetGoal>::SharedFuture future)
                    {
                        const auto response = future.get();
                        if (response->success)
                        {
                            RCLCPP_INFO(get_logger(), "DSP accepted goal: %s",
                                        response->message.c_str());
                        }
                        else
                        {
                            RCLCPP_WARN(get_logger(), "DSP rejected goal: %s",
                                        response->message.c_str());
                        }
                    });
            });
    }

private:
    rclcpp::Client<SetGoal>::SharedPtr client_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GoalConverterDsp>());
    rclcpp::shutdown();
    return 0;
}
