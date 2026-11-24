#include <chrono>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <dsp_interfaces/srv/set_goal.hpp>

// Forwards goals (e.g. from RViz) to the DSP global planner through its set_goal service,
// replacing the goal height with a fixed altitude (goal_height). x, y, orientation and frame
// are kept; DSP transforms the goal into its own frame and forwards the orientation to
// ego-planner.
//
// RViz2's "2D Goal Pose" tool publishes on /goal_pose (RELIABLE, VOLATILE, depth 5).
class GoalConverterDsp : public rclcpp::Node
{
public:
    using SetGoal = dsp_interfaces::srv::SetGoal;
    using PoseStamped = geometry_msgs::msg::PoseStamped;

    GoalConverterDsp() : Node("goal_converter_dsp")
    {
        declare_parameter("goal_height", 2.0);
        const auto goal_topic = declare_parameter("goal_topic", std::string("/goal_pose"));
        const auto service_name = declare_parameter("set_goal_service", std::string("/dsp/set_goal"));
        // "reliable" matches RViz's default. "best_effort" also matches best-effort publishers
        // (a reliable subscriber does not), at the risk of losing a goal.
        const auto reliability = declare_parameter("goal_qos_reliability", std::string("reliable"));
        const auto depth = declare_parameter("goal_qos_depth", 10);

        rclcpp::QoS qos{rclcpp::KeepLast(static_cast<size_t>(depth))};
        qos.durability_volatile();  // matches both volatile and transient-local publishers
        if (reliability == "best_effort")
        {
            qos.best_effort();
        }
        else
        {
            if (reliability != "reliable")
            {
                RCLCPP_WARN(get_logger(),
                            "Unknown goal_qos_reliability '%s', using 'reliable'",
                            reliability.c_str());
            }
            qos.reliable();
        }

        client_ = create_client<SetGoal>(service_name);

        goal_sub_ = create_subscription<PoseStamped>(
            goal_topic, qos,
            [this](PoseStamped::ConstSharedPtr msg) { goalCallback(msg); });

        RCLCPP_INFO(get_logger(),
                    "Listening for goals on '%s' (QoS %s, depth %ld), forwarding to '%s' "
                    "at height %.2f m",
                    goal_sub_->get_topic_name(), reliability.c_str(), depth,
                    client_->get_service_name(), get_parameter("goal_height").as_double());

        // Wall timer so the diagnostics also run before /clock arrives in simulation.
        status_timer_ = create_wall_timer(std::chrono::seconds(5), [this]() { logStatus(); });
    }

private:
    void logStatus()
    {
        const size_t publishers = goal_sub_->get_publisher_count();
        if (publishers == 0)
        {
            RCLCPP_WARN(get_logger(),
                        "No publishers on '%s' - is the RViz goal tool publishing to this "
                        "topic? (set parameter goal_topic)",
                        goal_sub_->get_topic_name());
        }
        else if (publishers != last_publisher_count_)
        {
            RCLCPP_INFO(get_logger(), "%zu publisher(s) on '%s'", publishers,
                        goal_sub_->get_topic_name());
        }
        last_publisher_count_ = publishers;

        const bool ready = client_->service_is_ready();
        if (!ready)
        {
            RCLCPP_WARN(get_logger(), "Service '%s' not available (is dsp running?)",
                        client_->get_service_name());
        }
        else if (!service_was_ready_)
        {
            RCLCPP_INFO(get_logger(), "Service '%s' is available", client_->get_service_name());
        }
        service_was_ready_ = ready;
    }

    void goalCallback(const PoseStamped::ConstSharedPtr &msg)
    {
        ++goals_received_;
        RCLCPP_INFO(get_logger(),
                    "Goal #%zu received on '%s': x=%.2f y=%.2f z=%.2f (frame '%s', stamp %d.%09u)",
                    goals_received_, goal_sub_->get_topic_name(),
                    msg->pose.position.x, msg->pose.position.y, msg->pose.position.z,
                    msg->header.frame_id.c_str(), msg->header.stamp.sec, msg->header.stamp.nanosec);

        if (msg->header.frame_id.empty())
        {
            RCLCPP_WARN(get_logger(), "Goal has an empty frame_id; DSP may not transform it");
        }

        if (!client_->service_is_ready())
        {
            RCLCPP_WARN(get_logger(),
                        "Service '%s' not available (is dsp running?); ignoring goal #%zu",
                        client_->get_service_name(), goals_received_);
            return;
        }

        auto request = std::make_shared<SetGoal::Request>();
        request->goal = *msg;
        request->goal.pose.position.z = get_parameter("goal_height").as_double();

        RCLCPP_INFO(get_logger(),
                    "Sending goal #%zu to DSP: x=%.2f y=%.2f z=%.2f (frame '%s')",
                    goals_received_,
                    request->goal.pose.position.x,
                    request->goal.pose.position.y,
                    request->goal.pose.position.z,
                    request->goal.header.frame_id.c_str());

        const size_t id = goals_received_;
        client_->async_send_request(
            request,
            [this, id](rclcpp::Client<SetGoal>::SharedFuture future)
            {
                try
                {
                    const auto response = future.get();
                    if (response->success)
                    {
                        RCLCPP_INFO(get_logger(), "DSP accepted goal #%zu: %s", id,
                                    response->message.c_str());
                    }
                    else
                    {
                        RCLCPP_WARN(get_logger(), "DSP rejected goal #%zu: %s", id,
                                    response->message.c_str());
                    }
                }
                catch (const std::exception &e)
                {
                    RCLCPP_ERROR(get_logger(), "set_goal call for goal #%zu failed: %s", id,
                                 e.what());
                }
            });
    }

    rclcpp::Client<SetGoal>::SharedPtr client_;
    rclcpp::Subscription<PoseStamped>::SharedPtr goal_sub_;
    rclcpp::TimerBase::SharedPtr status_timer_;
    size_t goals_received_{0};
    size_t last_publisher_count_{0};
    bool service_was_ready_{false};
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GoalConverterDsp>());
    rclcpp::shutdown();
    return 0;
}
