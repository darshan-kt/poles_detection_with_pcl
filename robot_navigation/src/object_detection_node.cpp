#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav2_msgs/srv/manage_lifecycle_nodes.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "nav2_msgs/srv/clear_entire_costmap.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "action_msgs/msg/goal_status.hpp"


class LidarObjectDetector : public rclcpp::Node
{
public:
    LidarObjectDetector()
        : Node("lidar_object_detector"), obstacle_detected_(false), current_goal_(nullptr)
    {
        // Subscription to the LiDAR data
        scan_subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10, std::bind(&LidarObjectDetector::scanCallback, this, std::placeholders::_1));

        // Subscription to track the current goal pose
        goal_subscription_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/goal_pose", 10, std::bind(&LidarObjectDetector::goalCallback, this, std::placeholders::_1));

        // Action client to control navigation goals
        navigation_action_client_ = rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(this, "navigate_to_pose");

        RCLCPP_INFO(this->get_logger(), "Lidar Object Detector Node has been started.");
    }

private:
    void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        bool obstacle_detected = false;

        // Check if there are obstacles within 2 meters
        for (const auto &range : msg->ranges)
        {
            if (range > msg->range_min && range < 2.0)
            {
                obstacle_detected = true;
                break;
            }
        }

        if (obstacle_detected && !obstacle_detected_)
        {
            RCLCPP_WARN(this->get_logger(), "Obstacle detected within 2.0m range! Pausing Nav2.");
            manageNav2Lifecycle("pause");
            cancelCurrentGoal(); // Cancel the current goal explicitly
            obstacle_detected_ = true;
        }
        else if (!obstacle_detected && obstacle_detected_)
        {
            RCLCPP_INFO(this->get_logger(), "Obstacle cleared! Resuming Nav2.");
            manageNav2Lifecycle("resume");
            reissueGoal();  // Reissue the goal after Nav2 resumes
            obstacle_detected_ = false;
        }
    }

    void goalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        // Store the current goal pose to reissue it later if needed
        current_goal_ = std::make_shared<geometry_msgs::msg::PoseStamped>(*msg);
        RCLCPP_INFO(this->get_logger(), "Received a new goal pose.");
    }

    void cancelCurrentGoal()
    {
        if (!navigation_action_client_ || !navigation_action_client_->wait_for_action_server(std::chrono::seconds(5)))
        {
            RCLCPP_ERROR(this->get_logger(), "Navigation action server not available when trying to cancel goal.");
            return;
        }

        RCLCPP_INFO(this->get_logger(), "Cancelling the current navigation goal.");
        auto cancel_future = navigation_action_client_->async_cancel_all_goals();

        try
        {
            auto cancel_response = cancel_future.get();
            if (cancel_response->goals_canceling.size() > 0)
            {
                RCLCPP_INFO(this->get_logger(), "Goal successfully canceled.");
            }
            else
            {
                RCLCPP_WARN(this->get_logger(), "Failed to cancel the goal or no goal was in progress.");
            }
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "Error cancelling the goal: %s", e.what());
        }
    }

    void reissueGoal()
    {
        if (!current_goal_)
        {
            RCLCPP_WARN(this->get_logger(), "No goal to reissue.");
            return;
        }

        if (!navigation_action_client_ || !navigation_action_client_->wait_for_action_server(std::chrono::seconds(5)))
        {
            RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting.");
            return;
        }

        auto goal_msg = nav2_msgs::action::NavigateToPose::Goal();
        goal_msg.pose = *current_goal_;

        RCLCPP_INFO(this->get_logger(), "Reissuing goal to continue navigation.");

        auto send_goal_options = rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SendGoalOptions();
        send_goal_options.goal_response_callback = [](auto) { /* Handle goal response */ };
        send_goal_options.result_callback = [](auto) { /* Handle result */ };

        navigation_action_client_->async_send_goal(goal_msg, send_goal_options);
    }


    void manageNav2Lifecycle(const std::string &command)
        {
            auto request = std::make_shared<nav2_msgs::srv::ManageLifecycleNodes::Request>();

            if (command == "pause")
            {
                request->command = nav2_msgs::srv::ManageLifecycleNodes::Request::PAUSE;
            }
            else if (command == "resume")
            {
                request->command = nav2_msgs::srv::ManageLifecycleNodes::Request::RESUME;
            }
            else
            {
                RCLCPP_ERROR(this->get_logger(), "Invalid command for Nav2 lifecycle management.");
                return;
            }

            if (!lifecycle_client_->wait_for_service(std::chrono::seconds(1)))
            {
                RCLCPP_WARN(this->get_logger(), "Nav2 lifecycle management service not available, trying again...");
                return; // Exit if the service is not available
            }

            auto future = lifecycle_client_->async_send_request(request,
                [this, command](rclcpp::Client<nav2_msgs::srv::ManageLifecycleNodes>::SharedFuture future) {
                    try
                    {
                        auto response = future.get();
                        if (response->success)
                        {
                            RCLCPP_INFO(this->get_logger(), "Nav2 Lifecycle command '%s' executed successfully.", command.c_str());
                        }
                        else
                        {
                            RCLCPP_ERROR(this->get_logger(), "Failed to execute Nav2 Lifecycle command: %s", command.c_str());
                        }
                    }
                    catch (const std::exception &e)
                    {
                        RCLCPP_ERROR(this->get_logger(), "Service call failed: %s", e.what());
                    }
                }
            );
        }

    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_subscription_;
    rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr navigation_action_client_;
    // geometry_msgs::msg::PoseStamped::SharedPtr current_goal_;
    std::shared_ptr<geometry_msgs::msg::PoseStamped> current_goal_;

    rclcpp::Client<nav2_msgs::srv::ClearEntireCostmap>::SharedPtr costmap_clear_client_;

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_subscription_;
    rclcpp::Client<nav2_msgs::srv::ManageLifecycleNodes>::SharedPtr lifecycle_client_;
    bool obstacle_detected_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LidarObjectDetector>());
    rclcpp::shutdown();
    return 0;
}
