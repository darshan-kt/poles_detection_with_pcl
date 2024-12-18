#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/twist.hpp"

class LidarObjectDetector : public rclcpp::Node
{
public:
    LidarObjectDetector()
        : Node("lidar_object_detector"), obstacle_detected_(false)
    {
        // Subscription to the LiDAR data
        scan_subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10, std::bind(&LidarObjectDetector::scanCallback, this, std::placeholders::_1));

        // Subscription to the velocity commands (cmd_vel) from Nav2
        velocity_subscription_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel_original", 10, std::bind(&LidarObjectDetector::velocityCallback, this, std::placeholders::_1));

        // Publisher to override velocity commands when an obstacle is detected
        velocity_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

        RCLCPP_INFO(this->get_logger(), "Lidar Object Detector Node has been started.");
    }

private:
    void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        bool obstacle_detected = false;

        // Check all ranges in the laser scan to see if any obstacle is within 2 meters
        for (const auto &range : msg->ranges)
        {
            if (!std::isnan(range) && !std::isinf(range) && range < 2.0)
            {
                obstacle_in_front = true;
                break;  // Stop checking further if an obstacle is already detected
            }
        }

        // Update obstacle status and handle velocity control
        if (obstacle_in_front && !obstacle_detected_)
        {
            RCLCPP_WARN(this->get_logger(), "Obstacle detected within 2.0m range! Stopping the robot.");
            stopRobot();
            obstacle_detected_ = true;
        }
        else if (!obstacle_in_front && obstacle_detected_)
        {
            RCLCPP_INFO(this->get_logger(), "Obstacle cleared! Resuming robot movement.");
            obstacle_detected_ = false;
            resumeRobot();
        }
    }

    void velocityCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        // Only publish velocity commands if there is no obstacle detected
        if (!obstacle_detected_)
        {
            velocity_publisher_->publish(*msg);
        }
    }

    void stopRobot()
    {
        // Publish a zero velocity command to stop the robot
        auto stop_msg = geometry_msgs::msg::Twist();
        velocity_publisher_->publish(stop_msg);
    }

    void resumeRobot()
    {
        // We don't need to explicitly resume here since the original cmd_vel commands
        // are handled in the velocityCallback method when no obstacle is detected.
        RCLCPP_INFO(this->get_logger(), "Robot is ready to resume navigation.");
    }

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_subscription_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr velocity_subscription_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr velocity_publisher_;
    bool obstacle_detected_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LidarObjectDetector>());
    rclcpp::shutdown();
    return 0;
}
