#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/centroid.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/search/kdtree.h>

class PoleCentroidNode : public rclcpp::Node
{
public:
    PoleCentroidNode() : Node("pole_centroid_node")
    {
        // Subscriber to the poles point cloud topic
        poles_subscriber_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/only_poles", 10,
            std::bind(&PoleCentroidNode::polesCallback, this, std::placeholders::_1));

        // Publisher for the calculated mid-point as a marker for visualization in RViz
        marker_publisher_ = this->create_publisher<visualization_msgs::msg::Marker>("/mid_point_marker", 10);

        // Publisher for the straight line path as a marker for visualization in RViz
        line_path_publisher_ = this->create_publisher<visualization_msgs::msg::Marker>("/bezier_path_tracking", 10);
    }

private:
    void polesCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        // Convert the ROS2 PointCloud2 message to PCL format
        pcl::PointCloud<pcl::PointXYZ>::Ptr poles_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(*msg, *poles_cloud);

        if (poles_cloud->empty()) {
            RCLCPP_WARN(this->get_logger(), "Received empty poles cloud, skipping processing.");
            return;
        }

        // Step 1: Cluster the points to separate individual poles
        std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> pole_clusters = extractClusters(poles_cloud);

        if (pole_clusters.size() < 2) {
            RCLCPP_WARN(this->get_logger(), "Less than two poles detected, cannot compute midpoint.");
            return;
        }

        // Step 2: Calculate the centroids of the immediate front pair of poles
        pcl::PointXYZ right_pole_centroid, left_pole_centroid;
        identifyImmediateFrontPoles(pole_clusters, right_pole_centroid, left_pole_centroid);

        // Step 3: Calculate the aligned midpoint between the right and left pole centroids
        geometry_msgs::msg::Point midpoint;
        midpoint.x = (right_pole_centroid.x + left_pole_centroid.x) / 2.0;
        midpoint.y = (right_pole_centroid.y + left_pole_centroid.y) / 2.0;
        midpoint.z = (right_pole_centroid.z + left_pole_centroid.z) / 2.0;

        // Ensure the midpoint is at least 0.1m in front of the robot
        if (midpoint.x < 0.1) {
            RCLCPP_WARN(this->get_logger(), "Calculated midpoint is not in front of the robot. Adjusting...");
            midpoint.x = 0.1;  // Force the midpoint to be at least 0.1m in front of the robot
        }

        // Step 4: Publish the midpoint as a marker for visualization in RViz
        if (midpoint.y != 0.0)
        {
            RCLCPP_WARN(this->get_logger(), "Can't able to find mid between pair of pole");
            midpoint.y = 0.0;
            publishMidPointMarker(midpoint);
        }
        else
        {
            publishMidPointMarker(midpoint);
        }
        

        // Step 5: Generate and publish a straight line path from the robot's position to the midpoint
        pcl::PointXYZ robot_position(0.0, 0.0, 0.0); // Assuming the robot is at the origin
        publishStraightLine(robot_position, midpoint);
    }

    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> extractClusters(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
    {
        std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> clusters;
        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
        tree->setInputCloud(cloud);

        std::vector<pcl::PointIndices> cluster_indices;
        pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
        ec.setClusterTolerance(0.3);  // Adjust cluster tolerance as needed
        ec.setMinClusterSize(30);     // Minimum number of points to consider a cluster as a pole
        ec.setMaxClusterSize(10000);  // Maximum number of points in a cluster
        ec.setSearchMethod(tree);
        ec.setInputCloud(cloud);
        ec.extract(cluster_indices);

        for (const auto &indices : cluster_indices) {
            pcl::PointCloud<pcl::PointXYZ>::Ptr cluster(new pcl::PointCloud<pcl::PointXYZ>);
            for (const auto &index : indices.indices) {
                cluster->points.push_back(cloud->points[index]);
            }
            clusters.push_back(cluster);
        }

        return clusters;
    }

    bool identifyImmediateFrontPoles(const std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> &clusters,
                                     pcl::PointXYZ &right_pole_centroid, pcl::PointXYZ &left_pole_centroid)
    {
        float robot_position_x = 0.0;  // Assuming the robot is at the origin on the x-axis

        float min_right_distance = std::numeric_limits<float>::max();
        float min_left_distance = std::numeric_limits<float>::max();

        bool right_pole_found = false;
        bool left_pole_found = false;

        for (const auto &cluster : clusters) {
            Eigen::Vector4f centroid;
            pcl::compute3DCentroid(*cluster, centroid);
            pcl::PointXYZ cluster_centroid(centroid[0], centroid[1], centroid[2]);

            // Ensure the pole is in front of the robot (x > 0.1m)
            if (cluster_centroid.x > 0.05) {
                float distance_to_robot = std::hypot(cluster_centroid.x, cluster_centroid.y);

                // Check whether the pole is on the right or left of the robot and if it is the closest one in that direction
                if (cluster_centroid.y > 0 && distance_to_robot < min_right_distance) {
                    min_right_distance = distance_to_robot;
                    right_pole_centroid = cluster_centroid;
                    right_pole_found = true;
                } else if (cluster_centroid.y < 0 && distance_to_robot < min_left_distance) {
                    min_left_distance = distance_to_robot;
                    left_pole_centroid = cluster_centroid;
                    left_pole_found = true;
                }
            }
        }

        return right_pole_found && left_pole_found;
    }

    void publishStraightLine(const pcl::PointXYZ &start_point, const geometry_msgs::msg::Point &end_point)
    {
        visualization_msgs::msg::Marker line_strip;
        line_strip.header.frame_id = "base_link"; // Change to your appropriate frame
        line_strip.header.stamp = this->get_clock()->now();
        line_strip.ns = "straight_line_path";
        line_strip.id = 1;
        line_strip.type = visualization_msgs::msg::Marker::LINE_STRIP;
        line_strip.action = visualization_msgs::msg::Marker::ADD;

        // Set the scale of the line
        line_strip.scale.x = 0.05; // Line width

        // Set the color of the line
        line_strip.color.r = 0.0f;
        line_strip.color.g = 1.0f; // Green color
        line_strip.color.b = 0.0f;
        line_strip.color.a = 1.0; // Fully opaque

        // Add the start and end points to the line strip
        geometry_msgs::msg::Point start_geometry;
        start_geometry.x = start_point.x;
        start_geometry.y = start_point.y;
        start_geometry.z = start_point.z;

        line_strip.points.push_back(start_geometry);
        line_strip.points.push_back(end_point);

        line_path_publisher_->publish(line_strip);
        RCLCPP_INFO(this->get_logger(), "Published straight line path between robot and midpoint.");
    }

    void publishMidPointMarker(const geometry_msgs::msg::Point &midpoint)
    {
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "base_link"; // Change to your appropriate frame
        marker.header.stamp = this->get_clock()->now();
        marker.ns = "mid_point";
        marker.id = 0;
        marker.type = visualization_msgs::msg::Marker::SPHERE; // Use a sphere to represent the midpoint
        marker.action = visualization_msgs::msg::Marker::ADD;

        // Set the position of the marker
        marker.pose.position.x = midpoint.x;
        marker.pose.position.y = midpoint.y;
        marker.pose.position.z = midpoint.z;
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;

        // Set the scale of the marker (size of the sphere)
        marker.scale.x = 0.2; // Diameter of the sphere
        marker.scale.y = 0.2;
        marker.scale.z = 0.2;

        // Set the color of the marker
        marker.color.r = 1.0f; // Red color
        marker.color.g = 0.0f;
        marker.color.b = 0.0f;
        marker.color.a = 1.0; // Fully opaque

        marker.lifetime = rclcpp::Duration::from_seconds(0); // 0 means marker stays forever

        marker_publisher_->publish(marker);
        RCLCPP_INFO(this->get_logger(), "Published midpoint marker at [x: %.2f, y: %.2f, z: %.2f]", midpoint.x, midpoint.y, midpoint.z);
    }

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr poles_subscriber_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_publisher_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr line_path_publisher_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PoleCentroidNode>());
    rclcpp::shutdown();
    return 0;
}
