#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/search/kdtree.h>

class LidarPoleDetectionNode : public rclcpp::Node
{
public:
    LidarPoleDetectionNode() : Node("lidar_pole_detection_node")
    {
        // Subscriber to the raw LiDAR point cloud data
        pointcloud_subscriber_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/velodyne_points", 10,
            std::bind(&LidarPoleDetectionNode::pointcloudCallback, this, std::placeholders::_1));

        // Publisher for detected poles
        poles_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/only_poles", 10);
    }

private:
    void pointcloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        // Convert the ROS2 PointCloud2 message to PCL format
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(*msg, *cloud);

        // Step : Detect poles from the filtered point cloud
        pcl::PointCloud<pcl::PointXYZ>::Ptr poles_cloud = detectPoles(cloud, 0.15, 0.8);

        if (poles_cloud->empty()) {
            RCLCPP_WARN(this->get_logger(), "No poles detected in the point cloud.");
            return;
        }

        // Convert the detected poles to ROS2 format and publish
        sensor_msgs::msg::PointCloud2 poles_msg;
        pcl::toROSMsg(*poles_cloud, poles_msg);
        poles_msg.header = msg->header; // Preserve the original header
        poles_publisher_->publish(poles_msg);
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr filterGroundAndBelowLidar(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, float lidar_height)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);

        // Check if the cloud is empty before filtering
        if (cloud->empty()) {
            RCLCPP_WARN(this->get_logger(), "Cannot filter an empty cloud.");
            return cloud_filtered;  // Return an empty cloud
        }

        // Pass-through filter to remove points below the LiDAR mounting height
        pcl::PassThrough<pcl::PointXYZ> pass;
        pass.setInputCloud(cloud);
        pass.setFilterFieldName("z");
        pass.setFilterLimits(lidar_height, std::numeric_limits<float>::max());
        pass.filter(*cloud_filtered);

        if (cloud_filtered->empty()) {
            RCLCPP_WARN(this->get_logger(), "No points remaining after height filtering.");
            return cloud_filtered;  // Return an empty cloud
        }

        // RANSAC plane segmentation to remove the ground plane
        pcl::SACSegmentation<pcl::PointXYZ> seg;
        pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
        pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
        seg.setOptimizeCoefficients(true);
        seg.setModelType(pcl::SACMODEL_PLANE);
        seg.setMethodType(pcl::SAC_RANSAC);
        seg.setDistanceThreshold(0.01);
        seg.setInputCloud(cloud_filtered);
        seg.segment(*inliers, *coefficients);

        if (inliers->indices.empty()) {
            RCLCPP_WARN(this->get_logger(), "RANSAC did not find any planar model.");
            return cloud_filtered;  // Return the filtered cloud as is
        }

        // Extract the non-ground points
        pcl::ExtractIndices<pcl::PointXYZ> extract;
        pcl::PointCloud<pcl::PointXYZ>::Ptr ground_removed_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        extract.setInputCloud(cloud_filtered);
        extract.setIndices(inliers);
        extract.setNegative(true); // True means remove the ground
        extract.filter(*ground_removed_cloud);

        return ground_removed_cloud;
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr detectPoles(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, float min_height, float max_height)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr poles_cloud(new pcl::PointCloud<pcl::PointXYZ>);

        // Check if the input cloud is empty
        if (cloud->empty()) {
            RCLCPP_WARN(this->get_logger(), "Input cloud for pole detection is empty.");
            return poles_cloud;  // Return an empty cloud
        }

        // Step 1: Height filter to focus only on vertical structures within the given range
        pcl::PointCloud<pcl::PointXYZ>::Ptr height_filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PassThrough<pcl::PointXYZ> pass;
        pass.setInputCloud(cloud);
        pass.setFilterFieldName("z");
        pass.setFilterLimits(min_height, max_height);  // Filter points between 0.1m and 0.5m
        pass.filter(*height_filtered_cloud);

        if (height_filtered_cloud->empty()) {
            RCLCPP_WARN(this->get_logger(), "No points remaining after height filtering for pole detection.");
            return poles_cloud;  // Return an empty cloud
        }

        // Step 2: Perform Euclidean Cluster Extraction to find clusters of points representing poles
        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
        tree->setInputCloud(height_filtered_cloud);

        std::vector<pcl::PointIndices> cluster_indices;
        pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
        ec.setClusterTolerance(0.2);  // Adjust this value to control cluster separation distance
        ec.setMinClusterSize(50);     // Minimum number of points to form a valid cluster  [Limititng the cloud cluster size w.r.t poles, it reject any object below this or after this size]
        ec.setMaxClusterSize(250);  // Maximum number of points in a cluster
        ec.setSearchMethod(tree);
        ec.setInputCloud(height_filtered_cloud);
        ec.extract(cluster_indices);

        if (cluster_indices.empty()) {
            RCLCPP_WARN(this->get_logger(), "No clusters found in the point cloud for pole detection.");
            return poles_cloud;  // Return an empty cloud if no clusters were found
        }

        // Step 3: Analyze clusters to identify pole-like structures based on vertical extent
        for (const auto &indices : cluster_indices) {
            pcl::PointCloud<pcl::PointXYZ>::Ptr cluster(new pcl::PointCloud<pcl::PointXYZ>);
            for (const auto &index : indices.indices) {
                cluster->points.push_back(height_filtered_cloud->points[index]);
            }

            // Calculate the height of the cluster
            float min_z = std::numeric_limits<float>::max();
            float max_z = std::numeric_limits<float>::lowest();
            for (const auto &point : cluster->points) {
                if (point.z < min_z) min_z = point.z;
                if (point.z > max_z) max_z = point.z;
            }
            float cluster_height = max_z - min_z;

            // If the cluster has a significant height, consider it a pole
            if (cluster_height >= min_height && cluster_height <= max_height) {
                *poles_cloud += *cluster;  // Add this cluster to the pole cloud
            }
        }

        if (poles_cloud->empty()) {
            RCLCPP_WARN(this->get_logger(), "No poles detected in the filtered clusters.");
        }

        return poles_cloud;
    }

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_subscriber_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr poles_publisher_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<LidarPoleDetectionNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
