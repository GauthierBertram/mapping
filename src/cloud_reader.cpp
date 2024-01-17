#include <rclcpp/rclcpp.hpp>
#include <rclcpp/node.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/common/centroid.h>
#include <geometry_msgs/msg/pose_array.hpp>
#include <iostream>

class PointCloudProcessor : public rclcpp::Node {
public:
    PointCloudProcessor() : Node("point_cloud_processor") {
        subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/wamv/sensors/lidars/lidar_wamv_sensor/points", 10, std::bind(&PointCloudProcessor::pointCloudCallback, this, std::placeholders::_1));

        // Create a publisher for PoseArray
        centroid_publisher_ = this->create_publisher<geometry_msgs::msg::PoseArray>("cluster_centroids", 10);
    }

private:
    void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(*msg, *cloud);

        // Filter based on z-coordinate
        filterPointCloud(cloud);

        // Extract clusters
        std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> clusters;
        extractClusters(cloud, clusters);

        // Calculate centroids and publish as PoseArray
        calculateCentroidsAndPublish(clusters);
    }

    void filterPointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud) {
        pcl::PassThrough<pcl::PointXYZ> pass;
        pass.setInputCloud(cloud);
        pass.setFilterFieldName("z");
        pass.setFilterLimits(-0.7, FLT_MAX); 
        pass.filter(*cloud);
    }

    void extractClusters(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
                         std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>& clusters) {
        pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
        ec.setClusterTolerance(10.);  
        ec.setMinClusterSize(2);        
        ec.setInputCloud(cloud);

        std::vector<pcl::PointIndices> cluster_indices;
        ec.extract(cluster_indices);

        for (const auto& indices : cluster_indices) {
            pcl::PointCloud<pcl::PointXYZ>::Ptr cluster(new pcl::PointCloud<pcl::PointXYZ>);
            for (const auto& index : indices.indices) {
                cluster->points.push_back(cloud->points[index]);
            }
            clusters.push_back(cluster);
        }
    }

    void calculateCentroidsAndPublish(const std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>& clusters) {
        geometry_msgs::msg::PoseArray pose_array;

        for (size_t i = 0; i < clusters.size(); ++i) {
            Eigen::Vector4f centroid;
            pcl::compute3DCentroid(*clusters[i], centroid);

            geometry_msgs::msg::Pose pose;
            pose.position.x = centroid[0];
            pose.position.y = centroid[1];
            pose.position.z = centroid[2];

            pose_array.poses.push_back(pose);

            
            std::cout << "Centroide del cluster " << i + 1 << ": "
                    << "x = " << centroid[0] << ", y = " << centroid[1] << ", z = " << centroid[2] << std::endl;
        }

        centroid_publisher_->publish(pose_array);
    }

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr centroid_publisher_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PointCloudProcessor>());
    rclcpp::shutdown();
    return 0;
}