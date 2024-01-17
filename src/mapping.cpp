#include <rclcpp/rclcpp.hpp>
#include <rclcpp/node.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <iostream>
#include <ctime>

class PointCloudProcessor : public rclcpp::Node {
public:
    PointCloudProcessor() : Node("onfailamap") {
        subscription_ = this->create_subscription<geometry_msgs::msg::PoseArray>(
            "cluster_centroids", 10, std::bind(&PointCloudProcessor::poseArrayCallback, this, std::placeholders::_1));

        // Create a publisher for PoseArray
        MapPublisher_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("lamap", 10);
    }

private:
    void poseArrayCallback(const geometry_msgs::msg::PoseArray::SharedPtr pose_array_msg)
      {
        // Traitez les données de la PoseArray ici
        nav_msgs::msg::OccupancyGrid map;
        geometry_msgs::msg::Pose origin;
        origin.position.x = 500;
        origin.position.y = 500;
        map.header.stamp = this->now();
        map.header.frame_id = "map";
        map.info.map_load_time = this->now();
        map.info.width = 1000;
        map.info.height = 1000;
        map.info.resolution = 1;

        map.info.origin = origin;
        map.data.resize(map.info.width*map.info.height,-1);

        for (const auto& pose : pose_array_msg->poses) {
          // Accédez aux champs de la pose (position et orientation)
          auto position = pose.position;
          auto orientation = pose.orientation;

          for ( unsigned int i = 0; i < map.data.size();i++)
          {
              map.data[i] = 0;
          }



          // Faites quelque chose avec les données de la pose...
        }
        MapPublisher_->publish(map);
      }

      rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr subscription_;
      rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr MapPublisher_;
};


int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PointCloudProcessor>());
    rclcpp::shutdown();
    return 0;
}
