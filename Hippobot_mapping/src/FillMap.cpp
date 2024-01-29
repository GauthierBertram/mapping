#include <rclcpp/rclcpp.hpp>
#include <rclcpp/node.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <iostream>
#include <ctime>
#include <map_msgs/msg/occupancy_grid_update.hpp>
#include <my_robot_interfaces/msg/centroid_with_radius.hpp>
#include <my_robot_interfaces/msg/centroid_array_with_radius.hpp>




class Mapping : public rclcpp::Node {
public:
    Mapping() :
        Node("onfailamap"){

            subscription_ = this->create_subscription<my_robot_interfaces::msg::CentroidArrayWithRadius>(
                "centroids", 10, std::bind(&Mapping::UpdateCentroids, this, std::placeholders::_1));
            subscription2_= this->create_subscription<geometry_msgs::msg::Pose>(
                        "boat_pose", 10, std::bind(&Mapping::BoatPoseCallback, this, std::placeholders::_1));
            MapPublisher_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("WorldMap", 10);
            MapUpdatePublisher_ = this->create_publisher<map_msgs::msg::OccupancyGridUpdate>("WorldMap_Updates", 10);
            MapCreate();

            // Create a publisher for PoseArray

            set_parameter(rclcpp::Parameter("use_sim_time",true));
                            }


private:
    void MapCreate()
    {
        map_.header.frame_id = "world";
        map_.info.width = 1000;
        map_.info.height = 1000;
        map_.info.resolution = 1;
        map_.info.origin.position.x = -500;
        map_.info.origin.position.y = -500;

        map_.data.resize(map_.info.width*map_.info.height,0);

        MapPublisher_->publish(map_);
    }


    void BoatPoseCallback(const geometry_msgs::msg::Pose::SharedPtr msg ){
        boat_X = msg->position.x;
        boat_Y = msg->position.y;
    }



    void UpdateCentroids(const my_robot_interfaces::msg::CentroidArrayWithRadius::SharedPtr CentroidsWithRadius)
      {

        std::cout << "traitement centroides";
        for (const my_robot_interfaces::msg::CentroidWithRadius& centroid : CentroidsWithRadius->centroids)
        {
          auto center = centroid.position;
          auto radius = centroid.radius;
          int x = (int)center.x;
          int y = (int)center.y;
          int r = (int)radius;
          UpdateMap(x,y,r,1);
        }
        UpdateBoat();
      }



    void UpdateBoat()
      {
        UpdateMap(oldposx,oldposy,1,0);
        oldposx = (int)boat_X;
        oldposy = (int)boat_Y,
        UpdateMap(oldposx,oldposy,1,-1);
      }



    void UpdateMap(const int x, const int y, const int r, const int val)
      {

          map_msgs::msg::OccupancyGridUpdate UpdateMap;
          UpdateMap.header.frame_id = "world";
          UpdateMap.x = 500+x-r;
          UpdateMap.y = 500+y-r;
          UpdateMap.width = 2*r+1;
          UpdateMap.height = 2*r+1;
          UpdateMap.data.resize(UpdateMap.width*UpdateMap.height,val);

          MapUpdatePublisher_->publish(UpdateMap);
      }




      rclcpp::Subscription<my_robot_interfaces::msg::CentroidArrayWithRadius>::SharedPtr subscription_;
      rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr subscription2_;
      rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr MapPublisher_;
      rclcpp::Publisher<map_msgs::msg::OccupancyGridUpdate>::SharedPtr MapUpdatePublisher_;
      nav_msgs::msg::OccupancyGrid map_;

      int oldposx = 500;
      int oldposy = 500;
      int boat_X = 500;
      int boat_Y = 500;
};




int main(int argc, char** argv) {
    std::cout << "cccccccccccc";
    rclcpp::init(argc, argv);
    std::cout << "cccccccccccc";
    rclcpp::spin(std::make_shared<Mapping>());
    rclcpp::shutdown();
    return 0;
}