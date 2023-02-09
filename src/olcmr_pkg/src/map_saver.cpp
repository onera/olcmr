#include "rclcpp/rclcpp.hpp"
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl/io/ply_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <unistd.h>
#include <iostream>
#include <cstdlib>
#include <signal.h>
#include <std_srvs/srv/empty.hpp>

sensor_msgs::msg::PointCloud2::SharedPtr map_ptr;
sensor_msgs::msg::PointCloud2::SharedPtr modified_map_ptr;

class MapSaver : public rclcpp::Node
{
public:
  MapSaver()
  : Node("map_saver")
  {
      using std::placeholders::_1;
      map_subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
          "map", 10, std::bind(&MapSaver::map_topic_callback, this, _1));
      modified_map_subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
          "modified_map", 10, std::bind(&MapSaver::modified_map_topic_callback, this, _1));
      map_ptr = std::make_shared<sensor_msgs::msg::PointCloud2>();
      modified_map_ptr = std::make_shared<sensor_msgs::msg::PointCloud2>();
  }

private:

  void map_topic_callback(const sensor_msgs::msg::PointCloud2::SharedPtr message) const
  {
    *map_ptr = *message;
  }

  void modified_map_topic_callback(const sensor_msgs::msg::PointCloud2::SharedPtr message) const
  {
    *modified_map_ptr = *message;
  }
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr map_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr modified_map_subscription_;

};

void save_map(const std::shared_ptr<std_srvs::srv::Empty::Request> request, std::shared_ptr<std_srvs::srv::Empty::Response> response){
  std::cout << "save map request received" << std::endl;
  pcl::PointCloud<pcl::PointXYZI> map_pcd;
  pcl::PointCloud<pcl::PointXYZI> modified_map_pcd;
  sensor_msgs::msg::PointCloud2 map = *(map_ptr.get());
  sensor_msgs::msg::PointCloud2 modified_map = *(modified_map_ptr.get());
  pcl::fromROSMsg(map, map_pcd);
  pcl::fromROSMsg(modified_map, modified_map_pcd);
  pcl::io::savePLYFile(std::string("maps/map.ply"),map_pcd);
  std::cout << "map saved" << std::endl;
  pcl::io::savePLYFile(std::string("maps/modified_map.ply"),modified_map_pcd);
  std::cout << "modified map saved" << std::endl;
  rclcpp::shutdown();
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MapSaver>();
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr service = node->create_service<std_srvs::srv::Empty>("/map_saver/save_map", &save_map);
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}