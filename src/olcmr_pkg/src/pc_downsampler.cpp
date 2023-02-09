#include "rclcpp/rclcpp.hpp"
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>

class PcDownsampler : public rclcpp::Node
{
public:
  PcDownsampler()
  : Node("pc_downsampler")
  {
      declare_parameter("vg_size_for_slam", 0.2);
      get_parameter("vg_size_for_slam", vg_size_for_slam_);
      declare_parameter("vg_size_for_tsdf", 0.1);
      get_parameter("vg_size_for_tsdf", vg_size_for_tsdf_);
      using std::placeholders::_1;
      pc_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
          "point_cloud", 10, std::bind(&PcDownsampler::pc_callback, this, _1));
      filtered_pc_pub_slam_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("point_cloud/downsampled/slam", 10);
      filtered_pc_pub_tsdf_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("point_cloud/downsampled/tsdf", 10);
  }

private:
  void pc_callback(const sensor_msgs::msg::PointCloud2::SharedPtr message) const
  {
    pcl::fromROSMsg(*message, *cloud_ptr);
    
    pcl::VoxelGrid<pcl::PointXYZI> voxel_grid;
    voxel_grid.setLeafSize(vg_size_for_slam_, vg_size_for_slam_, vg_size_for_slam_);
    voxel_grid.setInputCloud(cloud_ptr);
    voxel_grid.filter(*filtered_cloud_slam_ptr);

    voxel_grid.setLeafSize(vg_size_for_tsdf_, vg_size_for_tsdf_, vg_size_for_tsdf_);
    voxel_grid.setInputCloud(cloud_ptr);
    voxel_grid.filter(*filtered_cloud_tsdf_ptr);

    sensor_msgs::msg::PointCloud2 ros_cloud_filtered_slam;
    sensor_msgs::msg::PointCloud2 ros_cloud_filtered_tsdf;
    pcl::toROSMsg(*filtered_cloud_slam_ptr, ros_cloud_filtered_slam);
    pcl::toROSMsg(*filtered_cloud_tsdf_ptr, ros_cloud_filtered_tsdf);
    ros_cloud_filtered_slam.header = (*message).header;
    ros_cloud_filtered_tsdf.header = (*message).header;
    filtered_pc_pub_slam_->publish(ros_cloud_filtered_slam);
    filtered_pc_pub_tsdf_->publish(ros_cloud_filtered_tsdf);
  }
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pc_sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr filtered_pc_pub_slam_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr filtered_pc_pub_tsdf_;
  double vg_size_for_slam_;
  double vg_size_for_tsdf_;
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_ptr = pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>());
  pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_cloud_slam_ptr = pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>());
  pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_cloud_tsdf_ptr = pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>());
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PcDownsampler>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}