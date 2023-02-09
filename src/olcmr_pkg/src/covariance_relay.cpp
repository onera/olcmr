// Copyright 2016 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <chrono>
#include <memory>
#include <random>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/imu.hpp"

using std::placeholders::_1;

//odometry covariances
static double sigma_x_odom = 0.1;
static double sigma_y_odom = 0.2;
static double sigma_z_odom = 0.2;
static double sigma_w_odom = 0.01;

static double sigma_vx_odom = 0.01;
static double sigma_vy_odom = 0.1;
static double sigma_vz_odom = 0.1;
static double sigma_wz_odom = 10;

static double sigma_r_imu = 0.02;
static double sigma_p_imu = 0.1;
static double sigma_y_imu = 0.001;

static double sigma_wr_imu = 0.02;
static double sigma_wp_imu = 0.1;
static double sigma_wy_imu = 0.005;

static double sigma_ax_imu = 0.1;
static double sigma_ay_imu = 0.1;
static double sigma_az_imu = 0.1;

class CovarianceRelay : public rclcpp::Node
{
public:
  CovarianceRelay()
  : Node("covariance_relay")
  {
    odom_subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "odom", 10, std::bind(&CovarianceRelay::odom_topic_callback, this, _1));

    odom_publisher_ = this->create_publisher<nav_msgs::msg::Odometry>("odom_with_covariance", 10);

    imu_subscription_ = this->create_subscription<sensor_msgs::msg::Imu>(
      "imu", 10, std::bind(&CovarianceRelay::imu_topic_callback, this, _1));

    imu_publisher_ = this->create_publisher<sensor_msgs::msg::Imu>("imu_with_covariance", 10);
  }

private:

  void odom_topic_callback(const nav_msgs::msg::Odometry::SharedPtr message) const
  {

    auto odometry = *message;
    odometry.child_frame_id = "ekf_base_link";
    odometry.pose.covariance = std::array<double,36> {
        sigma_x_odom,0,0,0,0,0,
        0,sigma_y_odom,0,0,0,0,
        0,0,sigma_z_odom,0,0,0,
        0,0,0,0,0,0,
        0,0,0,0,0,0,
        0,0,0,0,0,sigma_w_odom};
    odometry.twist.covariance = std::array<double,36> {
        sigma_vx_odom,0,0,0,0,0,
        0,sigma_vy_odom,0,0,0,0,
        0,0,sigma_vz_odom,0,0,0,
        0,0,0,0,0,0,
        0,0,0,0,0,0,
        0,0,0,0,0,sigma_wz_odom};
    odom_publisher_->publish(odometry);
  }

  void imu_topic_callback(const sensor_msgs::msg::Imu::SharedPtr message) const
  {

    auto imu = *message;
    imu.header.frame_id = "ekf_base_link";
    imu.orientation_covariance = std::array<double,9> {
        sigma_r_imu,0,0,
        0,sigma_p_imu,0,
        0,0,sigma_y_imu};

    imu.angular_velocity_covariance = std::array<double,9> {
        sigma_wr_imu,0,0,
        0,sigma_wp_imu,0,
        0,0,sigma_wy_imu};

    imu.linear_acceleration_covariance = std::array<double,9> {
        sigma_ax_imu,0,0,
        0,sigma_ay_imu,0,
        0,0,sigma_az_imu};
    imu_publisher_->publish(imu);
  }

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscription_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscription_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_publisher_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CovarianceRelay>());
  rclcpp::shutdown();
  return 0;
}
