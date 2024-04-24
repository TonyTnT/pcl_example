// Copyright (c) 2022 Jonas Mahler

// This file is part of pcl_example.

// pcl_example is free software: you can redistribute it and/or modify it under the terms 
// of the GNU General Public License as published by the Free Software Foundation, 
// either version 3 of the License, or (at your option) any later version.

// pcl_example is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; 
// without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. 
// See the GNU General Public License for more details.

// You should have received a copy of the GNU General Public License along 
// with Foobar. If not, see <https://www.gnu.org/licenses/>. 

#define BOOST_BIND_NO_PLACEHOLDERS

#include <memory>
#include "iostream"
#include <string>
#include <stdio.h>
#include <iterator>
#include <algorithm>

#include "pcl_example/pcl_example_node.hpp"

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/passthrough.h>

#include "sensor_msgs/msg/point_cloud2.hpp"
#include "std_msgs/msg/header.hpp"

using std::placeholders::_1;
    
Pcl_Example::Pcl_Example(const rclcpp::NodeOptions& options) : Node("pcl_example",options) 
{
      
  declare_parameter<std::string>("topic_pointcloud_in","bf_lidar/point_cloud_out");
  declare_parameter<std::string>("topic_pointcloud_out", "bf_lidar/point_cloud_pcl_example");
  declare_parameter<std::string>("qos_type","best_effort");
  declare_parameter<float>("x_max", 1.0);
  declare_parameter<float>("x_min", -1.0);
  declare_parameter<float>("y_max", 1.0);
  declare_parameter<float>("y_min", -1.0);
  declare_parameter<float>("z_max", 1.0);
  declare_parameter<float>("z_min", -1.0);
  
  param_topic_pointcloud_in = get_parameter("topic_pointcloud_in").as_string();
  param_topic_pointcloud_out = get_parameter("topic_pointcloud_out").as_string();
  pointcloud_qos_type = get_parameter("qos_type").as_string();
  x_max = get_parameter("x_max").as_double();
  x_min = get_parameter("x_min").as_double();
  y_max = get_parameter("y_max").as_double();
  y_min = get_parameter("y_min").as_double();
  z_max = get_parameter("z_max").as_double();
  z_min = get_parameter("z_min").as_double();
  
  rclcpp::QoS qos(rclcpp::KeepLast(2));
  if (pointcloud_qos_type == "best_effort") {
    qos = rclcpp::QoS(rclcpp::KeepLast(2)).best_effort();
  } else if (pointcloud_qos_type == "reliable") {
    qos = rclcpp::QoS(rclcpp::KeepLast(2)).reliable();
  } else {
    RCLCPP_WARN(this->get_logger(), "Invalid pointcloud_qos_type parameter. Using default best_effort.");
    qos = rclcpp::QoS(rclcpp::KeepLast(2)).best_effort();
  }
  publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(param_topic_pointcloud_out, qos);
  subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    param_topic_pointcloud_in, qos, std::bind(&Pcl_Example::topic_callback, this, _1));

  RCLCPP_INFO(this->get_logger(), "\n"
  "Node:       pcl_example\n"
  "Subscribes: Pointcloud2 message: %s\n"
  "Publishes:  Pointcloud2 message: %s \n"
  "Details:    VoxelGrid & PassThrough applied in this example.\n"
  "            PassThrough with x range {%f, %f}, y range {%f, %f}, z range {%f, %f}.\n"
  "Running...", param_topic_pointcloud_in.c_str(),param_topic_pointcloud_out.c_str(), x_min, x_max, y_min, y_max, z_min, z_max);

}

void Pcl_Example::topic_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  unsigned int num_points = msg->width;
  RCLCPP_INFO(this->get_logger(), "The number of points in the input pointcloud is %i", num_points);
    
  pcl::PCLPointCloud2::Ptr cloud (new pcl::PCLPointCloud2 ());
  // ROS2 Pointcloud2 to PCL Pointcloud2
  
  pcl_conversions::toPCL(*msg,*cloud);    
                       
  // Insert your pcl object here
  // -----------------------------------
  pcl::PCLPointCloud2::Ptr cloud_filtered_voxelgrid (new pcl::PCLPointCloud2 ());
  pcl::PCLPointCloud2::Ptr cloud_filtered_passthrough (new pcl::PCLPointCloud2 ());

  // VoxelGrid filter
  pcl::VoxelGrid<pcl::PCLPointCloud2> sor_voxelgrid;
  sor_voxelgrid.setInputCloud(cloud);
  sor_voxelgrid.setLeafSize(0.01, 0.01, 0.01);
  sor_voxelgrid.filter(*cloud_filtered_voxelgrid);

  // PassThrough filter
  pcl::PassThrough<pcl::PCLPointCloud2> sor_passthrough;
  sor_passthrough.setInputCloud(cloud_filtered_voxelgrid);
  sor_passthrough.setFilterFieldName("x");
  sor_passthrough.setFilterLimits(x_min, x_max);
  sor_passthrough.filter(*cloud_filtered_passthrough);

  sor_passthrough.setInputCloud(cloud_filtered_passthrough);
  sor_passthrough.setFilterFieldName("y");
  sor_passthrough.setFilterLimits(y_min, y_max);
  sor_passthrough.filter(*cloud_filtered_passthrough);

  sor_passthrough.setInputCloud(cloud_filtered_passthrough);
  sor_passthrough.setFilterFieldName("z");
  sor_passthrough.setFilterLimits(z_min, z_max);
  sor_passthrough.filter(*cloud_filtered_passthrough);

  //------------------------------------

  //------------------------------------

  // PCL message to ROS2 message 
  sensor_msgs::msg::PointCloud2 cloud_out;
  pcl_conversions::fromPCL(*cloud_filtered_passthrough, cloud_out);  

  unsigned int num_points_out = cloud_out.width;
  RCLCPP_INFO(this->get_logger(), "The number of points in the pcl_example_out pointcloud is %i", num_points_out);

  cloud_out.header.frame_id = msg->header.frame_id;
  cloud_out.header.stamp = msg->header.stamp;

  // Publish to ROS2 network
  publisher_->publish(cloud_out);
}
