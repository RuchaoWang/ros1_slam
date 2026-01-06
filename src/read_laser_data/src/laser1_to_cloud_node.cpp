/*
 * @Author: your name
 * @Date: 2023-07-25 10:04:57
 * @LastEditTime: 2023-10-16 23:06:56
 * @LastEditors: your name
 * @Description: 
 * @FilePath: /MyFormProject/src/read_laser_data/src/laser1_to_cloud_node.cpp
 * 可以输入预定的版权声明、个性签名、空行等
 */
#include <iostream>

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <laser_geometry/laser_geometry.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/LaserScan.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include "pcl_ros/transforms.h"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>

class Laser_to_Cloud {
  public:
    Laser_to_Cloud();
    //订阅 Laser1Scan　数据，并发布 PointCloud2 点云 
    void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan);

    //直接订阅 PointCloud2 然后自动转换为 pcl::PointCloud
    void pclCloudCallback(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& cloud);

  private:
    ros::NodeHandle node_;
    ros::NodeHandle private_node_;          // ros中的私有句柄
    laser_geometry::LaserProjection projector;
    
    tf::TransformListener tfListener;

    std::string laserFrame;     //激光雷达1frame_id

    tf::TransformListener laser_tf_car_listener;

    //发布 雷达1的点云数据　"PointCloud2"
    ros::Publisher point_cloud_publisher_;

    //订阅雷达1的话题名称 "/scan"
    ros::Subscriber scan_sub_;

    //订阅 "/cloud2" -> "PointCloud2"
    ros::Subscriber pclCloud_sub_;
};

Laser_to_Cloud::Laser_to_Cloud() : private_node_("~")
{
  // 获取激光雷达frame_id
  // laser是实车雷达1的frame_id
  private_node_.param<std::string>("laserFrame",laserFrame,"laser");
  //订阅雷达消息　"/scan"
  scan_sub_ = node_.subscribe<sensor_msgs::LaserScan> ("/scan", 10, &Laser_to_Cloud::scanCallback, this);

  //发布LaserScan转换为PointCloud2的后的数据
  point_cloud_publisher_ = node_.advertise<sensor_msgs::PointCloud2> ("/cloud_map", 10, false);

  //此处的tf是 laser_geometry 要用到的
  tfListener.setExtrapolationLimit(ros::Duration(0.1));
}

// 这里的更改，将点云信息修改到合适的坐标系
void Laser_to_Cloud::scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan)
{
  // 关键：把点云输出到“水平车体坐标系”
  const std::string target_frame = "base_footprint";

  sensor_msgs::PointCloud2 cloud_out;

  try {
    // 等 TF 可用（scan 的 frame 通常就是 laser）
    tfListener.waitForTransform(
        target_frame,
        scan->header.frame_id,
        scan->header.stamp,
        ros::Duration(0.05));

    // 一步到位：LaserScan -> PointCloud2，并变换到 base_footprint
    projector.transformLaserScanToPointCloud(
        target_frame,
        *scan,
        cloud_out,
        tfListener);

    // 保险：统一 header
    cloud_out.header.stamp = scan->header.stamp;
    cloud_out.header.frame_id = target_frame;

    point_cloud_publisher_.publish(cloud_out);
  }
  catch (tf::TransformException& ex) {
    ROS_WARN_THROTTLE(1.0, "Laser->Cloud TF failed (%s -> %s): %s",
                      scan->header.frame_id.c_str(), target_frame.c_str(), ex.what());
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "my_pcl_node");

  Laser_to_Cloud cloudtest;

  ros::spin();

  return 0;
}
