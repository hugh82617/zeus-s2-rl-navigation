/*
 * Copyright 2020 The Project Author: lixiang
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef LESSON2_SCAN_MATCH_ICP
#define LESSON2_SCAN_MATCH_ICP

#include <vector>
#include <map>
#include <Eigen/Dense>

#include <ecl/threads.hpp>
// ros
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>

// pcl_ros
#include <pcl_ros/point_cloud.h>

// pcl
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/radius_outlier_removal.h>


// 使用PCL中点的数据结构 pcl::PointXYZ
typedef pcl::PointXYZ PointT;
// 使用PCL中点云的数据结构 pcl::PointCloud<pcl::PointXYZ>
typedef pcl::PointCloud<PointT> PointCloudT;

typedef pcl::PointNormal PointNormalT;
typedef pcl::PointCloud<PointNormalT> PointCloudWithNormals;

class ScanMatchICP
{
    
private:
    ros::NodeHandle node_handle_;           // ros中的句柄
    ros::NodeHandle private_node_;          // ros中的私有句柄
    ros::Subscriber laser_scan_subscriber_; // 声明一个Subscriber

    bool is_first_scan_;    // 判断是否是第一个雷达数据

    pcl::PointCloud<pcl::PointXYZ>::Ptr current_pointcloud_;    // 当前帧雷达数据
    pcl::PointCloud<pcl::PointXYZ>::Ptr last_pointcloud_;       // 前一帧雷达数据

    // icp算法
    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp_;

    pcl::IterativeClosestPointWithNormals<pcl::PointNormal, pcl::PointNormal> icp_normals;

    // downsample
    pcl::Filter<PointT>::Ptr downsample_filter;

    // outlier delete
    pcl::Filter<PointT>::Ptr outlier_removal_filter;

    void ScanMatchWithICP(const sensor_msgs::LaserScan::ConstPtr &scan_msg, float inlier_fraction);
    pcl::PointCloud<PointT> ConvertScan2PointCloud(const sensor_msgs::LaserScan::ConstPtr &scan_msg);
    pcl::PointCloud<PointT>::ConstPtr downsample(const pcl::PointCloud<PointT>::ConstPtr& cloud) const;
    pcl::PointCloud<PointT>::ConstPtr outlier_removal(const pcl::PointCloud<PointT>::ConstPtr& cloud) const;

    Eigen::Matrix4f prev_trans;
    float inlier_fraction_;
    ros::Publisher map_points_pub;
    ros::Publisher map_point_pub;

    std::vector<pcl::PointCloud<PointT>> key_frame_set;
    std::vector<Eigen::Matrix4f> key_frame_pose_set;

public:
    ScanMatchICP();
    ~ScanMatchICP();
    void ScanCallback(const sensor_msgs::LaserScan::ConstPtr &scan_msg);
};


#endif // LESSON2_SCAN_MATCH_ICP
