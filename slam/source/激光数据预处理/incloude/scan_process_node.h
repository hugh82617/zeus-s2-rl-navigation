
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
#include <pcl_conversions/pcl_conversions.h>


// 使用PCL中点的数据结构 pcl::PointXYZ
typedef pcl::PointXYZ PointT;
// 使用PCL中点云的数据结构 pcl::PointCloud<pcl::PointXYZ>
typedef pcl::PointCloud<PointT> PointCloudT;

class ScanProcess
{
    
private:
    ros::NodeHandle node_handle_;           // ros中的句柄
    ros::NodeHandle private_node_;          // ros中的私有句柄
    ros::Subscriber laser_scan_subscriber_; // 声明一个Subscriber
    ros::Publisher cloud_publisher_;

    bool is_first_scan_;    // 判断是否是第一个雷达数据

   
    // downsample
    pcl::Filter<PointT>::Ptr downsample_filter;

    // outlier delete
    pcl::Filter<PointT>::Ptr outlier_removal_filter;

    pcl::PointCloud<PointT>::ConstPtr downsample(const pcl::PointCloud<PointT>::ConstPtr& cloud) const;
    pcl::PointCloud<PointT>::ConstPtr outlier_removal(const pcl::PointCloud<PointT>::ConstPtr& cloud) const;

    
public:
    ScanProcess();
    ~ScanProcess();
    void ScanCallback(const sensor_msgs::LaserScan::ConstPtr &scan_msg);
};


