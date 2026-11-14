#include "scan_process_node.h"
#include <chrono>


/*
 * 构造函数
 */
ScanProcess::ScanProcess()
{

  double downsample_resolution = private_node_.param<double>("/scan_process_node/downsample_resolution", 0.1);
  std::cout << "voxel grid size is: " << downsample_resolution << std::endl;
  auto voxelgrid = new pcl::VoxelGrid<PointT>();
    
  voxelgrid->setLeafSize(downsample_resolution, downsample_resolution, downsample_resolution);
  downsample_filter.reset(voxelgrid);


  double radius = private_node_.param<double>("/scan_process_node/radius_radius", 0.8);
  int min_neighbors = private_node_.param<int>("/scan_process_node/radius_min_neighbors", 2);
  std::cout << "outlier_removal: RADIUS " << radius << " - " << min_neighbors << std::endl;
  pcl::RadiusOutlierRemoval<PointT>::Ptr rad(new pcl::RadiusOutlierRemoval<PointT>());
  rad->setRadiusSearch(radius);
  rad->setMinNeighborsInRadius(min_neighbors);
  outlier_removal_filter = rad;

  laser_scan_subscriber_ = node_handle_.subscribe("scan", 1, &ScanProcess::ScanCallback, this);

  cloud_publisher_ = node_handle_.advertise<sensor_msgs::PointCloud2>("process_scan", 10);

  // 第一帧数据的标志
  is_first_scan_ = true;
    
}

/*
 * 析构函数
 */
ScanProcess::~ScanProcess()
{
    ROS_INFO("Destroying ScanProcess");
}

/*
 * 回调函数 进行数据处理
 */
void ScanProcess::ScanCallback(const sensor_msgs::LaserScan::ConstPtr &scan_msg)
{
    // step1 进行数据类型转换
    std::chrono::steady_clock::time_point start_time = std::chrono::steady_clock::now();
    
    // 对第一帧数据进行特殊处理
    if (is_first_scan_ == true)
    {      
      // 进行第一帧数据的处理,只转换数据类型 并 保存到current_pointcloud_
      // PointCloudT::Ptr的数据类型为boost::shared_ptr
      PointCloudT::Ptr cloud_msg = boost::shared_ptr<PointCloudT>(new PointCloudT());
      // 对容器进行初始化
      cloud_msg->points.resize(scan_msg->ranges.size());

      for (unsigned int i = 0; i < scan_msg->ranges.size(); ++i)
	{
	  // 首先声明一个 cloud_msg第i个点的 引用
	  pcl::PointXYZ &point_tmp = cloud_msg->points[i];
	  // 获取scan的第i个点的距离值
	  float range = scan_msg->ranges[i];

	  // 将 inf 与 nan 点 设置为无效点
	  if (!std::isfinite(range))
            continue;

	  // 有些雷达驱动会将无效点设置成 range_max+1
	  // 所以要根据雷达的range_min与range_max进行有效值的判断
	  if (range > scan_msg->range_min && range < scan_msg->range_max)
	    {
	      // 获取第i个点对应的角度
	      float angle = scan_msg->angle_min + i * scan_msg->angle_increment;
	      // 获取第i个点在笛卡尔坐标系下的坐标
	      point_tmp.x = range * cos(angle);
	      point_tmp.y = range * sin(angle);
	      point_tmp.z = 0.0;
	    }
	}

      // 高度为1的情况下, width即为所有点的个数
      cloud_msg->width = scan_msg->ranges.size();
      cloud_msg->height = 1;
      cloud_msg->is_dense = true; // not contains nans
      cloud_msg->header.frame_id = scan_msg->header.frame_id;

      // 将scan_msg的消息头 赋值到 pcl::PointCloud<pcl::PointXYZ>的消息头
      pcl_conversions::toPCL(scan_msg->header, cloud_msg->header);

      // downsample PointCloud
      auto filtered = outlier_removal(cloud_msg);

      filtered = downsample(filtered);

      sensor_msgs::PointCloud2 outMsg;
      pcl::toROSMsg(*filtered, outMsg);
      cloud_publisher_.publish(outMsg);
 
      is_first_scan_ = true;   
      return;
    }
       
    else
      return;
}




/**
   * @brief downsample a point cloud
   * @param cloud  input cloud
   * @return downsampled point cloud
   */
  pcl::PointCloud<PointT>::ConstPtr ScanProcess::downsample(const pcl::PointCloud<PointT>::ConstPtr& cloud) const {
    if(!downsample_filter) {
      return cloud;
    }

    pcl::PointCloud<PointT>::Ptr filtered(new pcl::PointCloud<PointT>());
    downsample_filter->setInputCloud(cloud);
    downsample_filter->filter(*filtered);

    return filtered;
  }

/**
   * @brief outlier a point cloud
   * @param cloud  input cloud
   * @return outlier point cloud
   */
   pcl::PointCloud<PointT>::ConstPtr ScanProcess::outlier_removal(const pcl::PointCloud<PointT>::ConstPtr& cloud) const {
    if(!outlier_removal_filter) {
      return cloud;
    }

    pcl::PointCloud<PointT>::Ptr filtered(new pcl::PointCloud<PointT>());
    outlier_removal_filter->setInputCloud(cloud);
    outlier_removal_filter->filter(*filtered);
    filtered->header = cloud->header;

    return filtered;
  }




int main(int argc, char **argv)
{
    ros::init(argc, argv, "scan_process_node"); // 节点的名字
    ScanProcess scan_process;

    ros::spin(); // 程序执行到此处时开始进行等待，每次订阅的消息到来都会执行一次ScanCallback()
    return 0;
}
