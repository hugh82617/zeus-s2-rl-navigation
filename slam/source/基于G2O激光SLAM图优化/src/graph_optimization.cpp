#include <string>
#include <chrono>
#include <iostream>
#include <fstream>
#include "ros/ros.h"
#include "ros/console.h"
#include "sensor_msgs/PointCloud.h"
#include "sensor_msgs/LaserScan.h"
#include <std_msgs/String.h>
#include <std_msgs/Header.h>
#include "message_filters/subscriber.h"
#include "geometry_msgs/Pose2D.h"
#include <pcl_ros/point_cloud.h>

#include <g2o/types/slam2d/types_slam2d.h>
#include <g2o/types/slam3d/types_slam3d.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/solvers/eigen/linear_solver_eigen.h>

using namespace std;

typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

ifstream fin;
ifstream fin_laser;
std::vector<pcl::PointCloud<PointT>> key_frame_set;
std::vector<sensor_msgs::LaserScan> key_frame_;
std::vector<geometry_msgs::Pose2D> key_pose_;
std::vector<geometry_msgs::Pose2D> g2o_key_pose_;
pcl::PointCloud<pcl::PointXYZ>::Ptr pointcloud_; 
geometry_msgs::Pose2D g2o_pose_;
ros::Publisher map_points_pub;


int main(int argc, char **argv)
{
    ros::init(argc, argv, "graphoptimization"); // 节点的名字
    ros::NodeHandle nh_;
    //read original g2o file
    string original_g2o_file = nh_.param<string>("/graphoptimization/g2ofile","");
    string result_g2o_file_path = nh_.param<string>("/graphoptimization/resultg2o","");
    string keyscan_file = nh_.param<string>("/graphoptimization/keyscanfile","");


    ifstream fin(original_g2o_file);
    if (!fin) {
      cout << "file" << argv[1] << "does not exist." << endl;
      return 1;
    }
    // 设定g2o
    typedef g2o::BlockSolver<g2o::BlockSolverTraits<3, 3>> BlockSolverType;
    typedef g2o::LinearSolverEigen< BlockSolverType::PoseMatrixType> LinearSolverType;
    auto solver = new g2o::OptimizationAlgorithmLevenberg(g2o::make_unique< BlockSolverType>(g2o::make_unique< LinearSolverType>()));
    g2o::SparseOptimizer optimizer; // 图模型
    // g2o::VertexSE2* firstRobotPose = dynamic_cast<g2o::VertexSE2*>(optimizer.vertex(0));
    // firstRobotPose->setFixed(true);
    optimizer.setAlgorithm(solver); // 设置求解器
    optimizer.setVerbose(true); // 打开调试输出
    //
    int vertexCnt = 0, edgeCnt = 0; // 顶点和边的数量
    while (!fin.eof()) {
      string name;
      fin >> name;
      if (name == "VERTEX2") {
	// SE3 顶点
	g2o::VertexSE2 *v = new g2o::VertexSE2();//使用g2o默认的SE3顶点
	int index = 0;
	fin >> index; //读取g2o文件为每个位姿设置的下标
	//cout << "index: " << index << endl;
	v->setId(index);// 设置顶点ID，并把位姿数据读入顶点
	v->read(fin);
	optimizer.addVertex(v);
	vertexCnt++;
	if (index == 0)
	  v->setFixed(true);
      } else if (name == "EDGE2") {
	// SE3-SE3 边
	g2o::EdgeSE2 *e = new g2o::EdgeSE2();//使用g2o默认的SE3边
	int idx1, idx2; // 边所对应的关联的两个顶点ID
	fin >> idx1 >> idx2;
	//cout << "idx: " << idx1 << idx2 << endl;
	e->setId(edgeCnt++);
	e->setVertex(0, optimizer.vertices()[idx1]);//分别连接上下顶点
	e->setVertex(1, optimizer.vertices()[idx2]);
	e->read(fin);
	optimizer.addEdge(e);
      }
      if (!fin.good()) break;
    }
    
    fin.close();
    cout << "read total " << vertexCnt << " vertices, " << edgeCnt << " edges." << endl;
    cout << "optimizin..." << endl;
    optimizer.initializeOptimization();
    optimizer.optimize(128);
    cout << "saving optimization results..." << endl;

    string result_g2o = result_g2o_file_path + "result.g2o";

    optimizer.save(result_g2o.c_str());

    //publish after graph optimization point cloud map
    map_points_pub = nh_.advertise<sensor_msgs::PointCloud2>("/slam/map_points", 1, true);

    //read after graph optimization result pose
    fin.open(result_g2o_file_path + "result.g2o", ios::in);
    fin_laser.open(keyscan_file, ios::in);
    pointcloud_ = boost::shared_ptr<PointCloudT>(new PointCloudT());
        
    while (!fin.eof()) {
      string name;
      fin >> name;
      if (name == "VERTEX_SE2") {
        int index = 0;
        fin >> index;
        cout << "VERTEX_SE2" << index << endl;
        fin >> g2o_pose_.x >> g2o_pose_.y >> g2o_pose_.theta;
        //cout << "g2o_pose_: " << g2o_pose_ << endl;
        g2o_key_pose_.push_back(g2o_pose_);
      } 

      if (!fin.good()) break;
    }
    
    //read key scan point cloud
    int num_point_key_scan = 0;
    while (!fin_laser.eof()) {
      string name;
      fin_laser >> name;      
      if (name == "keyscanpointnumis")
      {
         fin_laser >> num_point_key_scan;
         //std::cout<<"std key scan num is "<<num_point_key_scan<<std::endl;
      }
      else if (name == "point") {
        for(unsigned int i = 0; i < num_point_key_scan ; ++i)//3585
        {

          pointcloud_->points.resize(num_point_key_scan);
          fin_laser >> pointcloud_->points[i].x >> pointcloud_->points[i].y;
          pointcloud_->points[i].z = 0.0;
          pointcloud_->width = num_point_key_scan;
          pointcloud_->height = 1;
          pointcloud_->is_dense = true;
          pointcloud_->header.frame_id = "laser";
        }
        //std::cout<<"get one key scan"<<std::endl;
        key_frame_set.push_back(*pointcloud_);

      } 
      if (!fin_laser.good()) break;
    }


    //merge key scan based on after pose graph optimization's pose
    PointCloudT::Ptr cloud_pl = boost::shared_ptr<PointCloudT>(new PointCloudT());
    cloud_pl->points.resize(key_frame_set.size()* num_point_key_scan);
    unsigned int m = 0;
    for(unsigned int i = 0; i < key_frame_set.size(); ++i)
	  {

      float cos_k = cos(g2o_key_pose_[i].theta);
      float sin_k = sin(g2o_key_pose_[i].theta);
      std::cout << "weizi: (" << g2o_key_pose_[i].x << ", " << g2o_key_pose_[i].y << ", " << g2o_key_pose_[i].theta * 180 / M_PI << ")" << std::endl;
      for(unsigned int j = 0; j < key_frame_set[i].width; ++j)
      {
        cloud_pl->points[m].x = cos_k * key_frame_set[i].points[j].x - sin_k * key_frame_set[i].points[j].y + g2o_key_pose_[i].x;
        cloud_pl->points[m].y = sin_k * key_frame_set[i].points[j].x + cos_k * key_frame_set[i].points[j].y + g2o_key_pose_[i].y;
        cloud_pl->points[m].z = 0.0;
        m++;
      }
	  }
    cloud_pl->width = key_frame_set.size()* num_point_key_scan;
    cloud_pl->height = 1;
    cloud_pl->is_dense = true;
    cloud_pl->header.frame_id = "laser";

    sensor_msgs::PointCloud2Ptr cloud_msgs(new sensor_msgs::PointCloud2());
    pcl::toROSMsg(*cloud_pl, *cloud_msgs);

    //publish point-cloud
    map_points_pub.publish(cloud_msgs);

    ros::spin();  
    return 0;
}
