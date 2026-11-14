 #include "ros/ros.h"
#include "ros/console.h"
#include "sensor_msgs/PointCloud.h"
#include "sensor_msgs/LaserScan.h"
#include <std_msgs/String.h>
#include <std_msgs/Header.h>
#include "message_filters/subscriber.h"
#include "geometry_msgs/Pose2D.h"
#include <pcl_ros/point_cloud.h>

// pcl
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/radius_outlier_removal.h>

#include <Eigen/Dense>

#include <iostream>
#include "csm.h"
#include "egsl.h"
#include <fstream>

using namespace std;

//CSM* csm;

typedef pcl::PointXYZ PointT;
// 使用PCL中点云的数据结构 pcl::PointCloud<pcl::PointXYZ>
typedef pcl::PointCloud<PointT> PointCloudT;

class icp
{
    public:
    icp();
    ~icp();
    void laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg);
    void laserScanToLDP(const sensor_msgs::LaserScan::ConstPtr& msg, LDP& laser);
    void CalcuNormalVector(const sensor_msgs::LaserScan::ConstPtr& msg);
    void laserScanToLDP(const sensor_msgs::LaserScan msg, LDP& laser);

    pcl::PointCloud<PointT> ConvertScan2PointCloud(const sensor_msgs::LaserScan::ConstPtr &scan_msg);

    bool static inline distance_valid(double r,double min_r,double max_r)
    {
        if (r>min_r && r<max_r)
            return true;
        else
            return false;  
    }

    double static inline distance(double p1_x,double p1_y,double p2_x,double p2_y)
    {
        return sqrt(pow(p1_x-p2_x,2)+pow(p1_y-p2_y,2));
    }

    private:

    double key_scan_threshold;
    double loop_detect_dis;
    double loop_detect_threshold;
    double x[3] = { 0.0 , 0.0 , 0.0 };
    double delta[3] = { 0.0 , 0.0 , 0.0 };
    double delta_h[3] = { 0.0 , 0.0 , 0.0 };
    double prev_delta[3] = { 0.0 , 0.0 , 0.0 };
    CSM* csm;
    CSM* csm_h;
    double corres_ratio = 0.0;
    double corres_ratio_h = 0.0;
    bool first;
    struct sm_params params;
    struct sm_result result;
    struct sm_params params_h;
    struct sm_result result_h;
    LDP laser_ref;
    LDP laser_incidence_angle;
    ros::Subscriber laser_scan_sub_;
    ros::NodeHandle nh_;
     
    std::vector<pcl::PointCloud<PointT>> key_frame_set;
    std::vector<sensor_msgs::LaserScan> key_frame_;
    std::vector<geometry_msgs::Pose2D> key_pose_;
    std::vector<geometry_msgs::Pose2D> g2o_key_pose_;
    geometry_msgs::Pose2D pose_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr pointcloud_; 
    ros::Publisher map_points_pub;
    std::vector<LDP> key_ldp_;
    std::vector<sensor_msgs::LaserScan> key_lader_;
    ofstream ofs;
    ofstream ofs_p;
    int VERTEX2_int;
    //fstream finout;

};

icp::icp(){
   csm = new CSM();
   csm_h = new CSM();
   first = true;
   params.sigma = 0.01;
   params.use_corr_tricks = 1;
   params.max_correspondence_dist =1.5;
   params.max_iterations = 10;
   params.outliers_maxPerc = 0.9;
   params.outliers_adaptive_mult = 2.0;
   params.outliers_adaptive_order = 0.85;
   params.outliers_remove_doubles = 1;
   params.use_point_to_line_distance = 1;
   params.do_alpha_test = 0;
   params.do_compute_covariance = 1;//放开此项，计算耗时增加
   params.use_ml_weights = 0;
   params.use_sigma_weights = 0;
   params.min_reading = 0.0;
   params.max_reading = 30.0;
   params.laser[0] = 0.0;
   params.laser[1] = 0.0;
   params.laser[2] = 0.0;

   params_h.sigma = 0.01;
   params_h.use_corr_tricks = 1;
   params_h.max_correspondence_dist =1.5;
   params_h.max_iterations = 10;
   params_h.outliers_maxPerc = 0.9;
   params_h.outliers_adaptive_mult = 2.0;
   params_h.outliers_adaptive_order = 0.85;
   params_h.outliers_remove_doubles = 1;
   params_h.use_point_to_line_distance = 1;
   params_h.do_alpha_test = 0;
   params_h.do_compute_covariance = 1;//放开此项，计算耗时增加
   params_h.use_ml_weights = 0;
   params_h.use_sigma_weights = 0;
   params_h.min_reading = 0.0;
   params_h.max_reading = 30.0;
   params_h.laser[0] = 0.0;
   params_h.laser[1] = 0.0;
   params_h.laser[2] = 0.0;

   pointcloud_ = boost::shared_ptr<PointCloudT>(new PointCloudT());
   laser_scan_sub_ = nh_.subscribe("/scan", 1 , &icp::laserCallback , this);
   map_points_pub = nh_.advertise<sensor_msgs::PointCloud2>("keyscanpoint", 10, true);

   string file_path = nh_.param<string>("/lidarodom/filepath","");

   std::cout<<"output data file path is:"<<file_path<<std::endl;
   
   ofs.open(file_path + "test.g2o", ios::out);
   ofs_p.open(file_path + "test_p.txt", ios::out);
   VERTEX2_int = 0;

   key_scan_threshold = nh_.param<double>("/lidarodom/key_scan_threshold", 0.55);
   std::cout << "key scan threshold value is: " << key_scan_threshold << std::endl;

   loop_detect_dis = nh_.param<double>("/lidarodom/loop_detect_dis", 5.0);
   std::cout << "loop detect distance value is: " << loop_detect_dis << std::endl;
   
   loop_detect_threshold = nh_.param<double>("/lidarodom/loop_detect_threshold", 0.5);
   std::cout << "loop detect threshold value is: " << loop_detect_threshold << std::endl;
}

icp::~icp(){
    if(csm) delete csm;
    if(csm_h) delete csm_h;
    ofs.close();
    ofs_p.close();
}

void icp::laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
    if(!msg)
        return;

    ros::Time t1 = ros::Time::now();
    CalcuNormalVector(msg);
    ros::Time t2 = ros::Time::now();

    // csm->ld_fisher0(laser_incidence_angle);
    csm->ld_fisher(laser_incidence_angle);

    // std::cout <<"CalcuNormal cost time:"<<(t2-t1).toSec()<<std::endl;

    if(first)
    {
        laserScanToLDP(msg, laser_ref);
	
        laser_ref->odometry[0] = 0.0;
        laser_ref->odometry[1] = 0.0;
        laser_ref->odometry[2] = 0.0;

        laser_ref->estimate[0] = 0.0;
        laser_ref->estimate[1] = 0.0;
        laser_ref->estimate[2] = 0.0;

        laser_ref->true_pose[0] = 0.0;
        laser_ref->true_pose[1] = 0.0;
        laser_ref->true_pose[2] = 0.0;

        //save first key scan's pose
        pose_.x = 0;
        pose_.y = 0;
        pose_.theta = 0;
        ofs << "VERTEX2 " << VERTEX2_int++ << " " <<pose_.x << " " << pose_.y  << " " << pose_.theta << endl;
        first = false;

        *pointcloud_ = ConvertScan2PointCloud(msg);
        key_frame_set.push_back(*pointcloud_);

	//save first key scan's point cloud
        ofs_p <<"keyscanpointnumis "<<pointcloud_->width;
        ofs_p << " end" << endl;
        
        ofs_p << "point";        
	for(unsigned int i = 0; i < pointcloud_->width ; ++i)
	  {
	    //pointcloud_->points[i]
	    ofs_p << " " << pointcloud_->points[i].x << " " << pointcloud_->points[i].y ;
	  }
	ofs_p << " end" << endl;
        key_pose_.push_back(pose_);
        key_ldp_.push_back(laser_ref);
        key_lader_.push_back(*msg);
        
        return;

    }
    else
    {
        LDP laser_sens;
        laserScanToLDP(msg, laser_sens);

        params.laser_ref = laser_ref;
        params.laser_sens = laser_sens;

        if(!ld_valid_fields(laser_ref))  {
            printf("ScanToMap-Invalid laser data in first scan.\n");
            first = true;
            ld_free(laser_ref);
            return;
        }
        if(!ld_valid_fields(laser_sens))  {
            printf("ScanToMap-Invalid laser data in second scan.\n");
            ld_free(laser_sens);
            return;
        }
	 
        params.first_guess[0] = prev_delta[0];

        params.first_guess[1] = prev_delta[1];

        params.first_guess[2] = prev_delta[2];

        ros::Time t1 = ros::Time::now();

        csm->sm_icp(&params, &result, &corres_ratio);

        ros::Time t2 = ros::Time::now();

        //std::cout <<"icp cost time:"<< (t2-t1).toSec()<<",and corres ratio is:"<<corres_ratio<<std::endl;	

        if(!result.valid){
            printf("[icp][error] ICP result unvalid! ICP stop!\n");
            first = false;
            ld_free(laser_ref);
            ld_free(laser_sens);
            return;
         }
        else{
	  delta[0] = result.x[0];
	  delta[1] = result.x[1];
	  delta[2] = result.x[2];
	  delta[2] = atan2(sin(delta[2]), cos(delta[2]));
	  //Whether current scan laser's pose is far from last-key-scan laser's pose
	  if(corres_ratio < key_scan_threshold)
	    {
	      //current laser's pose is far from last key scan
	      key_ldp_.push_back(laser_sens);
	      key_lader_.push_back(*msg);
	      //std::cout<<"Corres ratio is:"<<corres_ratio<<",and delta pose is:"<<delta[0]<<" "<<delta[1]<<" "<<delta[2]*57.3<<"   ---------------"<<std::endl;

	      pose_.x = cos(key_pose_.back().theta) * delta[0] - sin(key_pose_.back().theta) * delta[1] + key_pose_.back().x;
	      pose_.y = sin(key_pose_.back().theta) * delta[0] + cos(key_pose_.back().theta) * delta[1] + key_pose_.back().y;
	      pose_.theta = key_pose_.back().theta + delta[2];
	      pose_.theta = atan2(sin(pose_.theta), cos(pose_.theta));

	      ofs << "VERTEX2 " << VERTEX2_int++ << " " <<pose_.x << " " << pose_.y  << " " << pose_.theta << endl;

	      Eigen::Matrix3d m_matrix_icp;
	      m_matrix_icp << result.cov_x_m->data[0],result.cov_x_m->data[1],result.cov_x_m->data[2],result.cov_x_m->data[3],result.cov_x_m->data[4],result.cov_x_m->data[5],result.cov_x_m->data[6],result.cov_x_m->data[7],result.cov_x_m->data[8];
	      Eigen::Matrix3d m_matrix__icp_inv_;
	      m_matrix__icp_inv_ = m_matrix_icp.inverse();
	      ofs << "EDGE2 " << VERTEX2_int-2 << " " << VERTEX2_int-1 << " " << delta[0] << " " << delta[1]  << " " << delta[2] << " " << m_matrix__icp_inv_(0,0) << " " << m_matrix__icp_inv_(0,1) << " " << m_matrix__icp_inv_(0,2) << " " << m_matrix__icp_inv_(1,1) << " " << m_matrix__icp_inv_(1,2) << " " << m_matrix__icp_inv_(2,2)<< endl;

	      key_pose_.push_back(pose_);
	      *pointcloud_ = ConvertScan2PointCloud(msg);
	      key_frame_set.push_back(*pointcloud_);
	      ofs_p << "point";
	      for(unsigned int i = 0; i < pointcloud_->width ; ++i)
		{
		  
		  ofs_p << " " << pointcloud_->points[i].x << " " << pointcloud_->points[i].y ;
		}
	      ofs_p << " end" << endl;
	      LDP laser_key_sens;

	      //new added key scan, and try to find loop closure set in key scan set.
	      for(unsigned int i = 0; i < key_pose_.size() - 1; ++i)
		{
		  float dis = sqrt(pow((pose_.x - key_pose_[i].x),2) + pow((pose_.y - key_pose_[i].y),2));
		  //std::cout << "dis: " << dis << std::endl;
		  if(dis < loop_detect_dis)
		    {
		      //loop clousure detect, and use icp to match them
		      laserScanToLDP(key_lader_[i], laser_key_sens);
		      params_h.laser_ref = laser_key_sens;
		      params_h.laser_sens = key_ldp_.back();

		      geometry_msgs::Pose2D pose_n;
            
		      Eigen::Matrix2d m_matrix;
		      Eigen::Matrix2d m_matrix_inv;
		      Eigen::Matrix<double, 2,1> m;
		      Eigen::Matrix<double, 2,1> m_n;
		      m << ( pose_.x - key_pose_[i].x),(pose_.y - key_pose_[i].y);
		      m_matrix << cos(key_pose_[i].theta), -sin(key_pose_[i].theta) , sin(key_pose_[i].theta) ,cos(key_pose_[i].theta);
		      m_matrix_inv = m_matrix.inverse();
		      m_n = m_matrix_inv*m;
		      

		      pose_n.theta = pose_.theta - key_pose_[i].theta;
		      pose_n.theta = atan2(sin(pose_n.theta), cos(pose_n.theta));
	
		      params_h.first_guess[0] = m_n(0,0);
		      params_h.first_guess[1] = m_n(1,0);
		      params_h.first_guess[2] = pose_n.theta;

		      //match loop closure set 
		      csm_h->sm_icp(&params_h, &result_h, &corres_ratio_h);

		      if(!result_h.valid){
                        printf("[icp][error] ICP result unvalid! ICP stop!\n");
		      }else{
                        //std::cout << "corres_ratio_h: " << corres_ratio_h << std::endl;
                        delta_h[0] = result_h.x[0];
			delta_h[1] = result_h.x[1];
			delta_h[2] = result_h.x[2];
			delta_h[2] = atan2(sin(delta_h[2]), cos(delta_h[2]));
                        
		      }

		      //Jurdge loop closure match result is valid or not
		      if(corres_ratio_h > loop_detect_threshold){
			//Hooray, the match result is valid, we find the loop-closure-edge-constraint.

			//calculate information matrix, pose graph optimization will use it
                        Eigen::Matrix3d m_matrix_icp_h;
                        m_matrix_icp_h << result_h.cov_x_m->data[0],result_h.cov_x_m->data[1],result_h.cov_x_m->data[2],result_h.cov_x_m->data[3],result_h.cov_x_m->data[4],result_h.cov_x_m->data[5],result_h.cov_x_m->data[6],result_h.cov_x_m->data[7],result_h.cov_x_m->data[8];
                        Eigen::Matrix3d m_matrix_icp_h_inv_;
                        m_matrix_icp_h_inv_ = m_matrix_icp_h.inverse();
                        ofs << "EDGE2 " << i << " " << key_pose_.size() - 1 << " " << delta_h[0] << " " << delta_h[1]  << " " << delta_h[2] << " " << m_matrix_icp_h_inv_(0,0) << " " << m_matrix_icp_h_inv_(0,1) << " " << m_matrix_icp_h_inv_(0,2) << " " << m_matrix_icp_h_inv_(1,1) << " " << m_matrix_icp_h_inv_(1,2) << " " << m_matrix_icp_h_inv_(2,2)<< endl;
		      }

		      delta_h[0] = 0.0; delta_h[1] = 0.0; delta_h[2] = 0.0;
		      corres_ratio_h = 0.0;

		    }
                
		}

	      // calculate key scan's point cloud based on non-graph-optimization-key-scan's-pose
	      PointCloudT::Ptr cloud_pl = boost::shared_ptr<PointCloudT>(new PointCloudT());
	      cloud_pl->points.resize(key_frame_set.size() * msg->ranges.size());
	      unsigned int m = 0;
	      for(unsigned int i = 0; i < key_frame_set.size(); ++i)
	        {

		  float cos_k = cos(key_pose_[i].theta);
		  float sin_k = sin(key_pose_[i].theta);

		  //std::cout << "weizi: (" << key_pose_[i].x << ", " << key_pose_[i].y << ", " << key_pose_[i].theta * 180 / M_PI << ")" << std::endl;


		  for(unsigned int j = 0; j < key_frame_set[i].width; ++j)
		    {
		      cloud_pl->points[m].x = cos_k * key_frame_set[i].points[j].x - sin_k * key_frame_set[i].points[j].y + key_pose_[i].x;
		      cloud_pl->points[m].y = sin_k * key_frame_set[i].points[j].x + cos_k * key_frame_set[i].points[j].y + key_pose_[i].y;
		      cloud_pl->points[m].z = 0.0;
		      m++;
		    }
	        }
	      cloud_pl->width = key_frame_set.size() * msg->ranges.size();
	      cloud_pl->height = 1;
	      cloud_pl->is_dense = true;
	      pcl_conversions::toPCL(msg->header, cloud_pl->header);
	      cloud_pl->header.frame_id = msg->header.frame_id;
           
	      sensor_msgs::PointCloud2 cloud_msgs;
	      pcl::toROSMsg(*cloud_pl, cloud_msgs);

	      //publish before graph optimization's key scan's point cloud
	      map_points_pub.publish(cloud_msgs);
            

	      //std::cout<<"cov:"<<*(result.cov_x_m)<<std::endl;
	      delta[0] = 0.0; delta[1] = 0.0; delta[2] = 0.0;
	      prev_delta[0] = 0.0; prev_delta[1] = 0.0; prev_delta[2] = 0.0;
	      corres_ratio = 0.0;
	      ld_free(laser_ref);
	      laser_ref = laser_sens;
	    }
	  prev_delta[0] = delta[0]; prev_delta[1] = delta[1]; prev_delta[2] = delta[2];
        }
    }     
}

//accomplish one frame laser scan data process
void icp::laserScanToLDP(const sensor_msgs::LaserScan msg, LDP& laser)
{
    int step =1;//对于2400个点，进行1/3下采样，得到800个点，进行ICP解算

    double delta_a = msg.angle_increment;

    laser = ld_alloc_new(msg.ranges.size()/step);

    laser->nrays = msg.ranges.size()/step;

    int num_rays = 0;
    //laser->nrays = msg->ranges.size();
    for (int i=0; i< msg.ranges.size(); i += step) 
    {
        double r = msg.ranges[i];
        if ((r > msg.range_min) && (r < msg.range_max))
        {
            laser->readings[num_rays] = r;
            laser->valid[num_rays] = 1;
            laser->alpha[num_rays] = laser_incidence_angle->alpha[i];
        }else{
            laser->readings[num_rays] = 0;
            laser->valid[num_rays] = 0;
            laser->alpha[num_rays] = GSL_NAN;
        }
        laser->theta[num_rays] = (double)(msg.angle_min + i * delta_a);
        laser->cluster[num_rays] = -1;
        num_rays++;
    }

    laser->max_theta = laser->theta[num_rays - 1];
    laser->min_theta = laser->theta[0];
    laser->odometry[0] = 0.0;
    laser->odometry[1] = 0.0;
    laser->odometry[2] = 0.0;

    laser->true_pose[0] = 0.0;
    laser->true_pose[1] = 0.0;
    laser->true_pose[2] = 0.0;
}

void icp::laserScanToLDP(const sensor_msgs::LaserScan::ConstPtr& msg, LDP& laser)
{
    int step =1;//对于2400个点，进行1/3下采样，得到800个点，进行ICP解算

    double delta_a = msg->angle_increment;

    laser = ld_alloc_new(msg->ranges.size()/step);

    laser->nrays = msg->ranges.size()/step;

    int num_rays = 0;
    //laser->nrays = msg->ranges.size();
    for (int i=0; i< msg->ranges.size(); i += step) 
    {
        double r = msg->ranges[i];
        if ((r > msg->range_min) && (r < msg->range_max))
        {
            laser->readings[num_rays] = r;
            laser->valid[num_rays] = 1;
            laser->alpha[num_rays] = laser_incidence_angle->alpha[i];
        }else{
            laser->readings[num_rays] = 0;
            laser->valid[num_rays] = 0;
            laser->alpha[num_rays] = GSL_NAN;
        }
        laser->theta[num_rays] = (double)(msg->angle_min + i * delta_a);
        laser->cluster[num_rays] = -1;
        num_rays++;
    }

    laser->max_theta = laser->theta[num_rays - 1];
    laser->min_theta = laser->theta[0];
    laser->odometry[0] = 0.0;
    laser->odometry[1] = 0.0;
    laser->odometry[2] = 0.0;

    laser->true_pose[0] = 0.0;
    laser->true_pose[1] = 0.0;
    laser->true_pose[2] = 0.0;
}

//calculate every valid point of one frame laser scan's normal vector 
void icp::CalcuNormalVector(const sensor_msgs::LaserScan::ConstPtr& msg)
{
    laser_incidence_angle = ld_alloc_new(msg->ranges.size());
    laser_incidence_angle->nrays = msg->ranges.size();
    double delta_a = msg->angle_increment;

    double Roi_dist = msg->range_max;//only ROI-Dist take care of

    for(int i=0;i<laser_incidence_angle->nrays;i++)
    {
        int index_down = i-1 < 0 ? (msg->ranges.size()-1):(i-1);
        int index_up = i+1 > (msg->ranges.size()-1) ? 0 : (i+1);
        double j = msg->ranges[i];
        double j_d = msg->ranges[index_down];
        double j_u = msg->ranges[index_up];
        laser_incidence_angle->theta[i] = msg->angle_min + i*msg->angle_increment;

        if(!distance_valid(j,msg->range_min,msg->range_max))//该束激光距离异常，相应地将其对应的法向量置为无效
        {
            laser_incidence_angle->alpha_valid[i] = 0;
            laser_incidence_angle->alpha[i] = GSL_NAN;
            
        }
        else//该激光束距离有效:待进一步判断
        {
            double threshold = 1.5 * j * delta_a;//相邻两束激光打到同一物体上的距离判断阈值
            laser_incidence_angle->readings[i] = j;

            if(!distance_valid(j_d,msg->range_min,msg->range_max) && 
               !distance_valid(j_u,msg->range_min,msg->range_max))//该激光束左右两侧均为无效点，那么其也无效
               {
                   laser_incidence_angle->alpha_valid[i] = 0;
                   laser_incidence_angle->alpha[i] = GSL_NAN;
               }
            else if(!distance_valid(j_d,msg->range_min,msg->range_max) &&
                     distance_valid(j_u,msg->range_min,msg->range_max))//DOWN点无效，UP点有效
                     {
                         double j_x = j * cos(laser_incidence_angle->theta[i]);
                         double j_y = j * sin(laser_incidence_angle->theta[i]);
                         double ju_x = j_u * cos(laser_incidence_angle->theta[i]+delta_a);
                         double ju_y = j_u * sin(laser_incidence_angle->theta[i]+delta_a);
                         double dist = distance(j_x,j_y,ju_x,ju_y);
                         
                         if(dist > threshold)
                         {
                             laser_incidence_angle->alpha_valid[i] = 0;
                             laser_incidence_angle->alpha[i] = GSL_NAN;
                         }
                         else
                         {
                             laser_incidence_angle->alpha_valid[i] = 1;
                             //注意，此处需要用UP点的坐标减去当前点(CP)的坐标，得到的向量是CP->UP向量
                             //再将上述向量与X轴的夹角逆时针旋转90度，得到该激光束的法向量与X轴的夹角;
                             //角度需要转换到[-PI,PI]
                             double incidenct_angle = atan2(ju_y-j_y,ju_x-j_x) + M_PI/2;
                             laser_incidence_angle->alpha[i] = atan2(sin(incidenct_angle),cos(incidenct_angle));
                         }
                     }
            else if(distance_valid(j_d,msg->range_min,msg->range_max) &&
                    !distance_valid(j_u,msg->range_min,msg->range_max))//DOWN点有效，UP点无效
                    {
                        double j_x = j * cos(laser_incidence_angle->theta[i]);
                        double j_y = j * sin(laser_incidence_angle->theta[i]);
                        double jd_x = j_d * cos(laser_incidence_angle->theta[i]-delta_a);
                        double jd_y = j_d * sin(laser_incidence_angle->theta[i]-delta_a);
                        double dist = distance(j_x,j_y,jd_x,jd_y);

                         if(dist > threshold)
                         {
                             laser_incidence_angle->alpha_valid[i] = 0;
                             laser_incidence_angle->alpha[i] = GSL_NAN;
                         }
                         else
                         {
                             laser_incidence_angle->alpha_valid[i] = 1;
                             //注意，此处需要用CP点的坐标减去DOWN的坐标，得到的向量是DOWN->CP向量
                             //再将上述向量与X轴的夹角逆时针旋转90度，得到该激光束的法向量与X轴的夹角;
                             //角度需要转换到[-PI,PI]
                             double incidenct_angle = atan2(j_y-jd_y,j_x-jd_x) + M_PI/2;
                             laser_incidence_angle->alpha[i] = atan2(sin(incidenct_angle),cos(incidenct_angle));
                         }
                    }

            else if(distance_valid(j_d,msg->range_min,msg->range_max) &&
                    distance_valid(j_u,msg->range_min,msg->range_max))//均有效
                    {
                        double j_x = j * cos(laser_incidence_angle->theta[i]);
                        double j_y = j * sin(laser_incidence_angle->theta[i]);
                        double jd_x = j_d * cos(laser_incidence_angle->theta[i] - delta_a);//此处有BUG～
                        double jd_y = j_d * sin(laser_incidence_angle->theta[i] - delta_a);
                        double ju_x = j_u * cos(laser_incidence_angle->theta[i] + delta_a);
                        double ju_y = j_u * sin(laser_incidence_angle->theta[i] + delta_a);

                        double dist2up = distance(j_x,j_y,ju_x,ju_y);
                        double dist2down = distance(j_x,j_y,jd_x,jd_y);

                        if(dist2up < threshold)
                        {
                            laser_incidence_angle->alpha_valid[i] = 1;
                            //注意，此处需要用UP点的坐标减去当前点(CP)的坐标，得到的向量是CP->UP向量
                            //再将上述向量与X轴的夹角逆时针旋转90度，得到该激光束的法向量与X轴的夹角;
                            //角度需要转换到[-PI,PI]
                            double incidenct_angle = atan2(ju_y-j_y,ju_x-j_x) + M_PI/2;
                            laser_incidence_angle->alpha[i] = atan2(sin(incidenct_angle),cos(incidenct_angle));
                        }
                        else if(dist2down < threshold)
                        {
                            laser_incidence_angle->alpha_valid[i] = 1;
                            //注意，此处需要用CP点的坐标减去DOWN的坐标，得到的向量是DOWN->CP向量
                            //再将上述向量与X轴的夹角逆时针旋转90度，得到该激光束的法向量与X轴的夹角;
                            //角度需要转换到[-PI,PI]
                            double incidenct_angle = atan2(j_y-jd_y,j_x-jd_x) + M_PI/2;
                            laser_incidence_angle->alpha[i] = atan2(sin(incidenct_angle),cos(incidenct_angle));
                        }
                        else
                        {
                            laser_incidence_angle->alpha_valid[i] = 0;
                            laser_incidence_angle->alpha[i] = GSL_NAN;
                        }

                    }
        }
        
    }

}



pcl::PointCloud<PointT> icp::ConvertScan2PointCloud(const sensor_msgs::LaserScan::ConstPtr &scan_msg)
{
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

    // 将scan_msg的消息头 赋值到 pcl::PointCloud<pcl::PointXYZ>的消息头
    pcl_conversions::toPCL(scan_msg->header, cloud_msg->header);


    return *cloud_msg;
    // 将转换完的数据赋值到current_pointcloud_中保存下来
    //*current_pointcloud_ = *filtered;
}



int main(int argc , char** argv) 
{
    ros::init(argc,argv,"lidarodom");
    ros::NodeHandle nh;

    //实例化两个对象，都同时调用icp库，测试是否冲突!
    icp icp1;
    //icp icp2;

    ros::Rate r(50.0);
    while(nh.ok())
    {
        ros::spinOnce();
        r.sleep();
    }
    return 0;
}
