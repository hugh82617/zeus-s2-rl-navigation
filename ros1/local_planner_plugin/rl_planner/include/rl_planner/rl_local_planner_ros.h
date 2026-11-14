#ifndef RL_PLANNER_RL_LOCAL_PLANNER_ROS_H
#define RL_PLANNER_RL_LOCAL_PLANNER_ROS_H

#include <array>
#include <deque>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <costmap_2d/costmap_2d_ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <nav_core/base_local_planner.h>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Float32MultiArray.h>
#include <tf2_ros/buffer.h>

namespace rl_planner
{

constexpr int kLidarFrameDim = 181;
constexpr int kLidarFrames = 3;
constexpr int kLidarTotalDim = kLidarFrameDim * kLidarFrames;
constexpr int kObsDim = 556;

struct RLLocalPlannerParams
{
  std::string scan_topic = "/scan";
  std::string speed_topic = "/zeus_speed";
  std::string model_path = "/home/ros/catkin_ws/src/rl_planner/models/model_1200_actor.onnx";
  std::string input_name = "obs";
  std::string output_name = "action";
  std::string out_cmd_topic = "/rl_cmd_vel";
  std::string debug_goal_topic = "/rl_local_goal_body";
  std::string debug_goal_status_topic = "/rl_local_goal_status";
  std::string debug_lidar_topic = "/rl/lidar_543";
  std::string debug_obs_topic = "/rl/obs_556";
  std::string base_frame = "base_footprint";
  std::string global_frame = "map";

  double lookahead_dist = 0.8;
  double goal_tolerance = 0.1;
  double yaw_goal_tolerance = 0.1;
  double tf_timeout = 0.1;
  double max_distance = 12.0;
  double range_min = 0.05;
  double fov_min_deg = -135.0;
  double fov_max_deg = 135.0;
  double action_clip_low = -1.0;
  double action_clip_hi = 1.0;
  double linear_min = -0.2;
  double linear_max = 0.3;
  double angular_min = -0.5;
  double angular_max = 0.8;
  double action_bias_w = -0.13;
  bool rotate_half_turn = true;
  bool use_action_clip = true;
  bool publish_debug_topics = true;
};

class LidarPipeline
{
public:
  void initialize(const ros::NodeHandle& nh, const RLLocalPlannerParams& params);
  bool hasLatest() const;
  bool getLatest(std::array<float, kLidarTotalDim>& lidar, ros::Time& stamp) const;

private:
  void scanCallback(const sensor_msgs::LaserScanConstPtr& msg);
  std::vector<float> rotateHalfTurn(const std::vector<float>& values) const;
  std::array<float, kLidarFrameDim> resample181(const sensor_msgs::LaserScan& scan,
                                                 const std::vector<float>& ranges) const;
  static float median3(float a, float b, float c);

  mutable std::mutex mutex_;
  ros::Subscriber scan_sub_;
  RLLocalPlannerParams params_;
  std::deque<std::array<float, kLidarFrameDim> > frames_;
  std::array<float, kLidarTotalDim> latest_;
  ros::Time latest_stamp_;
  bool has_latest_ = false;
};

class WaypointExtractor
{
public:
  void setPlan(const std::vector<geometry_msgs::PoseStamped>& plan);
  bool getLocalGoal(tf2_ros::Buffer& tf,
                    const std::string& base_frame,
                    double lookahead_dist,
                    double goal_tolerance,
                    double yaw_goal_tolerance,
                    double tf_timeout,
                    std::array<float, 4>& goal4,
                    geometry_msgs::PoseStamped& goal_pose_base,
                    bool& final_goal_reached);
  bool getStoredLocalTarget(geometry_msgs::PoseStamped& local_target) const;
  bool hasValidLocalTarget() const;
  bool isGoalReached() const;

private:
  mutable std::mutex mutex_;
  std::vector<geometry_msgs::PoseStamped> global_plan_;
  geometry_msgs::PoseStamped local_target_;
  int pruned_start_idx_ = 0;
  bool local_target_valid_ = false;
  bool goal_reached_ = false;
};

class OnnxPolicy;

class RLLocalPlannerROS : public nav_core::BaseLocalPlanner
{
public:
  RLLocalPlannerROS();
  ~RLLocalPlannerROS() override;

  void initialize(std::string name, tf2_ros::Buffer* tf,
                  costmap_2d::Costmap2DROS* costmap_ros) override;
  bool setPlan(const std::vector<geometry_msgs::PoseStamped>& orig_global_plan) override;
  bool computeVelocityCommands(geometry_msgs::Twist& cmd_vel) override;
  bool isGoalReached() override;

private:
  void loadParams(const ros::NodeHandle& private_nh);
  void speedCallback(const geometry_msgs::TwistConstPtr& msg);
  std_msgs::Float32MultiArray makeFloatArray(const std::vector<float>& values,
                                             const std::string& label) const;
  std_msgs::Float32MultiArray makeFloatArray(const std::array<float, kLidarTotalDim>& values,
                                             const std::string& label) const;
  std_msgs::Float32MultiArray makeGoalMsg(const std::array<float, 4>& goal4) const;
  std_msgs::Float32MultiArray makeObsMsg(const geometry_msgs::Twist& speed,
                                         const std::array<float, 4>& goal4,
                                         const std::array<float, kLidarTotalDim>& lidar) const;

  bool initialized_ = false;
  std::string name_;
  tf2_ros::Buffer* tf_ = nullptr;
  costmap_2d::Costmap2DROS* costmap_ros_ = nullptr;

  RLLocalPlannerParams params_;
  LidarPipeline lidar_pipeline_;
  WaypointExtractor waypoint_extractor_;
  std::unique_ptr<OnnxPolicy> policy_;

  ros::Subscriber speed_sub_;
  ros::Publisher goal_pub_;
  ros::Publisher goal_status_pub_;
  ros::Publisher cmd_pub_;
  ros::Publisher lidar_debug_pub_;
  ros::Publisher obs_debug_pub_;

  mutable std::mutex speed_mutex_;
  geometry_msgs::Twist latest_speed_;
  ros::Time latest_speed_stamp_;
  bool has_speed_ = false;
};

}  // namespace rl_planner

#endif  // RL_PLANNER_RL_LOCAL_PLANNER_ROS_H
