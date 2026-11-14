#include <rl_planner/rl_local_planner_ros.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

#include <angles/angles.h>
#include <geometry_msgs/TransformStamped.h>
#include <onnxruntime_cxx_api.h>
#include <pluginlib/class_list_macros.h>
#include <std_msgs/MultiArrayDimension.h>
#include <std_msgs/String.h>
#include <tf2/exceptions.h>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

namespace rl_planner
{
namespace
{

double clamp(double value, double low, double high)
{
  return std::max(low, std::min(value, high));
}

bool isFiniteAndInRange(float r, double range_min, double range_max)
{
  return std::isfinite(r) && r >= range_min && r <= range_max;
}

double planarDistance(const geometry_msgs::PoseStamped& pose)
{
  const double x = pose.pose.position.x;
  const double y = pose.pose.position.y;
  return std::sqrt(x * x + y * y);
}

double pointToSegmentDistanceSq(const geometry_msgs::PoseStamped& a,
                                const geometry_msgs::PoseStamped& b)
{
  const double ax = a.pose.position.x;
  const double ay = a.pose.position.y;
  const double bx = b.pose.position.x;
  const double by = b.pose.position.y;
  const double dx = bx - ax;
  const double dy = by - ay;
  const double len2 = dx * dx + dy * dy;
  if (len2 <= 1e-9)
  {
    return ax * ax + ay * ay;
  }

  const double t = clamp(((-ax) * dx + (-ay) * dy) / len2, 0.0, 1.0);
  const double closest_x = ax + t * dx;
  const double closest_y = ay + t * dy;
  return closest_x * closest_x + closest_y * closest_y;
}

}  // namespace

class OnnxPolicy
{
public:
  explicit OnnxPolicy(const RLLocalPlannerParams& params)
    : params_(params),
      env_(ORT_LOGGING_LEVEL_WARNING, "rl_local_planner"),
      session_options_(),
      session_(nullptr)
  {
    session_options_.SetIntraOpNumThreads(1);
    session_options_.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_EXTENDED);
    session_ = Ort::Session(env_, params_.model_path.c_str(), session_options_);

    Ort::AllocatorWithDefaultOptions allocator;
    const std::size_t input_count = session_.GetInputCount();
    const std::size_t output_count = session_.GetOutputCount();
    if (input_count == 0 || output_count == 0)
    {
      throw std::runtime_error("ONNX model has no input or output");
    }

    input_name_owned_ = session_.GetInputNameAllocated(0, allocator).get();
    output_name_owned_ = session_.GetOutputNameAllocated(0, allocator).get();
    if (!params_.input_name.empty())
    {
      input_name_owned_ = params_.input_name;
    }
    if (!params_.output_name.empty())
    {
      output_name_owned_ = params_.output_name;
    }

    input_names_.push_back(input_name_owned_.c_str());
    output_names_.push_back(output_name_owned_.c_str());

    std::array<float, kObsDim> dummy;
    dummy.fill(0.0f);
    geometry_msgs::Twist warmup_cmd;
    infer(dummy, warmup_cmd);
  }

  bool infer(const std::array<float, kObsDim>& obs, geometry_msgs::Twist& cmd)
  {
    std::array<int64_t, 2> input_shape = {{1, kObsDim}};
    Ort::MemoryInfo memory_info = Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault);
    Ort::Value input_tensor = Ort::Value::CreateTensor<float>(
        memory_info,
        const_cast<float*>(obs.data()),
        obs.size(),
        input_shape.data(),
        input_shape.size());

    std::vector<Ort::Value> outputs = session_.Run(
        Ort::RunOptions{nullptr},
        input_names_.data(),
        &input_tensor,
        1,
        output_names_.data(),
        1);

    if (outputs.empty() || !outputs[0].IsTensor())
    {
      ROS_WARN_THROTTLE(1.0, "[RLLocalPlannerROS] ONNX inference returned no tensor output");
      return false;
    }

    float* raw = outputs[0].GetTensorMutableData<float>();
    const auto info = outputs[0].GetTensorTypeAndShapeInfo();
    const std::size_t count = info.GetElementCount();
    if (count < 2)
    {
      ROS_WARN_THROTTLE(1.0, "[RLLocalPlannerROS] ONNX action dim %zu < 2", count);
      return false;
    }

    const double raw_v = static_cast<double>(raw[0]);
    const double raw_w = static_cast<double>(raw[1]) - params_.action_bias_w;

    double action_v = raw_v;
    double action_w = raw_w;
    if (params_.use_action_clip)
    {
      action_v = clamp(action_v, params_.action_clip_low, params_.action_clip_hi);
      action_w = clamp(action_w, params_.action_clip_low, params_.action_clip_hi);
    }

    const double denom = params_.action_clip_hi - params_.action_clip_low;
    if (denom <= 1e-9)
    {
      ROS_WARN_THROTTLE(1.0, "[RLLocalPlannerROS] invalid action clip range");
      return false;
    }

    const double alpha_v = (action_v - params_.action_clip_low) / denom;
    const double alpha_w = (action_w - params_.action_clip_low) / denom;
    const double cmd_v = params_.linear_min + alpha_v * (params_.linear_max - params_.linear_min);
    const double cmd_w = params_.angular_min + alpha_w * (params_.angular_max - params_.angular_min);

    cmd = geometry_msgs::Twist();
    cmd.linear.x = clamp(cmd_v, params_.linear_min, params_.linear_max);
    cmd.angular.z = clamp(cmd_w, params_.angular_min, params_.angular_max);
    return true;
  }

private:
  RLLocalPlannerParams params_;
  Ort::Env env_;
  Ort::SessionOptions session_options_;
  Ort::Session session_;
  std::string input_name_owned_;
  std::string output_name_owned_;
  std::vector<const char*> input_names_;
  std::vector<const char*> output_names_;
};

void LidarPipeline::initialize(const ros::NodeHandle& nh, const RLLocalPlannerParams& params)
{
  params_ = params;
  frames_.clear();
  latest_.fill(static_cast<float>(params_.max_distance));
  latest_stamp_ = ros::Time(0);
  has_latest_ = false;
  scan_sub_ = nh.subscribe(params_.scan_topic, 5, &LidarPipeline::scanCallback, this);
}

bool LidarPipeline::hasLatest() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return has_latest_;
}

bool LidarPipeline::getLatest(std::array<float, kLidarTotalDim>& lidar, ros::Time& stamp) const
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (!has_latest_)
  {
    return false;
  }
  lidar = latest_;
  stamp = latest_stamp_;
  return true;
}

std::vector<float> LidarPipeline::rotateHalfTurn(const std::vector<float>& values) const
{
  if (!params_.rotate_half_turn || values.empty())
  {
    return values;
  }
  const std::size_t half = values.size() / 2;
  std::vector<float> out;
  out.reserve(values.size());
  out.insert(out.end(), values.begin() + half, values.end());
  out.insert(out.end(), values.begin(), values.begin() + half);
  return out;
}

std::array<float, kLidarFrameDim> LidarPipeline::resample181(
    const sensor_msgs::LaserScan& scan,
    const std::vector<float>& ranges) const
{
  std::array<float, kLidarFrameDim> out;
  const double fov_min = params_.fov_min_deg * M_PI / 180.0;
  const double fov_max = params_.fov_max_deg * M_PI / 180.0;
  const double cur_min = std::max(params_.range_min, static_cast<double>(scan.range_min));
  double cur_max = params_.max_distance;
  if (std::isfinite(scan.range_max) && scan.range_max > 0.0)
  {
    cur_max = std::min(params_.max_distance, static_cast<double>(scan.range_max));
  }
  if (cur_max <= cur_min)
  {
    cur_max = params_.max_distance;
  }
  const float invalid_fill = static_cast<float>(0.5 * params_.max_distance);

  if (ranges.empty() || scan.angle_increment <= 0.0)
  {
    out.fill(static_cast<float>(invalid_fill / params_.max_distance));
    return out;
  }

  std::vector<float> repaired(ranges);
  for (std::size_t i = 0; i < repaired.size(); ++i)
  {
    if (!isFiniteAndInRange(repaired[i], cur_min, cur_max))
    {
      float left = std::numeric_limits<float>::quiet_NaN();
      float right = std::numeric_limits<float>::quiet_NaN();
      for (std::size_t j = i; j > 0; --j)
      {
        if (isFiniteAndInRange(repaired[j - 1], cur_min, cur_max))
        {
          left = repaired[j - 1];
          break;
        }
      }
      for (std::size_t j = i + 1; j < repaired.size(); ++j)
      {
        if (isFiniteAndInRange(repaired[j], cur_min, cur_max))
        {
          right = repaired[j];
          break;
        }
      }
      if (std::isfinite(left) && std::isfinite(right))
      {
        repaired[i] = 0.5f * (left + right);
      }
      else if (std::isfinite(left))
      {
        repaired[i] = left;
      }
      else if (std::isfinite(right))
      {
        repaired[i] = right;
      }
      else
      {
        repaired[i] = invalid_fill;
      }
    }
    repaired[i] = static_cast<float>(clamp(repaired[i], cur_min, cur_max));
  }

  for (int k = 0; k < kLidarFrameDim; ++k)
  {
    const double alpha = static_cast<double>(k) / static_cast<double>(kLidarFrameDim - 1);
    const double angle = fov_min + alpha * (fov_max - fov_min);
    const double pos = (angle - scan.angle_min) / scan.angle_increment;

    float r = invalid_fill;
    if (pos <= 0.0)
    {
      r = repaired.front();
    }
    else if (pos >= static_cast<double>(repaired.size() - 1))
    {
      r = repaired.back();
    }
    else
    {
      const int i0 = static_cast<int>(std::floor(pos));
      const int i1 = i0 + 1;
      const double w = pos - static_cast<double>(i0);
      r = static_cast<float>((1.0 - w) * repaired[i0] + w * repaired[i1]);
    }
    out[k] = static_cast<float>(clamp(r / params_.max_distance, 0.0, 1.0));
  }
  return out;
}

float LidarPipeline::median3(float a, float b, float c)
{
  if (a > b) std::swap(a, b);
  if (b > c) std::swap(b, c);
  if (a > b) std::swap(a, b);
  return b;
}

void LidarPipeline::scanCallback(const sensor_msgs::LaserScanConstPtr& msg)
{
  std::vector<float> ranges = rotateHalfTurn(std::vector<float>(msg->ranges.begin(), msg->ranges.end()));
  const std::array<float, kLidarFrameDim> frame = resample181(*msg, ranges);

  std::lock_guard<std::mutex> lock(mutex_);
  frames_.push_front(frame);
  while (frames_.size() < kLidarFrames)
  {
    frames_.push_back(frame);
  }
  while (frames_.size() > kLidarFrames)
  {
    frames_.pop_back();
  }

  for (int f = 0; f < kLidarFrames; ++f)
  {
    for (int k = 0; k < kLidarFrameDim; ++k)
    {
      float value = frames_[f][k];
      if (f == 0 && frames_.size() >= 3)
      {
        value = median3(frames_[0][k], frames_[1][k], frames_[2][k]);
      }
      latest_[f * kLidarFrameDim + k] = value;
    }
  }
  latest_stamp_ = msg->header.stamp.isZero() ? ros::Time::now() : msg->header.stamp;
  has_latest_ = true;
}

void WaypointExtractor::setPlan(const std::vector<geometry_msgs::PoseStamped>& plan)
{
  std::lock_guard<std::mutex> lock(mutex_);
  global_plan_ = plan;
  local_target_ = geometry_msgs::PoseStamped();
  pruned_start_idx_ = 0;
  local_target_valid_ = false;
  goal_reached_ = false;
}

bool WaypointExtractor::getLocalGoal(tf2_ros::Buffer& tf,
                                     const std::string& base_frame,
                                     double lookahead_dist,
                                     double goal_tolerance,
                                     double yaw_goal_tolerance,
                                     double tf_timeout,
                                     std::array<float, 4>& goal4,
                                     geometry_msgs::PoseStamped& goal_pose_base,
                                     bool& final_goal_reached)
{
  std::vector<geometry_msgs::PoseStamped> plan;
  int pruned_start_idx = 0;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    plan = global_plan_;
    pruned_start_idx = pruned_start_idx_;
  }

  final_goal_reached = false;
  if (plan.empty())
  {
    std::lock_guard<std::mutex> lock(mutex_);
    local_target_ = geometry_msgs::PoseStamped();
    pruned_start_idx_ = 0;
    local_target_valid_ = false;
    goal_reached_ = false;
    return false;
  }

  const int window_start = std::max(0, std::min(pruned_start_idx - 5,
                                               static_cast<int>(plan.size()) - 1));
  std::vector<geometry_msgs::PoseStamped> base_plan;
  base_plan.reserve(plan.size() - static_cast<std::size_t>(window_start));
  for (std::size_t i = static_cast<std::size_t>(window_start); i < plan.size(); ++i)
  {
    try
    {
      geometry_msgs::PoseStamped pose_for_tf = plan[i];
      pose_for_tf.header.stamp = ros::Time(0);
      geometry_msgs::PoseStamped transformed;
      tf.transform(pose_for_tf, transformed, base_frame, ros::Duration(tf_timeout));
      base_plan.push_back(transformed);
    }
    catch (const tf2::TransformException& ex)
    {
      ROS_WARN_THROTTLE(1.0, "[RLLocalPlannerROS] failed to transform global plan pose to %s: %s",
                        base_frame.c_str(), ex.what());
      std::lock_guard<std::mutex> lock(mutex_);
      local_target_ = geometry_msgs::PoseStamped();
      local_target_valid_ = false;
      goal_reached_ = false;
      return false;
    }
  }

  if (base_plan.empty())
  {
    std::lock_guard<std::mutex> lock(mutex_);
    local_target_ = geometry_msgs::PoseStamped();
    local_target_valid_ = false;
    goal_reached_ = false;
    return false;
  }

  const double final_dist = planarDistance(base_plan.back());
  const double final_yaw = angles::normalize_angle(tf2::getYaw(base_plan.back().pose.orientation));
  if (final_dist <= goal_tolerance)
  {
    goal_pose_base = base_plan.back();
    goal4 = {{static_cast<float>(goal_pose_base.pose.position.x),
              static_cast<float>(goal_pose_base.pose.position.y),
              static_cast<float>(goal_pose_base.pose.position.z),
              static_cast<float>(final_yaw)}};

    std::lock_guard<std::mutex> lock(mutex_);
    local_target_ = goal_pose_base;
    pruned_start_idx_ = static_cast<int>(plan.size()) - 1;
    local_target_valid_ = true;
    if (std::abs(final_yaw) <= yaw_goal_tolerance)
    {
      goal_reached_ = true;
      final_goal_reached = true;
      goal4 = {{0.0f, 0.0f, 0.0f, 0.0f}};
    }
    else
    {
      goal_reached_ = false;
    }
    return true;
  }

  int nearest_idx = 0;
  double nearest_dist = std::numeric_limits<double>::infinity();
  if (base_plan.size() == 1)
  {
    nearest_dist = planarDistance(base_plan.front());
  }
  else
  {
    for (std::size_t i = 0; i + 1 < base_plan.size(); ++i)
    {
      const double d = pointToSegmentDistanceSq(base_plan[i], base_plan[i + 1]);
      if (d < nearest_dist)
      {
        nearest_dist = d;
        nearest_idx = static_cast<int>(i);
      }
    }
  }

  int target_idx = nearest_idx;
  for (std::size_t i = nearest_idx; i < base_plan.size(); ++i)
  {
    const double x = base_plan[i].pose.position.x;
    const double d = planarDistance(base_plan[i]);
    if (x >= -0.05 && d >= lookahead_dist)
    {
      target_idx = static_cast<int>(i);
      break;
    }
    target_idx = static_cast<int>(i);
  }

  goal_pose_base = base_plan[target_idx];
  const double dx = goal_pose_base.pose.position.x;
  const double dy = goal_pose_base.pose.position.y;
  const double dz = goal_pose_base.pose.position.z;
  const double dyaw = std::atan2(dy, dx);
  goal4 = {{static_cast<float>(dx),
            static_cast<float>(dy),
            static_cast<float>(dz),
            static_cast<float>(angles::normalize_angle(dyaw))}};

  std::lock_guard<std::mutex> lock(mutex_);
  local_target_ = goal_pose_base;
  pruned_start_idx_ = window_start + nearest_idx;
  local_target_valid_ = true;
  goal_reached_ = false;
  return true;
}

bool WaypointExtractor::getStoredLocalTarget(geometry_msgs::PoseStamped& local_target) const
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (!local_target_valid_)
  {
    return false;
  }
  local_target = local_target_;
  return true;
}

bool WaypointExtractor::hasValidLocalTarget() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return local_target_valid_;
}

bool WaypointExtractor::isGoalReached() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return goal_reached_;
}

RLLocalPlannerROS::RLLocalPlannerROS() = default;

RLLocalPlannerROS::~RLLocalPlannerROS() = default;

void RLLocalPlannerROS::initialize(std::string name, tf2_ros::Buffer* tf,
                                   costmap_2d::Costmap2DROS* costmap_ros)
{
  if (initialized_)
  {
    ROS_WARN("[RLLocalPlannerROS] already initialized");
    return;
  }

  name_ = name;
  tf_ = tf;
  costmap_ros_ = costmap_ros;
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~/" + name);

  loadParams(private_nh);
  if (costmap_ros_ != nullptr)
  {
    params_.base_frame = costmap_ros_->getBaseFrameID();
    params_.global_frame = costmap_ros_->getGlobalFrameID();
  }

  try
  {
    policy_.reset(new OnnxPolicy(params_));
  }
  catch (const std::exception& ex)
  {
    ROS_FATAL("[RLLocalPlannerROS] failed to initialize ONNX policy: %s", ex.what());
    throw;
  }

  lidar_pipeline_.initialize(nh, params_);
  speed_sub_ = nh.subscribe(params_.speed_topic, 10, &RLLocalPlannerROS::speedCallback, this);

  goal_pub_ = nh.advertise<std_msgs::Float32MultiArray>(params_.debug_goal_topic, 10);
  goal_status_pub_ = nh.advertise<std_msgs::String>(params_.debug_goal_status_topic, 10);
  cmd_pub_ = nh.advertise<geometry_msgs::Twist>(params_.out_cmd_topic, 10);
  if (params_.publish_debug_topics)
  {
    lidar_debug_pub_ = nh.advertise<std_msgs::Float32MultiArray>(params_.debug_lidar_topic, 10);
    obs_debug_pub_ = nh.advertise<std_msgs::Float32MultiArray>(params_.debug_obs_topic, 10);
  }

  initialized_ = true;
  ROS_INFO("[RLLocalPlannerROS] initialized as %s: scan=%s model=%s base=%s global=%s",
           name_.c_str(),
           params_.scan_topic.c_str(),
           params_.model_path.c_str(),
           params_.base_frame.c_str(),
           params_.global_frame.c_str());
}

void RLLocalPlannerROS::loadParams(const ros::NodeHandle& private_nh)
{
  private_nh.param("scan_topic", params_.scan_topic, params_.scan_topic);
  private_nh.param("speed_topic", params_.speed_topic, params_.speed_topic);
  private_nh.param("model_path", params_.model_path, params_.model_path);
  private_nh.param("input_name", params_.input_name, params_.input_name);
  private_nh.param("output_name", params_.output_name, params_.output_name);
  private_nh.param("out_cmd_topic", params_.out_cmd_topic, params_.out_cmd_topic);
  private_nh.param("debug_goal_topic", params_.debug_goal_topic, params_.debug_goal_topic);
  private_nh.param("debug_goal_status_topic", params_.debug_goal_status_topic, params_.debug_goal_status_topic);
  private_nh.param("debug_lidar_topic", params_.debug_lidar_topic, params_.debug_lidar_topic);
  private_nh.param("debug_obs_topic", params_.debug_obs_topic, params_.debug_obs_topic);
  private_nh.param("base_frame", params_.base_frame, params_.base_frame);
  private_nh.param("global_frame", params_.global_frame, params_.global_frame);
  private_nh.param("lookahead_dist", params_.lookahead_dist, params_.lookahead_dist);
  private_nh.param("goal_tolerance", params_.goal_tolerance, params_.goal_tolerance);
  private_nh.param("yaw_goal_tolerance", params_.yaw_goal_tolerance, params_.yaw_goal_tolerance);
  private_nh.param("tf_timeout", params_.tf_timeout, params_.tf_timeout);
  private_nh.param("max_distance", params_.max_distance, params_.max_distance);
  private_nh.param("range_min", params_.range_min, params_.range_min);
  private_nh.param("fov_min_deg", params_.fov_min_deg, params_.fov_min_deg);
  private_nh.param("fov_max_deg", params_.fov_max_deg, params_.fov_max_deg);
  private_nh.param("action_clip_low", params_.action_clip_low, params_.action_clip_low);
  private_nh.param("action_clip_hi", params_.action_clip_hi, params_.action_clip_hi);
  private_nh.param("linear_min", params_.linear_min, params_.linear_min);
  private_nh.param("linear_max", params_.linear_max, params_.linear_max);
  private_nh.param("angular_min", params_.angular_min, params_.angular_min);
  private_nh.param("angular_max", params_.angular_max, params_.angular_max);
  private_nh.param("action_bias_w", params_.action_bias_w, params_.action_bias_w);
  private_nh.param("rotate_half_turn", params_.rotate_half_turn, params_.rotate_half_turn);
  private_nh.param("use_action_clip", params_.use_action_clip, params_.use_action_clip);
  private_nh.param("publish_debug_topics", params_.publish_debug_topics, params_.publish_debug_topics);
}

bool RLLocalPlannerROS::setPlan(const std::vector<geometry_msgs::PoseStamped>& orig_global_plan)
{
  if (!initialized_)
  {
    ROS_ERROR("[RLLocalPlannerROS] setPlan called before initialize");
    return false;
  }
  waypoint_extractor_.setPlan(orig_global_plan);
  std_msgs::String status;
  status.data = "goal_received";
  goal_status_pub_.publish(status);
  return true;
}

void RLLocalPlannerROS::speedCallback(const geometry_msgs::TwistConstPtr& msg)
{
  std::lock_guard<std::mutex> lock(speed_mutex_);
  latest_speed_ = *msg;
  latest_speed_stamp_ = ros::Time::now();
  has_speed_ = true;
}

std_msgs::Float32MultiArray RLLocalPlannerROS::makeFloatArray(const std::vector<float>& values,
                                                              const std::string& label) const
{
  std_msgs::Float32MultiArray msg;
  std_msgs::MultiArrayDimension dim;
  dim.label = label;
  dim.size = values.size();
  dim.stride = values.size();
  msg.layout.dim.push_back(dim);
  msg.layout.data_offset = 0;
  msg.data = values;
  return msg;
}

std_msgs::Float32MultiArray RLLocalPlannerROS::makeFloatArray(
    const std::array<float, kLidarTotalDim>& values,
    const std::string& label) const
{
  std::vector<float> vec(values.begin(), values.end());
  return makeFloatArray(vec, label);
}

std_msgs::Float32MultiArray RLLocalPlannerROS::makeGoalMsg(const std::array<float, 4>& goal4) const
{
  return makeFloatArray(std::vector<float>(goal4.begin(), goal4.end()), "goal4");
}

std_msgs::Float32MultiArray RLLocalPlannerROS::makeObsMsg(
    const geometry_msgs::Twist& speed,
    const std::array<float, 4>& goal4,
    const std::array<float, kLidarTotalDim>& lidar) const
{
  std::vector<float> obs(kObsDim, 0.0f);
  obs[0] = static_cast<float>(speed.linear.x);
  obs[1] = static_cast<float>(speed.linear.y);
  obs[2] = static_cast<float>(speed.linear.z);
  obs[3] = static_cast<float>(speed.angular.x);
  obs[4] = static_cast<float>(speed.angular.y);
  obs[5] = static_cast<float>(speed.angular.z);
  obs[6] = 0.0f;
  obs[7] = 0.0f;
  obs[8] = -1.0f;
  for (int i = 0; i < 4; ++i)
  {
    obs[9 + i] = goal4[i];
  }
  for (int i = 0; i < kLidarTotalDim; ++i)
  {
    obs[13 + i] = lidar[i];
  }
  return makeFloatArray(obs, "obs_556");
}

bool RLLocalPlannerROS::computeVelocityCommands(geometry_msgs::Twist& cmd_vel)
{
  if (!initialized_ || tf_ == nullptr)
  {
    ROS_ERROR_THROTTLE(1.0, "[RLLocalPlannerROS] computeVelocityCommands called before initialize");
    return false;
  }

  std::array<float, 4> goal4;
  geometry_msgs::PoseStamped goal_pose_base;
  bool final_goal_reached = false;
  if (!waypoint_extractor_.getLocalGoal(*tf_, params_.base_frame, params_.lookahead_dist,
                                        params_.goal_tolerance, params_.yaw_goal_tolerance,
                                        params_.tf_timeout,
                                        goal4, goal_pose_base, final_goal_reached))
  {
    cmd_vel = geometry_msgs::Twist();
    return false;
  }

  std_msgs::String status;
  status.data = final_goal_reached ? "goal_reached" : "goal_tracking";
  goal_status_pub_.publish(status);
  goal_pub_.publish(makeGoalMsg(goal4));

  if (final_goal_reached)
  {
    cmd_vel = geometry_msgs::Twist();
    cmd_pub_.publish(cmd_vel);
    return true;
  }

  std::array<float, kLidarTotalDim> lidar;
  ros::Time lidar_stamp;
  if (!lidar_pipeline_.getLatest(lidar, lidar_stamp))
  {
    ROS_WARN_THROTTLE(1.0, "[RLLocalPlannerROS] waiting for lidar data from %s",
                      params_.scan_topic.c_str());
    cmd_vel = geometry_msgs::Twist();
    return false;
  }

  if (params_.publish_debug_topics)
  {
    lidar_debug_pub_.publish(makeFloatArray(lidar, "lidar_543"));
  }

  geometry_msgs::Twist speed;
  {
    std::lock_guard<std::mutex> lock(speed_mutex_);
    if (has_speed_)
    {
      const double speed_age = (ros::Time::now() - latest_speed_stamp_).toSec();
      if (speed_age <= 0.5)
      {
        speed = latest_speed_;
      }
      else
      {
        ROS_WARN_THROTTLE(2.0, "[RLLocalPlannerROS] /zeus_speed stale > 0.5s, using zero");
      }
    }
  }
  std::array<float, kObsDim> obs;
  obs.fill(0.0f);
  obs[0] = static_cast<float>(speed.linear.x);
  obs[1] = static_cast<float>(speed.linear.y);
  obs[2] = static_cast<float>(speed.linear.z);
  obs[3] = static_cast<float>(speed.angular.x);
  obs[4] = static_cast<float>(speed.angular.y);
  obs[5] = static_cast<float>(speed.angular.z);
  obs[6] = 0.0f;
  obs[7] = 0.0f;
  obs[8] = -1.0f;
  for (int i = 0; i < 4; ++i)
  {
    obs[9 + i] = goal4[i];
  }
  for (int i = 0; i < kLidarTotalDim; ++i)
  {
    obs[13 + i] = lidar[i];
  }

  if (params_.publish_debug_topics)
  {
    obs_debug_pub_.publish(makeObsMsg(speed, goal4, lidar));
  }

  if (!policy_ || !policy_->infer(obs, cmd_vel))
  {
    ROS_WARN_THROTTLE(1.0, "[RLLocalPlannerROS] ONNX policy inference failed");
    cmd_vel = geometry_msgs::Twist();
    cmd_pub_.publish(cmd_vel);
    return false;
  }

  cmd_pub_.publish(cmd_vel);
  return true;
}

bool RLLocalPlannerROS::isGoalReached()
{
  return initialized_ && waypoint_extractor_.isGoalReached();
}

}  // namespace rl_planner

PLUGINLIB_EXPORT_CLASS(rl_planner::RLLocalPlannerROS, nav_core::BaseLocalPlanner)
