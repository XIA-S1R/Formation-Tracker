// 分布式粒子滤波 (DPF) 仿真节点
// 100%对齐 Gu 2007 "Distributed Particle Filter for Target Tracking" Algorithm 1
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/time_synchronizer.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/MarkerArray.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <unordered_set>
#include <cmath>
#include <mutex>
#include <target_ekf/target_ekf.hpp>
#include <target_ekf/target_dpf.hpp>
#include <target_ekf/LocalStats.h>
#include <target_ekf/LabeledConsensusState.h>
#include <std_msgs/Bool.h>
#include <unordered_set>
#include <algorithm>
#include <numeric>
#include "target_ekf/search_particles_manager.hpp"

// === 全局变量 ===
// 发布器
ros::Publisher target_odom_pub_;
ros::Publisher local_stats_pub_;
ros::Publisher labeled_consensus_pub_;
ros::Publisher search_state_pub_;
ros::Publisher search_particles_vis_pub_;
ros::Publisher search_gmm_vis_pub_; // 搜索GMM分布可视化发布器
std::vector<ros::Subscriber> stats_subs_;

// 相机外参
Eigen::Matrix3d cam2body_R_;
Eigen::Vector3d cam2body_p_;
double fx_, fy_, cx_, cy_, width_, height_;
double pitch_thr_ = 30;
bool check_fov_ = false;

// 无人机自身位姿（odom回调更新）
std::mutex odom_mutex_;
Eigen::Vector3d latest_odom_pos_;
Eigen::Quaterniond latest_odom_q_;
bool has_latest_odom_ = false;

// 目标观测（yolo回调更新）
std::mutex obs_mutex_;
Eigen::Vector3d latest_obs_pos_;
Eigen::Vector3d latest_obs_rpy_;
bool has_latest_obs_ = false;
ros::Time latest_obs_stamp_;
double obs_timeout_ = 0.2; // 观测超时时间0.2s

// DPF核心
int drone_id_ = 0;
int num_drones_ = 3;
int dpf_rate_ = 20;
std::shared_ptr<DistributedPF> dpfPtr_;
ros::Time last_update_stamp_;
int dpf_reset_suppress_count_ = 0;

// 搜索模式相关
int miss_detection_num_ = 10; // 连续多少帧都没有观测后进入搜索模式
int consecutive_no_obs_count_ = 0; // 连续无观测计数
bool search_mode_active_ = false; // 是否处于搜索模式
ros::Time last_global_obs_time_;

// 搜索粒子管理器
std::unique_ptr<SearchParticlesManager> search_particles_manager_;

// 邻居数据存储
std::map<int, LocalStat> received_stats_;
std::map<int, NeighborConsensus> received_consensus_;
std::map<int, std::vector<LabeledNeighborConsensus>> received_labeled_consensus_;
std::mutex stats_mutex_;
std::mutex labeled_stats_mutex_;

// === 占据栅格 ===
struct SimpleOccMap {
  double resolution = 0.3;
  std::unordered_set<int64_t> occ_cells;
  bool received = false;
  mutable std::mutex map_mutex_;
  int64_t toKey(int x, int y, int z) const {
    return ((int64_t)(x + 32768) << 32) | ((int64_t)(y + 32768) << 16) | (int64_t)(z + 32768);
  }
  void fromPointCloud(const pcl::PointCloud<pcl::PointXYZ>& cloud, double res) {
    std::lock_guard<std::mutex> lock(map_mutex_);
    resolution = res;
    occ_cells.clear();
    for (const auto& pt : cloud) {
      int x = (int)std::floor(pt.x / resolution);
      int y = (int)std::floor(pt.y / resolution);
      int z = (int)std::floor(pt.z / resolution);
      occ_cells.insert(toKey(x, y, z));
    }
    received = true;
  }
  bool isOccupied(const Eigen::Vector3d& p) const {
    std::lock_guard<std::mutex> lock(map_mutex_);
    int x = (int)std::floor(p.x() / resolution);
    int y = (int)std::floor(p.y() / resolution);
    int z = (int)std::floor(p.z() / resolution);
    return occ_cells.count(toKey(x, y, z)) > 0;
  }
} occMap_;

// === 视距检查 ===
bool isLineOfSightClear(const Eigen::Vector3d& start, const Eigen::Vector3d& end) {
  if (!occMap_.received) return true;
  double dist = (end - start).norm();
  if (dist > 15.0) return false;
  int steps = std::max(1, (int)(dist / (occMap_.resolution * 0.5)));
  for (int i = 1; i < steps; ++i) {
    double t = (double)i / steps;
    Eigen::Vector3d pt = start + t * (end - start);
    if (occMap_.isOccupied(pt)) return false;
  }
  return true;
}

// === 全局地图回调 ===
void global_map_callback(const sensor_msgs::PointCloud2ConstPtr& msg) {
  pcl::PointCloud<pcl::PointXYZ> cloud;
  pcl::fromROSMsg(*msg, cloud);
  occMap_.fromPointCloud(cloud, 0.3);
  ROS_INFO_ONCE("[dpf%d] Global map received, %zu obstacle points.", drone_id_, cloud.size());
}

// === 发布搜索粒子可视化 ===
void publishSearchParticlesVisualization() {
  if (!search_particles_manager_ || !search_particles_manager_->isInitialized()) return;
  
  const auto& search_particles = search_particles_manager_->getSearchParticles();
  if (search_particles.empty()) return;
  
  // 创建点云
  pcl::PointCloud<pcl::PointXYZRGB> cloud;
  cloud.header.frame_id = "world";
  cloud.header.stamp = ros::Time::now().toNSec() / 1e3;
  
  for (const auto& particle : search_particles) {
    pcl::PointXYZRGB point;
    point.x = particle.state(0);
    point.y = particle.state(1);
    point.z = particle.state(2);
    
    // 根据粒子的意图标签设置颜色
    switch (particle.intent) {
      case STRAIGHT:
        // 蓝色 - 直行
        point.r = 0;
        point.g = 0;
        point.b = 255;
        break;
      case LEFT_TURN:
        // 红色 - 左转
        point.r = 255;
        point.g = 0;
        point.b = 0;
        break;
      case RIGHT_TURN:
        // 绿色 - 右转
        point.r = 0;
        point.g = 255;
        point.b = 0;
        break;
      default:
        // 白色 - 默认
        point.r = 255;
        point.g = 255;
        point.b = 255;
        break;
    }
    
    // 根据粒子权重调整亮度
    double weight = std::min(1.0, particle.weight * search_particles.size());
    point.r = static_cast<uint8_t>(point.r * weight);
    point.g = static_cast<uint8_t>(point.g * weight);
    point.b = static_cast<uint8_t>(point.b * weight);
    
    cloud.push_back(point);
  }
  
  // 转换为ROS消息并发布
  sensor_msgs::PointCloud2 cloud_msg;
  pcl::toROSMsg(cloud, cloud_msg);
  search_particles_vis_pub_.publish(cloud_msg);
}

// === 发布GMM分布可视化 ===
void publishGMMVisualization() {
  if (!search_particles_manager_ || !search_particles_manager_->isInitialized()) return;
  
  // 获取GMM参数
  auto gmm_means = search_particles_manager_->getLabeledGMMMeans();
  auto gmm_covs = search_particles_manager_->getLabeledGMMCovs();
  auto gmm_pis = search_particles_manager_->getLabeledGMMPis();
  
  visualization_msgs::MarkerArray marker_array;
  int marker_id = 0;
  
  // 为每个标签创建GMM可视化
  for (const auto& label_means : gmm_means) {
    SearchIntent label = label_means.first;
    const auto& means = label_means.second;
    
    // 获取该标签的协方差和混合权重
    auto covs_it = gmm_covs.find(label);
    auto pis_it = gmm_pis.find(label);
    if (covs_it == gmm_covs.end() || pis_it == gmm_pis.end()) continue;
    
    const auto& covs = covs_it->second;
    const auto& pis = pis_it->second;
    
    // 根据标签设置颜色
    std_msgs::ColorRGBA color;
    switch (label) {
      case STRAIGHT:
        // 蓝色 - 直行
        color.r = 0.0;
        color.g = 0.0;
        color.b = 1.0;
        break;
      case LEFT_TURN:
        // 红色 - 左转
        color.r = 1.0;
        color.g = 0.0;
        color.b = 0.0;
        break;
      case RIGHT_TURN:
        // 绿色 - 右转
        color.r = 0.0;
        color.g = 1.0;
        color.b = 0.0;
        break;
      default:
        // 白色 - 默认
        color.r = 1.0;
        color.g = 1.0;
        color.b = 1.0;
        break;
    }
    color.a = 0.5; // 设置透明度
    
    // 为每个GMM分量创建标记
    for (size_t i = 0; i < means.size() && i < covs.size(); ++i) {
      const auto& mean = means[i];
      const auto& cov = covs[i];
      double weight = (i < pis.size()) ? pis(i) : 1.0 / means.size();
      
      // 创建球体标记表示GMM均值
      visualization_msgs::Marker mean_marker;
      mean_marker.header.frame_id = "world";
      mean_marker.header.stamp = ros::Time::now();
      mean_marker.id = marker_id++;
      mean_marker.type = visualization_msgs::Marker::SPHERE;
      mean_marker.action = visualization_msgs::Marker::ADD;
      mean_marker.pose.position.x = mean(0);
      mean_marker.pose.position.y = mean(1);
      mean_marker.pose.position.z = mean(2);
      mean_marker.pose.orientation.w = 1.0;
      
      // 根据权重设置大小
      double scale = 0.3 * weight;
      mean_marker.scale.x = scale;
      mean_marker.scale.y = scale;
      mean_marker.scale.z = scale;
      
      mean_marker.color = color;
      marker_array.markers.push_back(mean_marker);
      
      // 创建箭头标记表示GMM分量的方向（基于速度）
      if (mean.size() >= 6) {
        visualization_msgs::Marker arrow_marker;
        arrow_marker.header.frame_id = "world";
        arrow_marker.header.stamp = ros::Time::now();
        arrow_marker.id = marker_id++;
        arrow_marker.type = visualization_msgs::Marker::ARROW;
        arrow_marker.action = visualization_msgs::Marker::ADD;
        arrow_marker.pose.position.x = mean(0);
        arrow_marker.pose.position.y = mean(1);
        arrow_marker.pose.position.z = mean(2);
        
        // 计算速度方向
        Eigen::Vector3d velocity(mean(3), mean(4), mean(5));
        double speed = velocity.norm();
        if (speed > 0.1) {
          velocity.normalize();
          
          // 计算箭头方向的四元数
          Eigen::Vector3d z_axis(0, 0, 1);
          Eigen::Vector3d direction = velocity;
          Eigen::Quaterniond q;
          q.setFromTwoVectors(z_axis, direction);
          arrow_marker.pose.orientation.x = q.x();
          arrow_marker.pose.orientation.y = q.y();
          arrow_marker.pose.orientation.z = q.z();
          arrow_marker.pose.orientation.w = q.w();
          
          // 设置箭头大小
          arrow_marker.scale.x = 0.1 * speed;
          arrow_marker.scale.y = 0.05;
          arrow_marker.scale.z = 0.05;
          
          arrow_marker.color = color;
          marker_array.markers.push_back(arrow_marker);
        }
      }
    }
  }
  
  // 发布GMM可视化
  search_gmm_vis_pub_.publish(marker_array);
}

// === LocalStats 消息 <-> 结构体转换 ===
LocalStat fromMsg(const target_ekf::LocalStats::ConstPtr& msg) {
  LocalStat ls;
  ls.drone_id = msg->drone_id;
  ls.C = msg->num_components;
  ls.nx = msg->state_dim; // 9维状态空间
  ls.has_obs = msg->has_observation;
  ls.timestamp = msg->header.stamp;
  ls.alpha = Eigen::Map<const Eigen::VectorXd>(msg->alpha_local.data(), ls.C);
  ls.a.resize(ls.C);
  ls.b.resize(ls.C);
  for (int c = 0; c < ls.C; ++c) {
    ls.a[c] = Eigen::Map<const Eigen::VectorXd>(msg->a_local.data() + c * ls.nx, ls.nx);
    ls.b[c] = Eigen::Map<const Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>(
        msg->b_local.data() + c * ls.nx * ls.nx, ls.nx, ls.nx);
  }
  return ls;
}

target_ekf::LocalStats toMsg(const LocalStat& ls, const DistributedPF& dpf) {
  target_ekf::LocalStats msg;
  msg.header.stamp = ls.timestamp.isZero() ? ros::Time::now() : ls.timestamp;
  msg.drone_id = ls.drone_id;
  msg.num_components = ls.C;
  msg.state_dim = ls.nx; // 9维状态空间
  msg.obs_dim = dpf.nz_;
  msg.has_observation = ls.has_obs;
  // 打包本地统计量u
  msg.alpha_local.resize(ls.C);
  msg.a_local.resize(ls.C * ls.nx);
  msg.b_local.resize(ls.C * ls.nx * ls.nx);
  for (int c = 0; c < ls.C; ++c) {
    msg.alpha_local[c] = ls.alpha(c);
    for (int i = 0; i < ls.nx; ++i) {
      msg.a_local[c * ls.nx + i] = ls.a[c](i);
    }
    for (int i = 0; i < ls.nx; ++i) {
      for (int j = 0; j < ls.nx; ++j) {
        msg.b_local[c * ls.nx * ls.nx + i * ls.nx + j] = ls.b[c](i, j);
      }
    }
  }
  // 打包共识状态ζ
  msg.zeta_alpha.resize(dpf.C_);
  msg.zeta_a.resize(dpf.C_ * dpf.nx_);
  msg.zeta_b.resize(dpf.C_ * dpf.nx_ * dpf.nx_);
  for (int c = 0; c < dpf.C_; ++c) {
    msg.zeta_alpha[c] = dpf.zeta_alpha_(c);
    for (int i = 0; i < dpf.nx_; ++i) {
      msg.zeta_a[c * dpf.nx_ + i] = dpf.zeta_a_[c](i);
      for (int j = 0; j < dpf.nx_; ++j) {
        msg.zeta_b[c * dpf.nx_ * dpf.nx_ + i * dpf.nx_ + j] = dpf.zeta_b_[c](i, j);
      }
    }
  }
  return msg;
}

// === 收到其他无人机局部统计量的回调 ===
void stats_callback(const target_ekf::LocalStats::ConstPtr& msg) {
  std::lock_guard<std::mutex> lock(stats_mutex_);
  received_stats_[msg->drone_id] = fromMsg(msg);
  
  NeighborConsensus nc;
  nc.has_obs = msg->has_observation;
  nc.timestamp = msg->header.stamp;
  nc.zeta_alpha = Eigen::Map<const Eigen::VectorXd>(msg->zeta_alpha.data(), msg->num_components);
  nc.zeta_a.resize(msg->num_components);
  nc.zeta_b.resize(msg->num_components);
  for (int c = 0; c < msg->num_components; ++c) {
    nc.zeta_a[c] = Eigen::Map<const Eigen::VectorXd>(msg->zeta_a.data() + c * msg->state_dim, msg->state_dim);
    nc.zeta_b[c] = Eigen::Map<const Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>(
        msg->zeta_b.data() + c * msg->state_dim * msg->state_dim, msg->state_dim, msg->state_dim);
  }
  received_consensus_[msg->drone_id] = nc;
}

// === 收到其他无人机标签化共识状态的回调 ===
void labeled_consensus_callback(const target_ekf::LabeledConsensusState::ConstPtr& msg) {
  std::lock_guard<std::mutex> lock(labeled_stats_mutex_);
  
  LabeledNeighborConsensus nc; 
  nc.label = static_cast<SearchIntent>(msg->label);
  nc.zeta_alpha.resize(msg->alpha.size());
  for (size_t i = 0; i < msg->alpha.size(); ++i) {
    nc.zeta_alpha(i) = msg->alpha[i];
  }
  
  nc.zeta_a.resize(msg->num_components);
  nc.zeta_b.resize(msg->num_components);
  
  for (int c = 0; c < msg->num_components; ++c) {
    nc.zeta_a[c].resize(msg->state_dim);
    nc.zeta_b[c].resize(msg->state_dim, msg->state_dim);
    
    // 从数组填充a向量
    int a_start = c * msg->state_dim;
    for (int i = 0; i < msg->state_dim; ++i) {
      nc.zeta_a[c](i) = msg->a[a_start + i];
    }
    
    // 从数组填充b矩阵
    int b_start = c * msg->state_dim * msg->state_dim;
    for (int i = 0; i < msg->state_dim; ++i) {
      for (int j = 0; j < msg->state_dim; ++j) {
        nc.zeta_b[c](i, j) = msg->b[b_start + i * msg->state_dim + j];
      }
    }
  }
  
  nc.has_obs = msg->has_obs;
  nc.timestamp = msg->header.stamp;
  
  // 存储该标签的共识状态（追加而非覆盖）
  received_labeled_consensus_[msg->drone_id].push_back(nc);
}

// === 【关键修复】无人机odom回调：仅保存最新位姿，不执行核心逻辑 ===
void odom_callback(const nav_msgs::OdometryConstPtr& odom_msg) {
  std::lock_guard<std::mutex> lock(odom_mutex_);
  latest_odom_pos_.x() = odom_msg->pose.pose.position.x;
  latest_odom_pos_.y() = odom_msg->pose.pose.position.y;
  latest_odom_pos_.z() = odom_msg->pose.pose.position.z;
  latest_odom_q_.w() = odom_msg->pose.pose.orientation.w;
  latest_odom_q_.x() = odom_msg->pose.pose.orientation.x;
  latest_odom_q_.y() = odom_msg->pose.pose.orientation.y;
  latest_odom_q_.z() = odom_msg->pose.pose.orientation.z;
  has_latest_odom_ = true;
}

// === 【关键修复】YOLO目标回调：仅保存最新观测，不执行核心逻辑 ===
void yolo_callback(const nav_msgs::OdometryConstPtr& target_msg) {
  std::lock_guard<std::mutex> lock(obs_mutex_);
  latest_obs_pos_.x() = target_msg->pose.pose.position.x;
  latest_obs_pos_.y() = target_msg->pose.pose.position.y;
  latest_obs_pos_.z() = target_msg->pose.pose.position.z;
  Eigen::Quaterniond q;
  q.w() = target_msg->pose.pose.orientation.w;
  q.x() = target_msg->pose.pose.orientation.x;
  q.y() = target_msg->pose.pose.orientation.y;
  q.z() = target_msg->pose.pose.orientation.z;
  latest_obs_rpy_ = quaternion2euler(q);
  latest_obs_stamp_ = target_msg->header.stamp;
  has_latest_obs_ = true;
}

// === 【核心】固定频率Timer回调：执行DPF完整流程，解决回调饥饿 ===
// 无论有无观测，都会触发，保证无观测节点也能参与共识、发布数据
void dpf_core_timer_callback(const ros::TimerEvent& event) {
  // 检查是否有无人机位姿
  if (!has_latest_odom_ || !occMap_.received) {
    ROS_DEBUG_THROTTLE(1.0, "[dpf%d] Waiting for odom and map...", drone_id_);
    return;
  }
  // 读取无人机最新位姿
  Eigen::Vector3d odom_p;
  Eigen::Quaterniond odom_q;
  {
    std::lock_guard<std::mutex> lock(odom_mutex_);
    odom_p = latest_odom_pos_;
    odom_q = latest_odom_q_;
  }
  Eigen::Vector3d cam_p = odom_q.toRotationMatrix() * cam2body_p_ + odom_p;
  Eigen::Quaterniond cam_q = odom_q * Eigen::Quaterniond(cam2body_R_);
  // 检查是否有有效观测
  bool has_obs = false;
  Eigen::Vector3d obs_pos, obs_rpy;
  {
    std::lock_guard<std::mutex> lock(obs_mutex_);
    if (has_latest_obs_) {
      double obs_age = (ros::Time::now() - latest_obs_stamp_).toSec();
      if (obs_age < obs_timeout_) {
        // FOV检查
        if (check_fov_) {
          Eigen::Vector3d p_in_body = cam_q.inverse() * (latest_obs_pos_ - cam_p);
          if (p_in_body.z() > 0.1 && p_in_body.z() < 5.0) {
            double x = p_in_body.x() * fx_ / p_in_body.z() + cx_;
            double y = p_in_body.y() * fy_ / p_in_body.z() + cy_;
            if (x >= 0 && x <= height_ && y >=0 && y <= width_) {
              // 视距检查
              if (isLineOfSightClear(cam_p, latest_obs_pos_)) {
                has_obs = true;
                obs_pos = latest_obs_pos_;
                obs_rpy = latest_obs_rpy_;
              }
            }
          }
        } else {
          // 不检查FOV，仅检查视距
          if (isLineOfSightClear(cam_p, latest_obs_pos_)) {
            has_obs = true;
            obs_pos = latest_obs_pos_;
            obs_rpy = latest_obs_rpy_;
          }
        }
      }
    }
    // 用完重置观测标志，避免重复使用
    has_latest_obs_ = false;
  }
  // 初始化/重置逻辑
  double update_dt = (ros::Time::now() - last_update_stamp_).toSec();
  bool need_reset = (!dpfPtr_->initialized_ || update_dt > 1.0);
  if (need_reset) {
    if (has_obs) {
      dpfPtr_->reset(obs_pos, obs_rpy);
      dpf_reset_suppress_count_ = 3;
      ROS_WARN("[dpf%d] reset at obs=(%.2f,%.2f,%.2f) dt=%.2fs",
        drone_id_, obs_pos.x(), obs_pos.y(), obs_pos.z(), update_dt);
      last_update_stamp_ = ros::Time::now();
      return;
    } else {
      ROS_DEBUG_THROTTLE(1.0, "[dpf%d] no obs, skip reset (initialized=%d)", 
        drone_id_, dpfPtr_->initialized_);
      if (!dpfPtr_->initialized_) return;
    }
  }
  // ==============================================
  // 【模式判断 - 在回调开始时判断当前应执行的模式】
  // ==============================================
  ROS_DEBUG("[dpf%d] has_obs=%d, initialized=%d", drone_id_, has_obs, dpfPtr_->initialized_);
  if (search_mode_active_) {
    // =========================
    // 【搜索模式处理流程】
    // =========================

    // --- 优先退出检查1：本无人机发现目标 → 直接从观测重置，不走共识 ---
    if (has_obs) {
      dpfPtr_->reset(obs_pos, obs_rpy);
      dpf_reset_suppress_count_ = 3;
      search_mode_active_ = false;
      consecutive_no_obs_count_ = 0;
      ROS_WARN("[dpf%d] EXITING SEARCH MODE: Target reacquired, resetting from direct observation!", drone_id_);

      // 发布 local_stats（has_obs=true），通知邻居此机已重新观测到目标
      LocalStat local_stat = dpfPtr_->computeLocalStatsOnly(drone_id_, true);
      local_stats_pub_.publish(toMsg(local_stat, *dpfPtr_));

      {
        std::lock_guard<std::mutex> lock(labeled_stats_mutex_);
        received_labeled_consensus_.clear();
      }
      std_msgs::Bool search_state_msg;
      search_state_msg.data = false;
      search_state_pub_.publish(search_state_msg);
      last_update_stamp_ = ros::Time::now();
      return;
    }

    // --- 优先退出检查2：邻居无人机已重新观测到目标 → 退出搜索模式，由普通DPF共识同步状态 ---
    {
      std::lock_guard<std::mutex> lock(stats_mutex_);
      ros::Time current_time = ros::Time::now();
      for (auto& kv : received_consensus_) {
        double time_diff = (current_time - kv.second.timestamp).toSec();
        if (time_diff < 0.5 && kv.second.has_obs) {
          search_mode_active_ = false;
          consecutive_no_obs_count_ = 0;
          ROS_WARN("[dpf%d] EXITING SEARCH MODE: Neighbor drone %d reacquired target!", drone_id_, kv.first);
          break;
        }
      }
    }
    if (!search_mode_active_) {
      {
        std::lock_guard<std::mutex> lock(labeled_stats_mutex_);
        received_labeled_consensus_.clear();
      }
      std_msgs::Bool search_state_msg;
      search_state_msg.data = false;
      search_state_pub_.publish(search_state_msg);
      return;
    }

    // --- 步骤1：状态预测（搜索粒子状态更新）---
    search_particles_manager_->updateAllSearchParticles();

    // --- 步骤2：负观测权重更新（此处 has_obs 已确认为 false）---
    search_particles_manager_->updateSearchParticlesWithNegativeObservation(cam_p, cam_q, 0.05);

    // --- 步骤3：标签化的E步，计算每个标签的本地统计量（所有节点执行）---
    std::vector<LabeledLocalStat> local_labeled_stats = search_particles_manager_->computeLabeledLocalStats();

    // --- 步骤4：收集邻居标签化共识状态---
    std::vector<LabeledNeighborConsensus> neighbor_labeled_consensus;
    {
      std::lock_guard<std::mutex> lock(labeled_stats_mutex_);
      ros::Time current_time = ros::Time::now();
      double time_threshold = 0.3;
      for (auto& kv : received_labeled_consensus_) {
        if (kv.first != drone_id_) { // 不包括自己
          for (const auto& labeled_nc : kv.second) {
            double time_diff = (current_time - labeled_nc.timestamp).toSec();
            if (time_diff < time_threshold) {
              neighbor_labeled_consensus.push_back(labeled_nc);
            }
          }
        }
      }
      // 清理过期数据
      received_labeled_consensus_.clear();
    }

    // --- 步骤5：标签化的共识滤波（每个标签独立运行共识）---
    search_particles_manager_->labeledConsensusFilter(neighbor_labeled_consensus, local_labeled_stats);

    // --- 步骤5.1：发布搜索粒子可视化---
    publishSearchParticlesVisualization();

    // --- 步骤5.2：发布GMM分布可视化---
    publishGMMVisualization();

    // --- 步骤6：发布标签化共识状态（用于邻居间通信）---
  auto zeta_alphas = search_particles_manager_->getLabeledZetaAlpha();
  auto zeta_as = search_particles_manager_->getLabeledZetaA();
  auto zeta_bs = search_particles_manager_->getLabeledZetaB();

  for (int label_int = STRAIGHT; label_int <= RIGHT_TURN; ++label_int) {
    SearchIntent label = static_cast<SearchIntent>(label_int);
    
    target_ekf::LabeledConsensusState labeled_consensus_msg;
    labeled_consensus_msg.header.stamp = ros::Time::now();
    labeled_consensus_msg.header.frame_id = "world";
    labeled_consensus_msg.drone_id = drone_id_;
    labeled_consensus_msg.label = static_cast<int>(label);
    labeled_consensus_msg.num_components = num_components;
    labeled_consensus_msg.state_dim = 9;
    labeled_consensus_msg.has_obs = false;  // 搜索模式中总是false

    // 填充ζ而不是u
    auto& zeta_alpha = zeta_alphas[label];
    auto& zeta_a = zeta_as[label];
    auto& zeta_b = zeta_bs[label];
    
    labeled_consensus_msg.alpha.resize(zeta_alpha.size());
    for (int i = 0; i < zeta_alpha.size(); ++i) {
      labeled_consensus_msg.alpha[i] = zeta_alpha(i);  // ✅ 发布ζ
    }
    
    labeled_consensus_msg.a.resize(zeta_a.size() * 9);
    for (int c = 0; c < zeta_a.size(); ++c) {
      for (int i = 0; i < 9; ++i) {
        labeled_consensus_msg.a[c * 9 + i] = zeta_a[c](i);
      }
    }
    
    labeled_consensus_msg.b.resize(zeta_b.size() * 9 * 9);
    for (int c = 0; c < zeta_b.size(); ++c) {
      for (int i = 0; i < 9; ++i) {
        for (int j = 0; j < 9; ++j) {
          labeled_consensus_msg.b[c * 9 * 9 + i * 9 + j] = zeta_b[c](i, j);
        }
      }
    }
    
    labeled_consensus_pub_.publish(labeled_consensus_msg);
  }

    // --- 步骤7：发布搜索状态---
    std_msgs::Bool search_state_msg;
    search_state_msg.data = search_mode_active_;
    search_state_pub_.publish(search_state_msg);

    return;  // 提前结束，不执行普通DPF流程
  }
  else {
    // =========================
    // 【普通DPF模式处理流程 - 对齐论文Algorithm 1】
    // =========================
    // --- 论文步骤1：从GMM采样新粒子（Importance sampling step）---
    dpfPtr_->sampleParticlesFromGMM();
    
    // --- 论文步骤2：状态预测 ---
    dpfPtr_->predict();
    
    // --- 论文步骤3：权重更新（仅有观测节点执行）---
    if (has_obs) {
      dpfPtr_->updateWeights(obs_pos, obs_rpy);
    }
    
    // --- 论文步骤4：E步，计算本地统计量（所有节点执行，无观测节点也计算真实统计量）---
    LocalStat local_stat = dpfPtr_->computeLocalStatsOnly(drone_id_, has_obs);
    
    // --- 论文步骤5：收集邻居共识状态并检测是否需要切换到搜索模式 ---
    std::vector<NeighborConsensus> neighbor_consensus;
    std::vector<int> neighbor_ids;
    {
      std::lock_guard<std::mutex> lock(stats_mutex_);
      ros::Time current_time = ros::Time::now();
      double time_threshold = 0.3;
      for (auto& kv : received_consensus_) {
        double time_diff = (current_time - kv.second.timestamp).toSec();
        if (time_diff < time_threshold) {
          neighbor_consensus.push_back(kv.second);
          neighbor_ids.push_back(kv.first); // 存储邻居ID
        }
      }
      // 兜底：无有效邻居时保留最近1个
      if (neighbor_consensus.empty() && !received_consensus_.empty()) {
        neighbor_consensus.push_back(received_consensus_.begin()->second);
        neighbor_ids.push_back(received_consensus_.begin()->first);
      }
      // 清理过期数据
      received_consensus_.clear();
    }
    
    // 检查是否所有无人机都失去了观测（包括当前无人机和邻居）
    bool all_drones_no_obs = true;
    
    // 检查当前无人机是否有观测
    if (has_obs) {
      all_drones_no_obs = false;
    }
    
    // 检查邻居无人机是否有观测
    for (size_t i = 0; i < neighbor_consensus.size(); ++i) {
      if (neighbor_consensus[i].has_obs) {
        all_drones_no_obs = false;
        break;
      }
    }
    
    // 更新连续无观测计数
    if (all_drones_no_obs) {
      consecutive_no_obs_count_++;
      if (consecutive_no_obs_count_ >= miss_detection_num_ && !search_mode_active_) {
        // 立即进入搜索模式并进行初始化
        search_mode_active_ = true;
        last_global_obs_time_ = ros::Time::now();
        ROS_WARN("[dpf%d] ENTERING SEARCH MODE: All drones lost target for %d consecutive frames!", 
                 drone_id_, consecutive_no_obs_count_);
        // 继承当前DPF粒子并分配意图标签
        search_particles_manager_->inheritAndLabelParticles(dpfPtr_.get());
        
        // 发布搜索状态
        std_msgs::Bool search_state_msg;
        search_state_msg.data = search_mode_active_;
        search_state_pub_.publish(search_state_msg);
        
        // 跳过后续普通DPF流程，直接返回
        return;
      }
    } else {
      // 有任意无人机重新观测到目标
      consecutive_no_obs_count_ = 0;
    }
    
    // --- 论文步骤6：单步EM迭代（共识滤波+M步）---
    dpfPtr_->emStep(drone_id_, neighbor_consensus, local_stat);
    ROS_DEBUG("[dpf%d][EM] completed 1 step, fused %zu neighbors", drone_id_, neighbor_consensus.size());
    
    // --- 论文步骤7：系统重采样（仅有观测节点执行）---
    //if (has_obs) {
      //dpfPtr_->systematicResample();
    //}
    
    // --- 发布最新的共识状态ζ（给邻居下一帧使用）---
    local_stats_pub_.publish(toMsg(local_stat, *dpfPtr_));
    
    // --- 数值有效性检查 ---
    if (!dpfPtr_->isValid()) {
      ROS_ERROR("[dpf%d] update invalid! NaN/Inf detected, resetting.", drone_id_);
      if (has_obs) dpfPtr_->reset(obs_pos, obs_rpy);
      dpf_reset_suppress_count_ = 3;
      return;
    }
    
    // 发布搜索状态（在这种情况下为false）
    std_msgs::Bool search_state_msg;
    search_state_msg.data = search_mode_active_;
    search_state_pub_.publish(search_state_msg);
  }
  // --- 发布目标odom ---
  Eigen::Vector3d est_pos = dpfPtr_->pos();
  Eigen::Vector3d est_vel = dpfPtr_->vel();
  Eigen::Vector3d est_rpy = dpfPtr_->rpy();
  double pos_err = has_obs ? (est_pos - obs_pos).norm() : -1.0;
  
  // 打印GMM第一个分量的位置，确认多机同步
  Eigen::VectorXd gmm_mu0 = dpfPtr_->gmm_mu_[0];
  ROS_INFO_THROTTLE(0.5, "[dpf%d][est] pos=(%.2f,%.2f,%.2f) vel=(%.2f,%.2f,%.2f) | GMM0=(%.2f,%.2f,%.2f) | obs_err=%.3fm | has_obs=%d",
    drone_id_,
    est_pos.x(), est_pos.y(), est_pos.z(),
    est_vel.x(), est_vel.y(), est_vel.z(),
    gmm_mu0(0), gmm_mu0(1), gmm_mu0(2),
    pos_err, has_obs);
  nav_msgs::Odometry target_odom;
  target_odom.header.stamp = ros::Time::now();
  target_odom.header.frame_id = "world";
  target_odom.pose.pose.position.x = est_pos.x();
  target_odom.pose.pose.position.y = est_pos.y();
  target_odom.pose.pose.position.z = est_pos.z();
  target_odom.twist.twist.linear.x = est_vel.x();
  target_odom.twist.twist.linear.y = est_vel.y();
  target_odom.twist.twist.linear.z = est_vel.z();
  Eigen::Quaterniond q_out = euler2quaternion(est_rpy);
  target_odom.pose.pose.orientation.w = q_out.w();
  target_odom.pose.pose.orientation.x = q_out.x();
  target_odom.pose.pose.orientation.y = q_out.y();
  target_odom.pose.pose.orientation.z = q_out.z();
  target_odom_pub_.publish(target_odom);
  dpf_reset_suppress_count_ = 0;
  last_update_stamp_ = ros::Time::now();
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "target_dpf");
  ros::NodeHandle nh("~");
  last_update_stamp_ = ros::Time::now() - ros::Duration(10.0);
  // 读取参数
  nh.param<int>("drone_id", drone_id_, 0);
  nh.param<int>("num_drones", num_drones_, 3);
  std::vector<double> tmp;
  if (nh.param<std::vector<double>>("cam2body_R", tmp, std::vector<double>())) {
    cam2body_R_ = Eigen::Map<const Eigen::Matrix<double, -1, -1, Eigen::RowMajor>>(tmp.data(), 3, 3);
  }
  if (nh.param<std::vector<double>>("cam2body_p", tmp, std::vector<double>())) {
    cam2body_p_ = Eigen::Map<const Eigen::Matrix<double, -1, -1, Eigen::RowMajor>>(tmp.data(), 3, 1);
  }
  nh.getParam("cam_fx", fx_);
  nh.getParam("cam_fy", fy_);
  nh.getParam("cam_cx", cx_);
  nh.getParam("cam_cy", cy_);
  nh.getParam("cam_width", width_);
  nh.getParam("cam_height", height_);
  nh.getParam("pitch_thr", pitch_thr_);
  nh.getParam("check_fov", check_fov_);
  // 搜索模式参数
  nh.getParam("miss_detection_num", miss_detection_num_);
  // DPF参数
  int num_particles = 300;
  int num_components = 4;
  int num_em_iters = 10;
  nh.getParam("dpf_rate", dpf_rate_);
  nh.getParam("num_particles", num_particles);
  nh.getParam("num_components", num_components);
  nh.getParam("num_em_iters", num_em_iters);
  dpfPtr_ = std::make_shared<DistributedPF>(1.0 / dpf_rate_, num_particles, num_components, num_em_iters);
  
  // 创建搜索粒子管理器
  search_particles_manager_ = std::make_unique<SearchParticlesManager>(drone_id_, num_components, &nh);
  // ✅ 设置视线检查函数
  search_particles_manager_->setLineOfSightCheckFn(
  [](const Eigen::Vector3d& start, const Eigen::Vector3d& end) {
    return isLineOfSightClear(start, end);
  }
);
  
  // 订阅/发布
  target_odom_pub_ = nh.advertise<nav_msgs::Odometry>("target_odom", 1);
  local_stats_pub_ = nh.advertise<target_ekf::LocalStats>("local_stats", 1);
  labeled_consensus_pub_ = nh.advertise<target_ekf::LabeledConsensusState>("labeled_consensus", 1);
  search_state_pub_ = nh.advertise<std_msgs::Bool>("search_state", 1);
  search_gmm_vis_pub_ = nh.advertise<visualization_msgs::MarkerArray>("search_gmm_vis", 1);
  search_gmm_vis_pub_ = nh.advertise<visualization_msgs::MarkerArray>("search_gmm_vis", 1);
  search_particles_vis_pub_ = nh.advertise<sensor_msgs::PointCloud2>("search_particles_vis", 1);
  
  // 地图订阅
  ros::Subscriber global_map_sub = nh.subscribe("global_map", 10, &global_map_callback);
  // 无人机位姿订阅（单独订阅，不再同步）
  ros::Subscriber odom_sub = nh.subscribe("odom", 100, &odom_callback, ros::TransportHints().tcpNoDelay());
  // YOLO目标订阅（单独订阅，不再同步）
  ros::Subscriber yolo_sub = nh.subscribe("yolo", 1, &yolo_callback, ros::TransportHints().tcpNoDelay());
  // 订阅其他无人机的局部统计量
  for (int i = 0; i < num_drones_; ++i) {
    if (i == drone_id_) continue;
    std::string topic = "/drone" + std::to_string(i) + "/drone" + std::to_string(i) + "_target_dpf/local_stats";
    stats_subs_.push_back(nh.subscribe(topic, 10, &stats_callback));
    std::string labeled_topic = "/drone" + std::to_string(i) + "/drone" + std::to_string(i) + "_target_dpf/labeled_consensus";
    stats_subs_.push_back(nh.subscribe(labeled_topic, 10, &labeled_consensus_callback));
    ROS_INFO("[dpf%d] Subscribing to %s and %s", drone_id_, topic.c_str(), labeled_topic.c_str());
  }
  // 【核心】固定频率Timer执行DPF核心逻辑
  ros::Timer dpf_core_timer = nh.createTimer(ros::Duration(1.0 / dpf_rate_), &dpf_core_timer_callback);
  ros::MultiThreadedSpinner spinner(4); // 多线程Spinner，避免回调阻塞
  spinner.spin();
  return 0;
}