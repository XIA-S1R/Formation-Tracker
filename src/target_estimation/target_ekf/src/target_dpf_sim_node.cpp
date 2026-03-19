// 分布式粒子滤波 (DPF) 仿真节点
// 100%对齐 Gu 2007 "Distributed Particle Filter for Target Tracking" Algorithm 1
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/time_synchronizer.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseArray.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/MarkerArray.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <unordered_set>
#include <cmath>
#include <mutex>
#include <atomic>
#include <target_ekf/target_ekf.hpp>
#include <target_ekf/target_dpf.hpp>
#include <target_ekf/LocalStats.h>
#include <target_ekf/LabeledConsensusState.h>
#include <target_ekf/SearchLabelInfo.h>
#include <std_msgs/Bool.h>
#include <unordered_set>
#include <algorithm>
#include <numeric>
#include "target_ekf/search_particles_manager.hpp"

// === 前向声明 ===
struct GMMAssignment {
  int drone_id;
  int gmm_id;
  double distance;
};

void matchGMMCenters(const std::vector<Eigen::Vector3d>& current_mu,
                     const Eigen::VectorXd& current_pi,
                     double distance_threshold = 2.0);
std::vector<GMMAssignment> assignGMMTasks(const std::vector<Eigen::Vector3d>& drone_positions,
                                          const std::vector<Eigen::Vector3d>& gmm_centers,
                                          const std::vector<int>& gmm_ids,
                                          const Eigen::VectorXd& gmm_weights);

// === 全局变量 ===
// 发布器
ros::Publisher target_odom_pub_;
ros::Publisher local_stats_pub_;
ros::Publisher labeled_consensus_pub_;
ros::Publisher search_state_pub_;
ros::Publisher search_particles_vis_pub_;
ros::Publisher search_gmm_vis_pub_; // 搜索GMM分布可视化发布器
ros::Publisher search_pos_gmm_pub_;  // 位置GMM发布器
ros::Publisher search_targets_pub_;  // 搜索目标点发布器
ros::Publisher search_label_info_pub_;  // 搜索标签信息发布器
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
int consensus_rate_ = 100;  // 共识迭代频率，两次观测间可进行多轮真实通信
std::shared_ptr<DistributedPF> dpfPtr_;
std::mutex dpf_mutex_;  // 保护 dpfPtr_ 的并发访问
std::atomic<bool> has_recent_obs_{false};  // 观测timer通知共识timer有新观测
ros::Time last_update_stamp_;
int dpf_reset_suppress_count_ = 0;

// 搜索模式相关
int miss_detection_num_ = 10; // 连续多少帧都没有观测后进入搜索模式
int consecutive_no_obs_count_ = 0; // 连续无观测计数
bool search_mode_active_ = false; // 是否处于搜索模式
ros::Time last_global_obs_time_;

// 邻居无人机位置（用于匈牙利分配）
std::map<int, Eigen::Vector3d> neighbor_positions_;
std::mutex neighbor_odom_mutex_;
std::vector<ros::Subscriber> neighbor_odom_subs_;

// 标签分配结果
SearchIntent my_assigned_label_ = STRAIGHT;  // 本机分配到的标签
bool is_label_master_ = false;               // 是否是该标签的掌管者
std::map<SearchIntent, std::vector<int>> label_to_drones_;  // 每个标签分配到的无人机ID列表

// 搜索粒子管理器
std::unique_ptr<SearchParticlesManager> search_particles_manager_;

// 邻居数据存储
std::map<int, LocalStat> received_stats_;
std::map<int, NeighborConsensus> received_consensus_;
std::map<int, std::vector<LabeledNeighborConsensus>> received_labeled_consensus_;
std::mutex stats_mutex_;
std::mutex labeled_stats_mutex_;

// === GMM中心帧间匹配相关 ===
std::vector<Eigen::Vector3d> prev_pos_gmm_mu_;  // 上一帧的GMM中心
std::vector<int> gmm_center_ids_;               // 每个中心的持久ID
int next_gmm_id_ = 0;                           // 下一个可用的ID
std::mt19937 rng_;                              // 随机数生成器

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
/*void publishSearchParticlesVisualization() {
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
}*/

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

// === 邻居无人机odom回调：存储邻居位置用于匈牙利分配 ===
void neighbor_odom_callback(const nav_msgs::OdometryConstPtr& msg, int neighbor_id) {
  std::lock_guard<std::mutex> lock(neighbor_odom_mutex_);
  neighbor_positions_[neighbor_id] = Eigen::Vector3d(
      msg->pose.pose.position.x,
      msg->pose.pose.position.y,
      msg->pose.pose.position.z);
}

// === 匈牙利算法分配标签 ===
// 输入：各无人机位置，三个搜索目标点
// 输出：每个无人机分配到的标签，以及谁是掌管者
void hungarianAssignLabels(const std::vector<Eigen::Vector3d>& search_targets) {
  // 收集所有无人机位置（包括自己）
  std::vector<std::pair<int, Eigen::Vector3d>> drone_positions;

  // 添加自己的位置
  {
    std::lock_guard<std::mutex> lock(odom_mutex_);
    drone_positions.push_back({drone_id_, latest_odom_pos_});
  }

  // 添加邻居位置
  {
    std::lock_guard<std::mutex> lock(neighbor_odom_mutex_);
    for (const auto& kv : neighbor_positions_) {
      drone_positions.push_back({kv.first, kv.second});
    }
  }

  // 按drone_id排序
  std::sort(drone_positions.begin(), drone_positions.end(),
            [](const auto& a, const auto& b) { return a.first < b.first; });

  int n_drones = drone_positions.size();
  int n_labels = 3;  // STRAIGHT, LEFT_TURN, RIGHT_TURN

  ROS_INFO("[dpf%d] Hungarian assignment: %d drones, 3 labels", drone_id_, n_drones);

  // 打印所有无人机位置
  for (int d = 0; d < n_drones; ++d) {
    ROS_INFO("[dpf%d]   drone%d pos: (%.2f, %.2f, %.2f)",
             drone_id_, drone_positions[d].first,
             drone_positions[d].second.x(), drone_positions[d].second.y(), drone_positions[d].second.z());
  }

  // 打印搜索目标点
  const char* target_names[] = {"STRAIGHT", "LEFT_TURN", "RIGHT_TURN"};
  for (int l = 0; l < n_labels; ++l) {
    ROS_INFO("[dpf%d]   target %s: (%.2f, %.2f, %.2f)",
             drone_id_, target_names[l],
             search_targets[l].x(), search_targets[l].y(), search_targets[l].z());
  }

  // 构建代价矩阵：drone到各搜索目标点的距离
  Eigen::MatrixXd cost_matrix(n_drones, n_labels);
  for (int d = 0; d < n_drones; ++d) {
    for (int l = 0; l < n_labels; ++l) {
      cost_matrix(d, l) = (drone_positions[d].second - search_targets[l]).norm();
    }
  }

  // 贪心匈牙利分配（保证每个标签至少有一个无人机）
  std::vector<int> assignments(n_drones, -1);  // drone -> label
  std::vector<int> label_masters(n_labels, -1); // label -> master drone_id

  if (n_drones <= n_labels) {
    // 无人机数 <= 3：一对一分配
    std::vector<bool> label_assigned(n_labels, false);

    // 按距离排序所有(drone, label)对
    std::vector<std::tuple<double, int, int>> dist_pairs;
    for (int d = 0; d < n_drones; ++d) {
      for (int l = 0; l < n_labels; ++l) {
        dist_pairs.push_back({cost_matrix(d, l), d, l});
      }
    }
    std::sort(dist_pairs.begin(), dist_pairs.end());

    std::vector<bool> drone_assigned(n_drones, false);
    for (const auto& [dist, d, l] : dist_pairs) {
      if (!drone_assigned[d] && !label_assigned[l]) {
        assignments[d] = l;
        label_masters[l] = drone_positions[d].first;
        drone_assigned[d] = true;
        label_assigned[l] = true;
      }
    }
  } else {
    // 无人机数 > 3：先分配前3个掌管者，再分配剩余
    std::vector<bool> label_assigned(n_labels, false);
    std::vector<bool> drone_assigned(n_drones, false);

    // 第一轮：为每个标签找最近的无人机作为掌管者
    for (int l = 0; l < n_labels; ++l) {
      int best_d = -1;
      double best_dist = 1e9;
      for (int d = 0; d < n_drones; ++d) {
        if (!drone_assigned[d] && cost_matrix(d, l) < best_dist) {
          best_d = d;
          best_dist = cost_matrix(d, l);
        }
      }
      if (best_d >= 0) {
        assignments[best_d] = l;
        label_masters[l] = drone_positions[best_d].first;
        drone_assigned[best_d] = true;
        label_assigned[l] = true;
      }
    }

    // 第二轮：剩余无人机分配到最近的标签
    for (int d = 0; d < n_drones; ++d) {
      if (!drone_assigned[d]) {
        int best_l = 0;
        double best_dist = cost_matrix(d, 0);
        for (int l = 1; l < n_labels; ++l) {
          if (cost_matrix(d, l) < best_dist) {
            best_l = l;
            best_dist = cost_matrix(d, l);
          }
        }
        assignments[d] = best_l;
      }
    }
  }

  // 找到本机的分配结果
  for (int d = 0; d < n_drones; ++d) {
    if (drone_positions[d].first == drone_id_) {
      my_assigned_label_ = static_cast<SearchIntent>(assignments[d]);
      is_label_master_ = (label_masters[assignments[d]] == drone_id_);
      break;
    }
  }

  const char* label_names[] = {"STRAIGHT", "LEFT_TURN", "RIGHT_TURN"};
  ROS_WARN("[dpf%d] Assigned to label %s, is_master=%d",
           drone_id_, label_names[my_assigned_label_], is_label_master_);

  // 构建每个标签分配到的无人机列表
  label_to_drones_.clear();
  for (int l = 0; l < n_labels; ++l) {
    label_to_drones_[static_cast<SearchIntent>(l)] = std::vector<int>();
  }
  for (int d = 0; d < n_drones; ++d) {
    SearchIntent label = static_cast<SearchIntent>(assignments[d]);
    label_to_drones_[label].push_back(drone_positions[d].first);
  }
  // 对每个标签内的无人机ID排序
  for (auto& kv : label_to_drones_) {
    std::sort(kv.second.begin(), kv.second.end());
  }

  // 打印完整分配结果
  for (int d = 0; d < n_drones; ++d) {
    ROS_INFO("  drone%d -> %s %s",
             drone_positions[d].first,
             label_names[assignments[d]],
             (label_masters[assignments[d]] == drone_positions[d].first) ? "(master)" : "");
  }
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

// === 【共识Timer回调】高频共识迭代（100Hz），两次观测间多轮真实通信 ===
void consensus_timer_callback(const ros::TimerEvent& event) {
  if (!dpfPtr_->initialized_ || search_mode_active_) return;

  std::lock_guard<std::mutex> dpf_lock(dpf_mutex_);

  // E步：用当前GMM参数计算本地统计量
  bool obs_flag = has_recent_obs_.load();
  LocalStat local_stat = dpfPtr_->computeLocalStatsOnly(drone_id_, obs_flag);

  // 收集邻居最新的共识状态ζ
  std::vector<NeighborConsensus> neighbor_consensus;
  {
    std::lock_guard<std::mutex> lock(stats_mutex_);
    ros::Time current_time = ros::Time::now();
    double time_threshold = 0.3;
    for (auto& kv : received_consensus_) {
      double time_diff = (current_time - kv.second.timestamp).toSec();
      if (time_diff < time_threshold) {
        neighbor_consensus.push_back(kv.second);
      }
    }
    if (neighbor_consensus.empty() && !received_consensus_.empty()) {
      neighbor_consensus.push_back(received_consensus_.begin()->second);
    }
  }

  // 共识更新 + M步
  dpfPtr_->emStep(drone_id_, neighbor_consensus, local_stat);

  // 发布最新的ζ给邻居
  local_stats_pub_.publish(toMsg(local_stat, *dpfPtr_));
}

// === 【核心】固定频率Timer回调：执行DPF观测相关流程（20Hz）===
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
  // 加锁保护 dpfPtr_ 的并发访问（与 consensus_timer_callback 互斥）
  std::lock_guard<std::mutex> dpf_lock(dpf_mutex_);
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

    // --- 步骤1：状态预测（单标签粒子群更新，带意图切换衰减）---
    search_particles_manager_->updateSingleLabelParticles();

    // --- 步骤2：负观测更新（删除在FOV内但未观测到目标的粒子，复制其他粒子补充）---
    search_particles_manager_->applyNegativeObservationDelete(cam_p, cam_q);

    // --- 步骤3：发布搜索粒子可视化---
    //publishSearchParticlesVisualization();

    // --- 步骤4：发布搜索状态---
    std_msgs::Bool search_state_msg;
    search_state_msg.data = search_mode_active_;
    search_state_pub_.publish(search_state_msg);

    // --- 步骤5：发布目标odom和标签信息（使用单标签粒子群均值位置）---
    if (is_label_master_) {
      Eigen::Vector3d search_target_pos = search_particles_manager_->getSingleLabelMeanPosition();

      // 计算粒子群均值速度方向
      auto particles_weights = search_particles_manager_->getParticlesAndWeights();
      Eigen::Vector3d mean_vel = Eigen::Vector3d::Zero();
      double total_w = 0.0;
      for (int i = 0; i < particles_weights.first.cols(); ++i) {
        mean_vel += particles_weights.second(i) * particles_weights.first.col(i).segment(3, 3);
        total_w += particles_weights.second(i);
      }
      if (total_w > 1e-300) mean_vel /= total_w;

      // 发布 SearchLabelInfo 消息
      target_ekf::SearchLabelInfo label_info_msg;
      label_info_msg.header.stamp = ros::Time::now();
      label_info_msg.header.frame_id = "world";
      label_info_msg.source_drone_id = drone_id_;
      label_info_msg.label = static_cast<int>(my_assigned_label_);
      label_info_msg.mean_pos.x = search_target_pos.x();
      label_info_msg.mean_pos.y = search_target_pos.y();
      label_info_msg.mean_pos.z = search_target_pos.z();
      label_info_msg.mean_vel.x = mean_vel.x();
      label_info_msg.mean_vel.y = mean_vel.y();
      label_info_msg.mean_vel.z = mean_vel.z();
      label_info_msg.assigned_drone_ids = label_to_drones_[my_assigned_label_];
      label_info_msg.num_particles = search_particles_manager_->getSearchParticles().size();
      search_label_info_pub_.publish(label_info_msg);

      // 发布 target_odom（掌管者自己也用这个）
      nav_msgs::Odometry target_odom;
      target_odom.header.stamp = ros::Time::now();
      target_odom.header.frame_id = "world";
      target_odom.pose.pose.position.x = search_target_pos.x();
      target_odom.pose.pose.position.y = search_target_pos.y();
      target_odom.pose.pose.position.z = search_target_pos.z();
      target_odom.twist.twist.linear.x = mean_vel.x();
      target_odom.twist.twist.linear.y = mean_vel.y();
      target_odom.twist.twist.linear.z = mean_vel.z();
      target_odom_pub_.publish(target_odom);

      ROS_INFO_THROTTLE(1.0, "[dpf%d] Search label %s: pos=(%.2f,%.2f,%.2f), %zu drones assigned",
                        drone_id_, SearchParticlesManager::getLabelName(my_assigned_label_),
                        search_target_pos.x(), search_target_pos.y(), search_target_pos.z(),
                        label_to_drones_[my_assigned_label_].size());
    }

    last_update_stamp_ = ros::Time::now();
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
      has_recent_obs_.store(true);
    } else {
      has_recent_obs_.store(false);
    }

    // --- 论文步骤4-6 由 consensus_timer_callback 高频执行 ---

    // --- 收集邻居共识状态，检测是否需要切换到搜索模式 ---
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
      // 清理过期数据（只删超时的，不全清）
      for (auto it = received_consensus_.begin(); it != received_consensus_.end(); ) {
        double age = (current_time - it->second.timestamp).toSec();
        if (age > time_threshold) {
          it = received_consensus_.erase(it);
        } else {
          ++it;
        }
      }
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

        // 计算三个搜索目标点（反推位置）
        auto particles_and_weights = dpfPtr_->getParticlesAndWeights();
        auto search_targets = search_particles_manager_->computeSearchTargets(
            particles_and_weights.first,
            particles_and_weights.second,
            consecutive_no_obs_count_);

        // 构建搜索目标点向量用于匈牙利分配
        std::vector<Eigen::Vector3d> target_points = {
            search_targets.forward,
            search_targets.left_45,
            search_targets.right_45
        };

        // 匈牙利算法分配标签
        hungarianAssignLabels(target_points);

        // 更新搜索粒子管理器的标签信息
        search_particles_manager_->setLabelAssignment(my_assigned_label_, is_label_master_);

        // 只有掌管者才初始化粒子群
        if (is_label_master_) {
          search_particles_manager_->initializeSingleLabelParticles(
              search_targets.mean_pos,
              search_targets.mean_vel,
              consecutive_no_obs_count_);
        }

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

    // --- EM迭代由 consensus_timer_callback 高频执行 ---

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

// === GMM中心帧间匹配（匈牙利贪心算法）===
void matchGMMCenters(const std::vector<Eigen::Vector3d>& current_mu,
                     const Eigen::VectorXd& current_pi,
                     double distance_threshold) {
  int num_components = current_mu.size();
  std::vector<int> matched_ids(num_components, -1);

  if (!prev_pos_gmm_mu_.empty() && prev_pos_gmm_mu_.size() == gmm_center_ids_.size()) {
    // 构建代价矩阵：距离
    Eigen::MatrixXd cost_matrix(num_components, num_components);
    for (int i = 0; i < num_components; ++i) {
      for (int j = 0; j < (int)prev_pos_gmm_mu_.size(); ++j) {
        cost_matrix(i, j) = (current_mu[i] - prev_pos_gmm_mu_[j]).norm();
      }
    }

    // 贪心匹配：按当前权重从高到低，匹配距离最近的上一帧簇
    std::vector<int> sorted_indices(num_components);
    std::iota(sorted_indices.begin(), sorted_indices.end(), 0);
    std::sort(sorted_indices.begin(), sorted_indices.end(),
              [&](int a, int b) { return current_pi(a) > current_pi(b); });

    std::vector<bool> prev_matched(prev_pos_gmm_mu_.size(), false);

    for (int i : sorted_indices) {
      int best_j = -1;
      double best_dist = distance_threshold;
      for (int j = 0; j < (int)prev_pos_gmm_mu_.size(); ++j) {
        if (!prev_matched[j] && cost_matrix(i, j) < best_dist) {
          best_j = j;
          best_dist = cost_matrix(i, j);
        }
      }
      if (best_j >= 0) {
        matched_ids[i] = gmm_center_ids_[best_j];
        prev_matched[best_j] = true;
      } else {
        matched_ids[i] = next_gmm_id_++;  // 新簇
      }
    }
  } else {
    // 第一帧或簇数变化，分配新ID
    for (int i = 0; i < num_components; ++i) {
      matched_ids[i] = next_gmm_id_++;
    }
  }

  gmm_center_ids_ = matched_ids;
  prev_pos_gmm_mu_ = current_mu;

  ROS_DEBUG_THROTTLE(1.0, "[dpf%d] GMM center IDs: ", drone_id_);
  for (int id : gmm_center_ids_) {
    ROS_DEBUG_THROTTLE(1.0, "%d ", id);
  }
}

// === 任务分配：根据簇数和无人机数量关系分配 ===
// (GMMAssignment 已在文件开头定义)

std::vector<GMMAssignment> assignGMMTasks(
    const std::vector<Eigen::Vector3d>& drone_positions,
    const std::vector<Eigen::Vector3d>& gmm_centers,
    const std::vector<int>& gmm_ids,
    const Eigen::VectorXd& gmm_pi) {

  int num_drones = drone_positions.size();
  int num_clusters = gmm_centers.size();
  std::vector<GMMAssignment> assignments;

  ROS_INFO("[dpf%d] Task assignment: %d drones, %d clusters", drone_id_, num_drones, num_clusters);

  if (num_clusters > num_drones) {
    // 簇数 > 无人机数：选择权重最高的 num_drones 个簇
    std::vector<int> sorted_indices(num_clusters);
    std::iota(sorted_indices.begin(), sorted_indices.end(), 0);
    std::sort(sorted_indices.begin(), sorted_indices.end(),
              [&](int a, int b) { return gmm_pi(a) > gmm_pi(b); });

    // 为每个无人机分配一个簇
    for (int d = 0; d < num_drones; ++d) {
      int cluster_idx = sorted_indices[d];
      double dist = (drone_positions[d] - gmm_centers[cluster_idx]).norm();
      assignments.push_back({d, gmm_ids[cluster_idx], dist});
      ROS_INFO("  Drone %d -> Cluster %d (GMM_ID=%d, weight=%.3f, dist=%.2f)",
               d, cluster_idx, gmm_ids[cluster_idx], gmm_pi(cluster_idx), dist);
    }
  } else if (num_clusters == num_drones) {
    // 簇数 = 无人机数：一对一分配（最优匹配）
    // 构建代价矩阵
    Eigen::MatrixXd cost_matrix(num_drones, num_clusters);
    for (int d = 0; d < num_drones; ++d) {
      for (int c = 0; c < num_clusters; ++c) {
        cost_matrix(d, c) = (drone_positions[d] - gmm_centers[c]).norm();
      }
    }

    // 贪心匹配
    std::vector<bool> cluster_assigned(num_clusters, false);
    for (int d = 0; d < num_drones; ++d) {
      int best_c = -1;
      double best_dist = 1e9;
      for (int c = 0; c < num_clusters; ++c) {
        if (!cluster_assigned[c] && cost_matrix(d, c) < best_dist) {
          best_c = c;
          best_dist = cost_matrix(d, c);
        }
      }
      if (best_c >= 0) {
        cluster_assigned[best_c] = true;
        assignments.push_back({d, gmm_ids[best_c], best_dist});
        ROS_INFO("  Drone %d -> Cluster %d (GMM_ID=%d, weight=%.3f, dist=%.2f)",
                 d, best_c, gmm_ids[best_c], gmm_pi(best_c), best_dist);
      }
    }
  } else {
    // 簇数 < 无人机数：先保证每个簇有一个无人机，剩余无人机分配到权重最高的簇
    // 第一步：为每个簇分配最近的无人机
    std::vector<bool> drone_assigned(num_drones, false);
    for (int c = 0; c < num_clusters; ++c) {
      int best_d = -1;
      double best_dist = 1e9;
      for (int d = 0; d < num_drones; ++d) {
        if (!drone_assigned[d]) {
          double dist = (drone_positions[d] - gmm_centers[c]).norm();
          if (dist < best_dist) {
            best_d = d;
            best_dist = dist;
          }
        }
      }
      if (best_d >= 0) {
        drone_assigned[best_d] = true;
        assignments.push_back({best_d, gmm_ids[c], best_dist});
        ROS_INFO("  Drone %d -> Cluster %d (GMM_ID=%d, weight=%.3f, dist=%.2f)",
                 best_d, c, gmm_ids[c], gmm_pi(c), best_dist);
      }
    }

    // 第二步：剩余无人机分配到权重最高的簇
    int max_weight_cluster = 0;
    for (int c = 1; c < num_clusters; ++c) {
      if (gmm_pi(c) > gmm_pi(max_weight_cluster)) {
        max_weight_cluster = c;
      }
    }
    for (int d = 0; d < num_drones; ++d) {
      if (!drone_assigned[d]) {
        double dist = (drone_positions[d] - gmm_centers[max_weight_cluster]).norm();
        assignments.push_back({d, gmm_ids[max_weight_cluster], dist});
        ROS_INFO("  Drone %d -> Cluster %d (GMM_ID=%d, weight=%.3f, dist=%.2f) [extra]",
                 d, max_weight_cluster, gmm_ids[max_weight_cluster], gmm_pi(max_weight_cluster), dist);
      }
    }
  }

  return assignments;
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
  nh.param("consensus_rate", consensus_rate_, 100);
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
  search_pos_gmm_pub_ = nh.advertise<target_ekf::LocalStats>("search_pos_gmm", 1);
  search_gmm_vis_pub_ = nh.advertise<visualization_msgs::MarkerArray>("search_gmm_vis", 1);
  search_particles_vis_pub_ = nh.advertise<sensor_msgs::PointCloud2>("search_particles_vis", 1);
  search_targets_pub_ = nh.advertise<geometry_msgs::PoseArray>("search_targets", 1);
  search_label_info_pub_ = nh.advertise<target_ekf::SearchLabelInfo>("search_label_info", 1);
  
  // 地图订阅
  ros::Subscriber global_map_sub = nh.subscribe("global_map", 10, &global_map_callback);
  // 无人机位姿订阅（单独订阅，不再同步）
  ros::Subscriber odom_sub = nh.subscribe("odom", 100, &odom_callback, ros::TransportHints().tcpNoDelay());
  // YOLO目标订阅（单独订阅，不再同步）
  ros::Subscriber yolo_sub = nh.subscribe("yolo", 1, &yolo_callback, ros::TransportHints().tcpNoDelay());
  // 订阅其他无人机的局部统计量和odom
  for (int i = 0; i < num_drones_; ++i) {
    if (i == drone_id_) continue;
    std::string topic = "/drone" + std::to_string(i) + "/drone" + std::to_string(i) + "_target_dpf/local_stats";
    stats_subs_.push_back(nh.subscribe(topic, 10, &stats_callback));
    std::string labeled_topic = "/drone" + std::to_string(i) + "/drone" + std::to_string(i) + "_target_dpf/labeled_consensus";
    stats_subs_.push_back(nh.subscribe(labeled_topic, 10, &labeled_consensus_callback));

    // 订阅邻居无人机的odom（用于匈牙利分配）
    std::string odom_topic = "/drone" + std::to_string(i) + "/odom";
    neighbor_odom_subs_.push_back(
        nh.subscribe<nav_msgs::Odometry>(odom_topic, 10,
            boost::bind(&neighbor_odom_callback, _1, i)));
    ROS_INFO("[dpf%d] Subscribing to neighbor %d: stats, labeled_consensus, odom (%s)", drone_id_, i, odom_topic.c_str());
  }
  // 【核心】观测Timer（20Hz）+ 共识Timer（100Hz）
  ros::Timer dpf_core_timer = nh.createTimer(ros::Duration(1.0 / dpf_rate_), &dpf_core_timer_callback);
  ros::Timer consensus_timer = nh.createTimer(ros::Duration(1.0 / consensus_rate_), &consensus_timer_callback);
  ros::MultiThreadedSpinner spinner(4); // 多线程Spinner，避免回调阻塞
  spinner.spin();
  return 0;
}