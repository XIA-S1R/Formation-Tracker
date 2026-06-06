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
#include <quadrotor_msgs/OccMap3d.h>
#include <mapping/mapping.h>
#include <unordered_set>
#include <cmath>
#include <mutex>
#include <atomic>
#include <target_ekf/target_ekf.hpp>
#include <target_ekf/target_dpf.hpp>
#include <target_ekf/LocalStats.h>
#include <target_ekf/SearchBranchHealth.h>
#include <target_ekf/InvalidRegionGMM.h>
#include <std_msgs/Bool.h>
#include <unordered_set>
#include <unordered_map>
#include <algorithm>
#include <numeric>
#include <cctype>
#include <cstdint>
#include <deque>
#include "target_ekf/search_particles_manager.hpp"

// === 前向声明 ===
struct GMMAssignment {
  int drone_id;
  int gmm_id;
  double distance;
};

struct SearchCoverageAssignment {
  int drone_vec_index = -1;
  int drone_id = -1;
  int hotspot_index = -1;
  double utility = -1e9;
  double distance = 0.0;
  double overlap_penalty = 0.0;
  double reuse_penalty = 0.0;
};

struct SearchViewCandidate {
  int hotspot_index = -1;
  Eigen::Vector3d pos = Eigen::Vector3d::Zero();
  double yaw = 0.0;
  double visible_mass = 0.0;
  std::vector<uint8_t> visible_mask;
};

struct SearchViewAssignment {
  int drone_vec_index = -1;
  int drone_id = -1;
  int candidate_index = -1;
  int hotspot_index = -1;
  double utility = -1e9;
  double gain_mass = 0.0;
  double overlap_mass = 0.0;
  double position_penalty = 0.0;
  double same_hotspot_penalty = 0.0;
  double travel_cost = 0.0;
  double visible_mass = 0.0;
  double distance = 0.0;
};

void matchGMMCenters(const std::vector<Eigen::Vector3d>& current_mu,
                     const Eigen::VectorXd& current_pi,
                     double distance_threshold = 2.0);
std::vector<GMMAssignment> assignGMMTasks(const std::vector<Eigen::Vector3d>& drone_positions,
                                          const std::vector<Eigen::Vector3d>& gmm_centers,
                                          const std::vector<int>& gmm_ids,
                                          const Eigen::VectorXd& gmm_weights);
bool isLineOfSightClear(const Eigen::Vector3d& start, const Eigen::Vector3d& end);
std::vector<SearchCoverageAssignment> assignSearchHotspotsForCoverage(
    const std::vector<std::pair<int, Eigen::Vector3d>>& drone_positions,
    const std::vector<SearchParticlesManager::SearchHotspot>& hotspots);
std::vector<SearchViewAssignment> assignSearchViewsForCoverage(
    const std::vector<std::pair<int, Eigen::Vector3d>>& drone_positions,
    const std::vector<SearchViewCandidate>& candidates,
    const std::vector<double>& w_norm);
std::vector<SearchParticlesManager::SearchHotspot> spreadSearchHotspots(
    const std::vector<SearchParticlesManager::SearchHotspot>& hotspots,
    int max_keep,
    double min_separation_m);

// === 全局变量 ===
// 发布器
ros::Publisher target_odom_pub_;
ros::Publisher local_stats_pub_;
ros::Publisher search_state_pub_;
ros::Publisher search_particles_vis_pre_prune_pub_;
ros::Publisher search_particles_vis_pub_;
ros::Publisher search_particles_vis_post_prune_pub_;
ros::Publisher search_branch_health_pub_;
ros::Publisher search_fallback_model_pub_;
ros::Publisher search_gmm_vis_pub_; // 搜索GMM分布可视化发布器
ros::Publisher search_pos_gmm_pub_;  // 位置GMM发布器
ros::Publisher search_targets_pub_;  // 搜索目标点发布器
ros::Publisher invalid_grid_pub_;      // 无效区域栅格发布器（消息类型沿用InvalidRegionGMM）
std::vector<ros::Subscriber> stats_subs_;
std::vector<ros::Subscriber> invalid_grid_subs_; // 邻居无效区域栅格订阅
std::vector<ros::Subscriber> search_branch_health_subs_;

// 相机外参
Eigen::Matrix3d cam2body_R_;
Eigen::Vector3d cam2body_p_;
double fx_, fy_, cx_, cy_, width_, height_;
double pitch_thr_ = 30;
bool check_fov_ = false;
double max_obs_depth_ = 8.0;  // 相机最大观测深度(m)

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
Eigen::Vector3d latest_obs_vel_ = Eigen::Vector3d::Zero();

// rollback验证：记录失锁开始时的参考真值（来自yolo/target odom）
bool rollback_validation_enable_ = true;
bool rollback_loss_ref_valid_ = false;
ros::Time rollback_loss_ref_stamp_;
Eigen::Vector3d rollback_loss_ref_pos_ = Eigen::Vector3d::Zero();
Eigen::Vector3d rollback_loss_ref_vel_ = Eigen::Vector3d::Zero();

// 搜索初始化缓存：持续维护“最近若干个有观测帧”的可靠快照；
// 失锁开始(0->1)时冻结一份，供达到threshold后入搜初始化使用
struct SearchInitSnapshot {
  ros::Time stamp;
  Eigen::MatrixXd particles;
  Eigen::VectorXd weights;
};
int reliable_snapshot_history_size_ = 12;
std::deque<SearchInitSnapshot> reliable_snapshot_ring_;
std::deque<SearchInitSnapshot> frozen_loss_snapshot_ring_;
bool loss_snapshot_frozen_ = false;

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
int miss_detection_num_ = 5; // 连续多少帧都没有观测后进入搜索模式
int consecutive_no_obs_count_ = 0; // 连续无观测计数
bool search_mode_active_ = false; // 是否处于搜索模式
enum class SearchEstimationMode {
  kSearchGmm6D = 0,
  kLegacyDpf = 1,
};
SearchEstimationMode search_estimation_mode_ = SearchEstimationMode::kSearchGmm6D;
ros::Time last_global_obs_time_;
bool reacquire_boost_active_ = false;                  // 重捕获后短时加强跟踪窗口
ros::Time reacquire_boost_end_time_ = ros::Time(0);    // 加强跟踪结束时间
double reacquire_boost_duration_sec_ = 3.0;            // 加强跟踪持续时长
double reacquire_boost_miss_scale_ = 3.0;              // 回搜阈值放大倍数
int reacquire_boost_miss_min_ = 0;                     // 回搜阈值最小值（<=0表示不限制）

// 搜索方向承诺机制
Eigen::Vector3d committed_search_dir_ = Eigen::Vector3d::Zero();
Eigen::Vector3d committed_target_pos_ = Eigen::Vector3d::Zero(); // 当前推进目标
ros::Time last_hotspot_extract_time_ = ros::Time(0);
double hotspot_extract_interval_base_sec_ = 0.5;   // 热点重提取固定周期(秒)
double hotspot_extract_interval_growth_sec_ = 0.0; // 兼容旧参数，当前不再使用
double hotspot_extract_interval_max_sec_ = 2.0;    // 兼容旧参数，当前固定等于base
double current_hotspot_extract_interval_sec_ = 0.5; // 当前生效的固定重提取周期(秒)
int committed_direction_refresh_count_ = 0;         // 当前搜索阶段内热点重提取次数
bool has_committed_direction_ = false;
double search_advance_vmax_ = 2.0; // 搜索模式推进速度，优先使用当前无人机 planning/vmax
double neg_obs_ttl_sec_ = 0.5;               // 负观测无效区域记忆时长（秒）
// 调试开关：为真时才做粒子云/可见性评估和点云发布（默认关，生产态省 CPU）
bool dbg_visualize_ = false;
bool search_debug_logs_ = false;
// 全图无效栅格扫描节流频率；<=0 表示每帧都扫
double invalid_grid_update_hz_ = 5.0;
ros::Time last_invalid_grid_update_stamp_ = ros::Time(0);
int search_mode_frame_count_ = 0;            // 已进入搜索模式后的帧计数（仅搜索主循环）
int search_spread_init_frames_ = 8;          // 搜索初期强制分散帧数
double search_spread_speed_min_ratio_ = 0.8; // 分散阶段速度下界比例（相对search_vmax）
double search_spread_theta_jitter_deg_ = 10.0; // 分散阶段角度抖动（度）
double search_mixed_turn_ratio_ = 0.50;      // 常规搜索阶段：每个重选周期内赋予omega的粒子比例
double search_mixed_turn_theta_sigma_deg_ = 90.0; // 常规搜索阶段：每个重选周期内累计转角标准差（度）
int search_mixed_turn_hold_steps_ = 5;       // mixed更新中，每组omega持续的帧数
double search_ess_resample_ratio_ = 0.50;          // 搜索期ESS重采样阈值比例
double search_resample_jitter_pos_m_ = 0.15;       // 搜索重采样后位置抖动标准差(m)
double search_resample_jitter_vel_mps_ = 0.20;     // 搜索重采样后速度抖动标准差(m/s)
double search_branch_retained_mass_ratio_ = 0.35;   // 分支失效判据：裁剪后保留质量比例下限
double search_branch_strong_decay_ratio_ = 0.55;    // 分支失效判据：强削权粒子比例上限
double search_branch_strong_phi_threshold_ = 0.35;  // 强削权判定阈值(phi<=threshold)
double search_branch_neff_ratio_ = 0.25;            // 分支失效判据：Neff比例下限
double search_branch_model_age_sec_ = 0.60;          // 可接管模型最大时延(s)
int search_em_session_target_iters_ = 12;          // 任务重分配前EM会话目标迭代轮数
double search_em_session_timeout_sec_ = 0.25;      // 任务重分配前EM会话超时(s)
bool search_em_wait_for_allocation_ = true;        // 是否等待EM会话完成后再重分配
bool search_em_session_active_ = false;            // 当前是否处于EM会话窗口
int search_em_session_done_iters_ = 0;             // 当前EM会话已执行轮数
ros::Time search_em_session_deadline_ = ros::Time(0); // 当前EM会话截止时间
double invalid_decay_phi_min_ = 0.08;        // 论文2.2: 深无效区最小权重衰减（推荐初值）
double invalid_decay_phi_mid_ = 0.95;        // 论文2.2: 边界处权重衰减（推荐初值）
double invalid_decay_r_occ_ = -0.50;         // 论文2.2: 无效区内截断距离(m, <0)（推荐初值）
double invalid_decay_r_safe_ = 0.05;         // 论文2.2: 有效区安全距离(m, >0)（推荐初值）
double hotspot_min_drone_dist_ = 2.0;        // 热点与最近无人机最小期望距离
double hotspot_seed_radius_ = 1.5;           // 热点局部融合半径
double hotspot_invalid_reject_ratio_ = 1.0;  // 无效密度拒绝阈值（相对prune阈值）
double search_assign_weight_scale_ = 25.0;   // 搜索分配：热点权重收益系数
double search_assign_distance_cost_ = 1.0;   // 搜索分配：路径距离代价系数
double search_assign_overlap_radius_m_ = 4.0; // 搜索分配：热点重叠判定半径
double search_assign_overlap_penalty_ = 6.0; // 搜索分配：与已分配热点重叠惩罚
double search_assign_reuse_penalty_ = 8.0;   // 搜索分配：热点被重复复用的惩罚
bool search_hold_assigned_target_ = true;    // 搜索分配后保持当前任务点，交给规划器稳定A*过去
double search_hotspot_min_separation_m_ = 4.0; // 热点最小间距，避免多个热点塌到同一团粒子上
double search_view_standoff_radius_m_ = 4.0;   // 视位：围绕热点的观测半径
int search_view_candidate_azimuths_ = 8;       // 视位：每个热点生成的环形候选数
double search_view_gain_scale_ = 100.0;         // 视位：可见质量收益缩放
double search_view_overlap_penalty_ = 140.0;    // 视位：重叠质量惩罚
double search_view_position_penalty_radius_m_ = 6.0; // 视位：空间过近判定半径
double search_view_position_penalty_ = 80.0;         // 视位：空间过近惩罚
double search_view_same_hotspot_penalty_ = 120.0;    // 视位：多机围同一热点惩罚
double search_view_travel_cost_scale_ = 1.0;    // 视位：航程代价缩放
double search_view_min_visible_mass_ = 0.01;     // 视位：最小可见质量阈值

// 无效区域虚拟栅格存储（并集共识）
using InvalidGrid2D = SearchParticlesManager::InvalidGrid2D;
std::mutex invalid_grid_mutex_;
// 栅格参数（与场景范围对齐，在main中初始化）
double grid_origin_x_ = -10.0;
double grid_origin_y_ = -15.0;
double grid_resolution_ = 0.15;
int grid_nx_ = 80;
int grid_ny_ = 80;
double grid_map_size_x_ = -1.0;
double grid_map_size_y_ = -1.0;
InvalidGrid2D global_invalid_grid_;  // 全局无效区域（本机历史 + 邻机/本机地图增量在搜索主循环中统一并入）
struct PendingInvalidGridDelta {
  double recv_stamp_sec = 0.0;
  InvalidGrid2D frame_grid;
};
std::unordered_map<int, PendingInvalidGridDelta> pending_invalid_grid_deltas_;
std::mutex local_map_cache_mutex_;
quadrotor_msgs::OccMap3dConstPtr pending_local_occ_map_msg_;
std::vector<uint8_t> cached_local_occ_2d_;
ros::Time cached_local_occ_stamp_ = ros::Time(0);
int cached_local_occ_min_ix_ = 0;
int cached_local_occ_max_ix_ = -1;
int cached_local_occ_min_iy_ = 0;
int cached_local_occ_max_iy_ = -1;
InvalidGrid2D scratch_frame_grid_;

bool useSearchGmm6DMode() {
  return search_estimation_mode_ == SearchEstimationMode::kSearchGmm6D;
}

void resetCommittedDirectionState() {
  has_committed_direction_ = false;
  committed_search_dir_.setZero();
  committed_target_pos_.setZero();
  last_hotspot_extract_time_ = ros::Time(0);
  committed_direction_refresh_count_ = 0;
  current_hotspot_extract_interval_sec_ = hotspot_extract_interval_base_sec_;
}

int getCurrentMissDetectionThreshold() {
  const int base = std::max(1, miss_detection_num_);
  if (!reacquire_boost_active_) {
    return base;
  }
  int boosted = std::max(base + 1, (int)std::ceil(base * reacquire_boost_miss_scale_));
  if (reacquire_boost_miss_min_ > 0) {
    boosted = std::max(boosted, reacquire_boost_miss_min_);
  }
  return boosted;
}

void activateReacquireBoost(const char* reason) {
  if (reacquire_boost_duration_sec_ <= 0.0) return;
  reacquire_boost_active_ = true;
  reacquire_boost_end_time_ = ros::Time::now() + ros::Duration(reacquire_boost_duration_sec_);
  ROS_WARN("[dpf%d] BOOSTED TRACKING ON (%s): hold=%.2fs, miss_threshold=%d->%d",
           drone_id_, reason, reacquire_boost_duration_sec_,
           std::max(1, miss_detection_num_), getCurrentMissDetectionThreshold());
}

void updateReacquireBoostState() {
  if (reacquire_boost_active_ && ros::Time::now() >= reacquire_boost_end_time_) {
    reacquire_boost_active_ = false;
    ROS_WARN("[dpf%d] BOOSTED TRACKING OFF: miss_threshold back to %d",
             drone_id_, std::max(1, miss_detection_num_));
  }
}

void startSearchEmSession(const ros::Time& now, const char* reason) {
  search_em_session_active_ = true;
  search_em_session_done_iters_ = 0;
  const double timeout = std::max(0.05, search_em_session_timeout_sec_);
  search_em_session_deadline_ = now + ros::Duration(timeout);
  ROS_INFO("[dpf%d] Search EM session started (%s): target_iters=%d timeout=%.3fs",
           drone_id_, reason ? reason : "unknown",
           std::max(1, search_em_session_target_iters_), timeout);
}

void clearSearchEmSession(const char* reason) {
  if (search_em_session_active_ || search_em_session_done_iters_ > 0) {
    ROS_INFO("[dpf%d] Search EM session cleared (%s): done_iters=%d",
             drone_id_, reason ? reason : "clear", search_em_session_done_iters_);
  }
  search_em_session_active_ = false;
  search_em_session_done_iters_ = 0;
  search_em_session_deadline_ = ros::Time(0);
}

bool isSearchEmSessionFinished(const ros::Time& now) {
  if (!search_em_session_active_) return true;
  const bool reached_iters =
      search_em_session_done_iters_ >= std::max(1, search_em_session_target_iters_);
  const bool timeout = (search_em_session_deadline_.isValid() && now >= search_em_session_deadline_);
  return reached_iters || timeout;
}

struct ParticleKinematicSummary {
  Eigen::Vector3d pos_mean = Eigen::Vector3d::Zero();
  Eigen::Vector3d vel_mean = Eigen::Vector3d::Zero();
};

ParticleKinematicSummary computeParticleKinematicSummary(
    const Eigen::MatrixXd& particles, const Eigen::VectorXd& weights) {
  ParticleKinematicSummary s;
  if (particles.rows() < 6 || particles.cols() <= 0) return s;
  const int N = particles.cols();
  const bool use_weights = (weights.size() == N);

  if (!use_weights) {
    s.pos_mean = particles.topRows(3).rowwise().mean();
    s.vel_mean = particles.block(3, 0, 3, N).rowwise().mean();
    return s;
  }

  double w_sum = weights.sum();
  if (!(w_sum > 1e-12)) {
    s.pos_mean = particles.topRows(3).rowwise().mean();
    s.vel_mean = particles.block(3, 0, 3, N).rowwise().mean();
    return s;
  }
  for (int i = 0; i < N; ++i) {
    const double w = weights(i) / w_sum;
    s.pos_mean += w * particles.col(i).head(3);
    s.vel_mean += w * particles.col(i).segment(3, 3);
  }
  return s;
}

void trimReliableSnapshotRing() {
  const size_t max_keep = static_cast<size_t>(std::max(1, reliable_snapshot_history_size_));
  while (reliable_snapshot_ring_.size() > max_keep) {
    reliable_snapshot_ring_.pop_front();
  }
}

void cacheReliableSnapshotFromDPF(const char* reason) {
  if (!dpfPtr_ || !dpfPtr_->initialized_) return;
  auto snap = dpfPtr_->getParticlesAndWeights();
  if (snap.first.cols() <= 0 || snap.second.size() != snap.first.cols()) return;

  SearchInitSnapshot item;
  item.stamp = ros::Time::now();
  item.particles = std::move(snap.first);
  item.weights = std::move(snap.second);
  reliable_snapshot_ring_.push_back(std::move(item));
  trimReliableSnapshotRing();

  const auto& newest = reliable_snapshot_ring_.back();
  ROS_DEBUG_THROTTLE(0.5,
                     "[dpf%d] Reliable snapshot cached (%s): live_ring=%zu max=%d newest_t=%.3f N=%d",
                     drone_id_, reason ? reason : "obs",
                     reliable_snapshot_ring_.size(), std::max(1, reliable_snapshot_history_size_),
                     newest.stamp.toSec(), dpfPtr_->N_);
}

void freezeReliableSnapshotRing(const char* reason) {
  if (reliable_snapshot_ring_.empty()) {
    frozen_loss_snapshot_ring_.clear();
    loss_snapshot_frozen_ = false;
    ROS_WARN("[dpf%d] Freeze reliable snapshot ring (%s) failed: live ring empty",
             drone_id_, reason ? reason : "loss_start");
    return;
  }
  frozen_loss_snapshot_ring_ = reliable_snapshot_ring_;
  loss_snapshot_frozen_ = true;
  const auto& newest = frozen_loss_snapshot_ring_.back();
  ROS_WARN("[dpf%d] Freeze reliable snapshot ring (%s): frozen=%zu newest_t=%.3f N=%d",
           drone_id_, reason ? reason : "loss_start",
           frozen_loss_snapshot_ring_.size(), newest.stamp.toSec(), dpfPtr_ ? dpfPtr_->N_ : -1);
}

void clearFrozenSnapshotRing(const char* reason) {
  if (loss_snapshot_frozen_ || !frozen_loss_snapshot_ring_.empty()) {
    ROS_INFO("[dpf%d] Clear frozen reliable snapshot ring (%s)",
             drone_id_, reason ? reason : "clear");
  }
  frozen_loss_snapshot_ring_.clear();
  loss_snapshot_frozen_ = false;
}

const SearchInitSnapshot* getFrozenSnapshot() {
  if (frozen_loss_snapshot_ring_.empty()) return nullptr;
  return &frozen_loss_snapshot_ring_.back();
}

// 6D搜索共识邻居数据
using NeighborConsensus6D = SearchParticlesManager::NeighborConsensus6D;
using LocalStat6D = SearchParticlesManager::LocalStat6D;
struct SearchFallbackModel {
  target_ekf::LocalStats msg;
  ros::Time model_stamp;
  ros::Time health_stamp;
  bool healthy = false;
  double retained_mass_ratio = 0.0;
  double strong_decay_ratio = 0.0;
  double neff_ratio = 0.0;
  double mean_phi = 0.0;
  double health_score = 0.0;
};
std::map<int, SearchFallbackModel> received_search_fallback_models_;
std::mutex search_fallback_mutex_;
std::map<int, NeighborConsensus6D> received_search_consensus_;
std::mutex search_consensus_mutex_;
ros::Publisher search_consensus_pub_;
std::vector<ros::Subscriber> search_consensus_subs_;
std::vector<ros::Subscriber> search_fallback_model_subs_;

// 邻居无人机位置（用于搜索热点分配）
std::map<int, Eigen::Vector3d> neighbor_positions_;
std::mutex neighbor_odom_mutex_;
std::vector<ros::Subscriber> neighbor_odom_subs_;

// 搜索粒子管理器
std::unique_ptr<SearchParticlesManager> search_particles_manager_;

// 邻居数据存储
std::map<int, LocalStat> received_stats_;
std::map<int, NeighborConsensus> received_consensus_;
std::mutex stats_mutex_;

// === GMM中心帧间匹配相关 ===
std::vector<Eigen::Vector3d> prev_pos_gmm_mu_;  // 上一帧的GMM中心
std::vector<int> gmm_center_ids_;               // 每个中心的持久ID
int next_gmm_id_ = 0;                           // 下一个可用的ID
std::mt19937 rng_;                              // 随机数生成器

double computeEffectiveSampleSize(const Eigen::VectorXd& weights) {
  if (weights.size() <= 0) return 0.0;
  const double sum_w = weights.sum();
  if (!(sum_w > 1e-300)) return 0.0;
  const Eigen::VectorXd wn = weights / sum_w;
  const double sum_w2 = wn.squaredNorm();
  if (!(sum_w2 > 1e-300)) return 0.0;
  return 1.0 / sum_w2;
}

double computeSearchHotspotOverlap(
    const SearchParticlesManager::SearchHotspot& a,
    const SearchParticlesManager::SearchHotspot& b,
    double radius_m) {
  const double radius = std::max(1e-3, radius_m);
  const double d = (a.pos - b.pos).norm();
  const double kernel = std::exp(-0.5 * d * d / (radius * radius));
  return kernel * std::min(a.weight, b.weight);
}

std::vector<SearchCoverageAssignment> assignSearchHotspotsForCoverage(
    const std::vector<std::pair<int, Eigen::Vector3d>>& drone_positions,
    const std::vector<SearchParticlesManager::SearchHotspot>& hotspots) {
  std::vector<SearchCoverageAssignment> assignments;
  const int nd = static_cast<int>(drone_positions.size());
  const int nh = static_cast<int>(hotspots.size());
  if (nd <= 0 || nh <= 0) return assignments;

  std::vector<bool> drone_assigned(nd, false);
  std::vector<int> hotspot_use_count(nh, 0);
  // 热点数足够时禁止复用；不足时允许复用，但用显式惩罚压制多机重叠。
  const bool allow_hotspot_reuse = (nh < nd);

  while (static_cast<int>(assignments.size()) < nd) {
    SearchCoverageAssignment best;
    bool found = false;
    for (int d = 0; d < nd; ++d) {
      if (drone_assigned[d]) continue;
      for (int h = 0; h < nh; ++h) {
        if (!allow_hotspot_reuse && hotspot_use_count[h] > 0) continue;

        const double dist = (drone_positions[d].second - hotspots[h].pos).norm();
        double overlap_penalty = 0.0;
        for (const auto& prev : assignments) {
          if (prev.hotspot_index < 0 || prev.hotspot_index >= nh) continue;
          overlap_penalty += computeSearchHotspotOverlap(
              hotspots[h], hotspots[prev.hotspot_index], search_assign_overlap_radius_m_);
        }
        const double reuse_penalty =
            search_assign_reuse_penalty_ * static_cast<double>(hotspot_use_count[h]) *
            std::max(0.0, hotspots[h].weight);
        const double utility =
            search_assign_weight_scale_ * hotspots[h].weight -
            search_assign_distance_cost_ * dist -
            search_assign_overlap_penalty_ * overlap_penalty -
            reuse_penalty;

        if (!found || utility > best.utility) {
          found = true;
          best.drone_vec_index = d;
          best.drone_id = drone_positions[d].first;
          best.hotspot_index = h;
          best.utility = utility;
          best.distance = dist;
          best.overlap_penalty = overlap_penalty;
          best.reuse_penalty = reuse_penalty;
        }
      }
    }

    if (!found) break;
    drone_assigned[best.drone_vec_index] = true;
    hotspot_use_count[best.hotspot_index]++;
    assignments.push_back(best);
  }

  return assignments;
}

std::vector<SearchParticlesManager::SearchHotspot> spreadSearchHotspots(
    const std::vector<SearchParticlesManager::SearchHotspot>& hotspots,
    int max_keep,
    double min_separation_m) {
  std::vector<SearchParticlesManager::SearchHotspot> selected;
  if (hotspots.empty() || max_keep <= 0) return selected;

  const double min_sep = std::max(0.0, min_separation_m);
  selected.reserve(std::min<int>(max_keep, hotspots.size()));

  for (const auto& h : hotspots) {
    bool too_close = false;
    for (const auto& prev : selected) {
      if ((h.pos - prev.pos).norm() < min_sep) {
        too_close = true;
        break;
      }
    }
    if (too_close) continue;
    selected.push_back(h);
    if (static_cast<int>(selected.size()) >= max_keep) break;
  }

  if (selected.empty()) {
    selected.push_back(hotspots.front());
  }

  return selected;
}

struct ParticleCloudDebugSummary {
  int N = 0;
  double neff = 0.0;
  Eigen::Vector3d mean = Eigen::Vector3d::Zero();
  double rms = 0.0;
  double w_max = 0.0;
};

ParticleCloudDebugSummary summarizeParticleCloud(const Eigen::MatrixXd& particles,
                                                 const Eigen::VectorXd& weights) {
  ParticleCloudDebugSummary s;
  if (particles.rows() < 3) return s;
  const int N = std::min<int>(particles.cols(), weights.size());
  if (N <= 0) return s;
  s.N = N;

  Eigen::VectorXd w = weights.head(N);
  for (int i = 0; i < N; ++i) {
    if (!std::isfinite(w(i)) || w(i) < 0.0) w(i) = 0.0;
  }

  double sum_w = w.sum();
  if (!(sum_w > 1e-300)) {
    w.setConstant(1.0 / N);
    sum_w = 1.0;
  }
  w /= sum_w;

  s.neff = computeEffectiveSampleSize(w);
  s.w_max = w.maxCoeff();

  for (int i = 0; i < N; ++i) {
    s.mean += w(i) * particles.col(i).head<3>();
  }
  double var = 0.0;
  for (int i = 0; i < N; ++i) {
    const Eigen::Vector3d diff = particles.col(i).head<3>() - s.mean;
    var += w(i) * diff.squaredNorm();
  }
  s.rms = std::sqrt(std::max(0.0, var));
  return s;
}

void publishParticleCloud(const Eigen::MatrixXd& particles,
                          const Eigen::VectorXd& weights,
                          ros::Publisher& pub) {
  if (particles.rows() < 3) return;
  const int N = std::min<int>(particles.cols(), weights.size());
  if (N <= 0) return;

  pcl::PointCloud<pcl::PointXYZI> cloud;
  cloud.header.frame_id = "world";
  cloud.points.reserve(N);

  double w_max = 0.0;
  for (int i = 0; i < N; ++i) {
    const double w = std::isfinite(weights(i)) ? std::max(0.0, weights(i)) : 0.0;
    w_max = std::max(w_max, w);
  }
  if (!(w_max > 1e-12)) w_max = 1.0;

  for (int i = 0; i < N; ++i) {
    const double x = particles(0, i);
    const double y = particles(1, i);
    const double z = particles(2, i);
    if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) continue;

    pcl::PointXYZI pt;
    pt.x = static_cast<float>(x);
    pt.y = static_cast<float>(y);
    pt.z = static_cast<float>(z);
    const double w = std::isfinite(weights(i)) ? std::max(0.0, weights(i)) : 0.0;
    pt.intensity = static_cast<float>(w / w_max);
    cloud.points.push_back(pt);
  }

  sensor_msgs::PointCloud2 cloud_msg;
  pcl::toROSMsg(cloud, cloud_msg);
  cloud_msg.header.stamp = ros::Time::now();
  cloud_msg.header.frame_id = "world";
  pub.publish(cloud_msg);
}

void logParticleCloudSummary(const char* stage,
                             const ParticleCloudDebugSummary& s) {
  ROS_INFO_THROTTLE(0.5,
                    "[dpf%d] Search cloud %s: N=%d Neff=%.1f mean=(%.2f,%.2f,%.2f) rms=%.2f wmax=%.3f",
                    drone_id_, stage ? stage : "unknown",
                    s.N, s.neff,
                    s.mean.x(), s.mean.y(), s.mean.z(),
                    s.rms, s.w_max);
}

struct SearchVisibilityStats {
  int total_particles = 0;
  int in_depth_particles = 0;
  int in_fov_particles = 0;
  int los_clear_particles = 0;
  int los_blocked_particles = 0;

  double in_depth_ratio = 0.0;      // in depth / total
  double in_fov_ratio = 0.0;        // in FOV / total
  double los_clear_ratio = 0.0;     // LOS clear / total
  double los_blocked_in_fov_ratio = 0.0; // LOS blocked / in FOV
};

SearchVisibilityStats evaluateSearchVisibility(const Eigen::MatrixXd& particles,
                                               int N,
                                               const Eigen::Vector3d& cam_p,
                                               const Eigen::Quaterniond& cam_q) {
  SearchVisibilityStats s;
  if (N <= 0 || particles.rows() < 3 || particles.cols() < N) return s;
  s.total_particles = N;

  const Eigen::Matrix3d R_cw = cam_q.toRotationMatrix().transpose();
  for (int i = 0; i < N; ++i) {
    const Eigen::Vector3d p_w = particles.col(i).head<3>();
    const Eigen::Vector3d p_c = R_cw * (p_w - cam_p);
    if (!(p_c.z() > 0.1 && p_c.z() < max_obs_depth_)) continue;
    ++s.in_depth_particles;

    const double u = p_c.x() * fx_ / p_c.z() + cx_;
    const double v = p_c.y() * fy_ / p_c.z() + cy_;
    if (!(u >= 0.0 && u <= width_ && v >= 0.0 && v <= height_)) continue;
    ++s.in_fov_particles;

    if (isLineOfSightClear(cam_p, p_w)) {
      ++s.los_clear_particles;
    } else {
      ++s.los_blocked_particles;
    }
  }

  const double total = static_cast<double>(std::max(1, s.total_particles));
  s.in_depth_ratio = static_cast<double>(s.in_depth_particles) / total;
  s.in_fov_ratio = static_cast<double>(s.in_fov_particles) / total;
  s.los_clear_ratio = static_cast<double>(s.los_clear_particles) / total;
  if (s.in_fov_particles > 0) {
    s.los_blocked_in_fov_ratio =
        static_cast<double>(s.los_blocked_particles) / static_cast<double>(s.in_fov_particles);
  }
  return s;
}

struct SearchVisibilityMassEval {
  double visible_mass = 0.0;
  int visible_particles = 0;
  std::vector<uint8_t> visible_mask;
};

std::vector<double> normalizeParticleWeights(const Eigen::VectorXd& weights, int N) {
  const int n = std::min<int>(N, weights.size());
  std::vector<double> w(n, 0.0);
  double sum_w = 0.0;
  for (int i = 0; i < n; ++i) {
    const double v = std::isfinite(weights(i)) ? std::max(0.0, weights(i)) : 0.0;
    w[i] = v;
    sum_w += v;
  }
  if (!(sum_w > 1e-300)) {
    if (n > 0) {
      const double uni = 1.0 / static_cast<double>(n);
      std::fill(w.begin(), w.end(), uni);
    }
    return w;
  }
  for (double& v : w) v /= sum_w;
  return w;
}

SearchVisibilityMassEval evaluateSearchVisibilityMass(const Eigen::MatrixXd& particles,
                                                      const Eigen::VectorXd& weights,
                                                      int N,
                                                      const Eigen::Vector3d& cam_p,
                                                      const Eigen::Quaterniond& cam_q) {
  SearchVisibilityMassEval eval;
  if (N <= 0 || particles.rows() < 3 || particles.cols() < N || weights.size() < N) return eval;
  eval.visible_mask.assign(N, 0);
  const std::vector<double> w_norm = normalizeParticleWeights(weights, N);
  const Eigen::Matrix3d R_cw = cam_q.toRotationMatrix().transpose();
  for (int i = 0; i < N; ++i) {
    const Eigen::Vector3d p_w = particles.col(i).head<3>();
    const Eigen::Vector3d p_c = R_cw * (p_w - cam_p);
    if (!(p_c.z() > 0.1 && p_c.z() < max_obs_depth_)) continue;
    const double u = p_c.x() * fx_ / p_c.z() + cx_;
    const double v = p_c.y() * fy_ / p_c.z() + cy_;
    if (!(u >= 0.0 && u <= width_ && v >= 0.0 && v <= height_)) continue;
    if (!isLineOfSightClear(cam_p, p_w)) continue;
    eval.visible_mask[i] = 1;
    eval.visible_mass += w_norm[i];
    ++eval.visible_particles;
  }
  return eval;
}

double computeVisibilityOverlapMass(const std::vector<uint8_t>& a,
                                    const std::vector<uint8_t>& b,
                                    const std::vector<double>& w_norm) {
  const int n = std::min<int>(std::min<int>(a.size(), b.size()), w_norm.size());
  double mass = 0.0;
  for (int i = 0; i < n; ++i) {
    if (a[i] && b[i]) mass += w_norm[i];
  }
  return mass;
}

std::vector<SearchViewCandidate> buildSearchViewCandidates(
    const std::vector<SearchParticlesManager::SearchHotspot>& hotspots,
    const std::vector<std::pair<int, Eigen::Vector3d>>& drone_positions,
    const Eigen::MatrixXd& particles,
    const Eigen::VectorXd& weights,
    int N,
    const SearchParticlesManager::InvalidGrid2D& invalid_grid) {
  std::vector<SearchViewCandidate> candidates;
  if (hotspots.empty() || N <= 0) return candidates;

  const double standoff = std::max(0.5, search_view_standoff_radius_m_);
  const int az_num = std::max(1, search_view_candidate_azimuths_);

  double nominal_z = 2.5;
  if (!drone_positions.empty()) {
    double sum_z = 0.0;
    for (const auto& dp : drone_positions) sum_z += dp.second.z();
    nominal_z = std::max(2.0, sum_z / static_cast<double>(drone_positions.size()));
  }

  for (size_t h = 0; h < hotspots.size(); ++h) {
    const auto& hs = hotspots[h];
    std::vector<Eigen::Vector3d> poses;
    poses.reserve(static_cast<size_t>(az_num) + 1);

    // 先尝试热点中心本身，yaw指向局部密集侧
    {
      Eigen::Vector3d center = hs.pos;
      center.z() = std::max(nominal_z, 2.0);
      poses.push_back(center);
    }

    for (int k = 0; k < az_num; ++k) {
      const double ang = 2.0 * M_PI * static_cast<double>(k) / static_cast<double>(az_num);
      Eigen::Vector3d p = hs.pos + Eigen::Vector3d(standoff * std::cos(ang),
                                                   standoff * std::sin(ang),
                                                   0.0);
      p.z() = std::max(nominal_z, 2.0);
      poses.push_back(p);
    }

    for (const auto& p : poses) {
      if (invalid_grid.valid()) {
        int ix = 0, iy = 0;
        invalid_grid.toCell(p.x(), p.y(), ix, iy);
        if (!invalid_grid.inBounds(ix, iy)) continue;
        if (invalid_grid.get(ix, iy) != 0) continue;
      }

      SearchViewCandidate cand;
      cand.hotspot_index = static_cast<int>(h);
      cand.pos = p;
      cand.yaw = std::atan2(hs.pos.y() - p.y(), hs.pos.x() - p.x());
      const Eigen::Quaterniond q(Eigen::AngleAxisd(cand.yaw, Eigen::Vector3d::UnitZ()));
      const auto vis = evaluateSearchVisibilityMass(particles, weights, N, cand.pos, q);
      cand.visible_mass = vis.visible_mass;
      cand.visible_mask = std::move(vis.visible_mask);
      if (cand.visible_mass < search_view_min_visible_mass_) {
        // 先保留低质量候选用于兜底，不在此处直接丢弃
      }
      candidates.push_back(std::move(cand));
    }
  }

  std::sort(candidates.begin(), candidates.end(),
            [](const SearchViewCandidate& a, const SearchViewCandidate& b) {
              if (a.visible_mass != b.visible_mass) return a.visible_mass > b.visible_mass;
              return a.hotspot_index < b.hotspot_index;
            });
  return candidates;
}

std::vector<SearchViewAssignment> assignSearchViewsForCoverage(
    const std::vector<std::pair<int, Eigen::Vector3d>>& drone_positions,
    const std::vector<SearchViewCandidate>& candidates,
    const std::vector<double>& w_norm) {
  std::vector<SearchViewAssignment> assignments;
  const int nd = static_cast<int>(drone_positions.size());
  const int nc = static_cast<int>(candidates.size());
  if (nd <= 0 || nc <= 0) return assignments;

  const int n_mask = static_cast<int>(candidates.front().visible_mask.size());
  if (n_mask <= 0 || static_cast<int>(w_norm.size()) < n_mask) return assignments;

  std::vector<bool> drone_assigned(nd, false);
  std::vector<int> candidate_use_count(nc, 0);
  std::vector<uint8_t> union_mask(n_mask, 0);
  const bool allow_candidate_reuse = (nc < nd);

  while (static_cast<int>(assignments.size()) < nd) {
    SearchViewAssignment best;
    bool found = false;

    for (int d = 0; d < nd; ++d) {
      if (drone_assigned[d]) continue;
      for (int c = 0; c < nc; ++c) {
        if (!allow_candidate_reuse && candidate_use_count[c] > 0) continue;
        const auto& cand = candidates[c];
        if (static_cast<int>(cand.visible_mask.size()) != n_mask) continue;

        double gain_mass = 0.0;
        double overlap_mass = 0.0;
        double position_penalty = 0.0;
        double same_hotspot_penalty = 0.0;
        for (int i = 0; i < n_mask; ++i) {
          if (!cand.visible_mask[i]) continue;
          if (union_mask[i]) overlap_mass += w_norm[i];
          else gain_mass += w_norm[i];
        }
        for (const auto& prev : assignments) {
          if (prev.candidate_index < 0 || prev.candidate_index >= nc) continue;
          const auto& prev_cand = candidates[prev.candidate_index];
          const double dpos = (cand.pos - prev_cand.pos).norm();
          const double rpos = std::max(1e-3, search_view_position_penalty_radius_m_);
          position_penalty += std::exp(-0.5 * dpos * dpos / (rpos * rpos));
          if (cand.hotspot_index == prev_cand.hotspot_index) {
            same_hotspot_penalty += 1.0;
          }
        }
        const double dist = (drone_positions[d].second - cand.pos).norm();
        // rate-of-coverage: 单位时间内能压下去的粒子质量
        // search_view_travel_cost_scale_ 语义改为饱和时间常数 τ(秒)，
        // 防止零距离除 0，并压制"就地小收益"候选。
        const double vmax = std::max(0.1, search_advance_vmax_);
        const double eta = dist / vmax;
        const double tau = std::max(1e-3, search_view_travel_cost_scale_);
        const double net_gain =
            search_view_gain_scale_ * gain_mass -
            search_view_overlap_penalty_ * overlap_mass -
            search_view_position_penalty_ * position_penalty -
            search_view_same_hotspot_penalty_ * same_hotspot_penalty;
        const double utility = net_gain / (eta + tau);
        if (!found || utility > best.utility) {
          found = true;
          best.drone_vec_index = d;
          best.drone_id = drone_positions[d].first;
          best.candidate_index = c;
          best.hotspot_index = cand.hotspot_index;
          best.utility = utility;
          best.gain_mass = gain_mass;
          best.overlap_mass = overlap_mass;
          best.position_penalty = position_penalty;
          best.same_hotspot_penalty = same_hotspot_penalty;
          best.travel_cost = dist;
          best.visible_mass = cand.visible_mass;
          best.distance = dist;
        }
      }
    }

    if (!found) break;
    drone_assigned[best.drone_vec_index] = true;
    candidate_use_count[best.candidate_index]++;
    const auto& selected_mask = candidates[best.candidate_index].visible_mask;
    for (int i = 0; i < n_mask; ++i) {
      if (selected_mask[i]) union_mask[i] = 1;
    }
    assignments.push_back(best);
  }

  return assignments;
}

void applySearchResampleJitter(Eigen::MatrixXd& particles) {
  if (particles.cols() <= 0 || particles.rows() < 6) return;
  const double pos_sigma = std::max(0.0, search_resample_jitter_pos_m_);
  const double vel_sigma = std::max(0.0, search_resample_jitter_vel_mps_);
  if (pos_sigma <= 0.0 && vel_sigma <= 0.0) return;

  std::normal_distribution<double> pos_noise(0.0, pos_sigma);
  std::normal_distribution<double> vel_noise(0.0, vel_sigma);
  for (int i = 0; i < particles.cols(); ++i) {
    if (pos_sigma > 0.0) {
      particles(0, i) += pos_noise(rng_);
      particles(1, i) += pos_noise(rng_);
    }
    if (vel_sigma > 0.0) {
      particles(3, i) += vel_noise(rng_);
      particles(4, i) += vel_noise(rng_);
    }
  }
}

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

// === 全局地图回调（仅用于LOS检查）===
void global_map_callback(const sensor_msgs::PointCloud2ConstPtr& msg) {
  pcl::PointCloud<pcl::PointXYZ> cloud;
  pcl::fromROSMsg(*msg, cloud);
  occMap_.fromPointCloud(cloud, 0.3);
  ROS_INFO_ONCE("[dpf%d] Global map received, %zu obstacle points.", drone_id_, cloud.size());
}

// === 局部地图回调（用于障碍无效栅格标记）===
void local_map_callback(const quadrotor_msgs::OccMap3dConstPtr& msg) {
  std::lock_guard<std::mutex> lock(local_map_cache_mutex_);
  pending_local_occ_map_msg_ = msg;
  if (grid_nx_ <= 0 || grid_ny_ <= 0) {
    return;
  }
  if ((int)cached_local_occ_2d_.size() != grid_nx_ * grid_ny_) {
    cached_local_occ_2d_.assign(grid_nx_ * grid_ny_, 0);
  } else {
    std::fill(cached_local_occ_2d_.begin(), cached_local_occ_2d_.end(), 0);
  }

  mapping::OccGridMap local_map;
  local_map.from_msg(*msg);
  const double local_min_x = local_map.offset_x * local_map.resolution;
  const double local_min_y = local_map.offset_y * local_map.resolution;
  const double local_max_x = (local_map.offset_x + local_map.size_x) * local_map.resolution;
  const double local_max_y = (local_map.offset_y + local_map.size_y) * local_map.resolution;
  cached_local_occ_min_ix_ = std::max(0, static_cast<int>(std::floor((local_min_x - grid_origin_x_) / grid_resolution_)));
  cached_local_occ_max_ix_ = std::min(grid_nx_ - 1, static_cast<int>(std::ceil((local_max_x - grid_origin_x_) / grid_resolution_)) - 1);
  cached_local_occ_min_iy_ = std::max(0, static_cast<int>(std::floor((local_min_y - grid_origin_y_) / grid_resolution_)));
  cached_local_occ_max_iy_ = std::min(grid_ny_ - 1, static_cast<int>(std::ceil((local_max_y - grid_origin_y_) / grid_resolution_)) - 1);
  if (cached_local_occ_min_ix_ > cached_local_occ_max_ix_ ||
      cached_local_occ_min_iy_ > cached_local_occ_max_iy_) {
    cached_local_occ_min_ix_ = 0;
    cached_local_occ_max_ix_ = -1;
    cached_local_occ_min_iy_ = 0;
    cached_local_occ_max_iy_ = -1;
    cached_local_occ_stamp_ = msg->header.stamp;
    return;
  }
  const double z_sample = 3.0;
  for (int iy = cached_local_occ_min_iy_; iy <= cached_local_occ_max_iy_; ++iy) {
    for (int ix = cached_local_occ_min_ix_; ix <= cached_local_occ_max_ix_; ++ix) {
      double wx = grid_origin_x_ + (ix + 0.5) * grid_resolution_;
      double wy = grid_origin_y_ + (iy + 0.5) * grid_resolution_;
      if (local_map.isOccupied(Eigen::Vector3d(wx, wy, z_sample))) {
        cached_local_occ_2d_[iy * grid_nx_ + ix] = 1;
      }
    }
  }
  cached_local_occ_stamp_ = msg->header.stamp;
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

// === 未初始化时借用邻机共识状态进行初始化 ===
bool tryBootstrapFromNeighborConsensus(Eigen::Vector3d& seed_pos,
                                       Eigen::Vector3d& seed_rpy,
                                       int& src_drone_id) {
  std::lock_guard<std::mutex> lock(stats_mutex_);
  const ros::Time now = ros::Time::now();
  const double max_consensus_age = 0.5;

  double best_alpha = -1.0;
  bool found = false;

  for (const auto& kv : received_consensus_) {
    const int neighbor_id = kv.first;
    const auto& nc = kv.second;
    const double age = (now - nc.timestamp).toSec();
    if (age > max_consensus_age || !nc.has_obs) continue;

    const int C = static_cast<int>(nc.zeta_alpha.size());
    for (int c = 0; c < C; ++c) {
      if (c >= static_cast<int>(nc.zeta_a.size())) continue;
      if (nc.zeta_a[c].size() < 9) continue;
      const double alpha = nc.zeta_alpha(c);
      if (alpha <= 1e-6 || alpha <= best_alpha) continue;

      Eigen::VectorXd state = nc.zeta_a[c] / alpha;
      if (!state.array().isFinite().all()) continue;

      seed_pos = state.head<3>();
      seed_rpy = state.segment<3>(6);
      seed_rpy.x() = wrapAngle(seed_rpy.x());
      seed_rpy.y() = wrapAngle(seed_rpy.y());
      seed_rpy.z() = wrapAngle(seed_rpy.z());
      src_drone_id = neighbor_id;
      best_alpha = alpha;
      found = true;
    }
  }
  return found;
}

// === 无效区域栅格消息转换 ===
target_ekf::InvalidRegionGMM toInvalidGridMsg(const InvalidGrid2D& grid, int drone_id) {
  target_ekf::InvalidRegionGMM msg;
  msg.header.stamp = ros::Time::now();
  msg.drone_id = drone_id;
  msg.origin_x = grid.origin_x;
  msg.origin_y = grid.origin_y;
  msg.resolution = grid.resolution;
  msg.nx = grid.nx;
  msg.ny = grid.ny;
  auto rle = SearchParticlesManager::rleEncode(grid);
  msg.rle_values = rle.values;
  msg.rle_counts = rle.counts;
  msg.rle_frames.clear();  // 兼容旧消息字段，当前实现不再使用帧号
  return msg;
}

InvalidGrid2D fromInvalidGridMsg(const target_ekf::InvalidRegionGMM::ConstPtr& msg) {
  SearchParticlesManager::RLEGrid rle;
  rle.origin_x = msg->origin_x;
  rle.origin_y = msg->origin_y;
  rle.resolution = msg->resolution;
  rle.nx = msg->nx;
  rle.ny = msg->ny;
  rle.values = msg->rle_values;
  rle.counts = msg->rle_counts;
  return SearchParticlesManager::rleDecode(rle);
}

// === 收到邻居无效区域栅格增量的回调 ===
void invalid_grid_callback(const target_ekf::InvalidRegionGMM::ConstPtr& msg) {
  if (msg->drone_id == drone_id_) return;
  std::lock_guard<std::mutex> lock(invalid_grid_mutex_);
  // 回调中只缓存邻机增量；在搜索主循环按帧统一融合，保证时序一致性
  PendingInvalidGridDelta delta;
  delta.recv_stamp_sec = ros::Time::now().toSec();
  delta.frame_grid = fromInvalidGridMsg(msg);
  pending_invalid_grid_deltas_[msg->drone_id] = delta;
}

// === 6D搜索共识消息转换（复用LocalStats消息，state_dim=6）===
target_ekf::LocalStats toSearchConsensusMsg(const LocalStat6D& ls, const SearchParticlesManager& mgr) {
  target_ekf::LocalStats msg;
  msg.header.stamp = ls.timestamp.isZero() ? ros::Time::now() : ls.timestamp;
  msg.drone_id = ls.drone_id;
  msg.num_components = mgr.getSearchC();
  msg.state_dim = 6;
  msg.obs_dim = 0;
  msg.has_observation = ls.has_obs;
  int C = mgr.getSearchC();
  msg.alpha_local.resize(C);
  msg.a_local.resize(C * 6);
  msg.b_local.resize(C * 6 * 6);
  for (int c = 0; c < C; ++c) {
    msg.alpha_local[c] = ls.alpha(c);
    for (int i = 0; i < 6; ++i) msg.a_local[c * 6 + i] = ls.a[c](i);
    for (int i = 0; i < 6; ++i)
      for (int j = 0; j < 6; ++j)
        msg.b_local[c * 36 + i * 6 + j] = ls.b[c](i, j);
  }
  // 打包共识状态ζ
  const auto& za = mgr.getSearchZetaAlpha();
  const auto& zav = mgr.getSearchZetaA();
  const auto& zbv = mgr.getSearchZetaB();
  msg.zeta_alpha.resize(C);
  msg.zeta_a.resize(C * 6);
  msg.zeta_b.resize(C * 6 * 6);
  for (int c = 0; c < C; ++c) {
    msg.zeta_alpha[c] = za(c);
    for (int i = 0; i < 6; ++i) msg.zeta_a[c * 6 + i] = zav[c](i);
    for (int i = 0; i < 6; ++i)
      for (int j = 0; j < 6; ++j)
        msg.zeta_b[c * 36 + i * 6 + j] = zbv[c](i, j);
  }
  return msg;
}

void search_consensus_callback(const target_ekf::LocalStats::ConstPtr& msg) {
  if (msg->drone_id == drone_id_) return;
  if (msg->state_dim != 6) return; // 只接收6D搜索共识
  std::lock_guard<std::mutex> lock(search_consensus_mutex_);
  NeighborConsensus6D nc;
  nc.has_obs = msg->has_observation;
  nc.timestamp = msg->header.stamp;
  int C = msg->num_components;
  nc.zeta_alpha = Eigen::Map<const Eigen::VectorXd>(msg->zeta_alpha.data(), C);
  nc.zeta_a.resize(C);
  nc.zeta_b.resize(C);
  for (int c = 0; c < C; ++c) {
    nc.zeta_a[c] = Eigen::Map<const Eigen::VectorXd>(msg->zeta_a.data() + c * 6, 6);
    nc.zeta_b[c] = Eigen::Map<const Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>(
        msg->zeta_b.data() + c * 36, 6, 6);
  }
  received_search_consensus_[msg->drone_id] = nc;
}

SearchParticlesManager::SearchFallbackGMM6D parseSearchFallbackModel(const target_ekf::LocalStats& msg) {
  SearchParticlesManager::SearchFallbackGMM6D model;
  if (msg.state_dim != 6 || msg.num_components <= 0) return model;

  const int C = msg.num_components;
  model.C = C;
  model.pi = Eigen::VectorXd::Zero(C);
  model.mu.resize(C);
  model.S.resize(C);
  double pi_sum = 0.0;
  for (int c = 0; c < C; ++c) {
    const double alpha = (c < (int)msg.zeta_alpha.size()) ? std::max(0.0, msg.zeta_alpha[c]) : 0.0;
    model.pi(c) = alpha;
    pi_sum += alpha;
    model.mu[c].setZero(6);
    model.S[c].setIdentity(6, 6);
    if (alpha <= 1e-12) continue;
    model.mu[c] = Eigen::Map<const Eigen::VectorXd>(msg.zeta_a.data() + c * 6, 6) / alpha;
    model.S[c] = Eigen::Map<const Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>(
        msg.zeta_b.data() + c * 36, 6, 6) / alpha;
    model.S[c] += Eigen::MatrixXd::Identity(6, 6) * 1e-4;
  }
  if (!(pi_sum > 1e-300)) return model;
  model.pi /= pi_sum;
  model.valid = true;
  return model;
}

target_ekf::LocalStats toSearchFallbackModelMsg(const SearchParticlesManager::SearchFallbackGMM6D& model,
                                                int drone_id, int N) {
  target_ekf::LocalStats msg;
  msg.header.stamp = ros::Time::now();
  msg.header.frame_id = "world";
  msg.drone_id = drone_id;
  msg.num_components = model.C;
  msg.state_dim = 6;
  msg.obs_dim = 0;
  msg.has_observation = false;
  msg.alpha_local.resize(model.C);
  msg.a_local.resize(model.C * 6);
  msg.b_local.resize(model.C * 36);
  msg.zeta_alpha.resize(model.C);
  msg.zeta_a.resize(model.C * 6);
  msg.zeta_b.resize(model.C * 36);
  for (int c = 0; c < model.C; ++c) {
    const double alpha = std::max(1e-12, model.pi(c) * static_cast<double>(std::max(1, N)));
    msg.alpha_local[c] = alpha;
    msg.zeta_alpha[c] = alpha;
    for (int i = 0; i < 6; ++i) {
      const double val = model.mu[c](i) * alpha;
      msg.a_local[c * 6 + i] = val;
      msg.zeta_a[c * 6 + i] = val;
    }
    for (int i = 0; i < 6; ++i) {
      for (int j = 0; j < 6; ++j) {
        const double val = model.S[c](i, j) * alpha;
        msg.b_local[c * 36 + i * 6 + j] = val;
        msg.zeta_b[c * 36 + i * 6 + j] = val;
      }
    }
  }
  return msg;
}

void search_branch_health_callback(const target_ekf::SearchBranchHealth::ConstPtr& msg) {
  if (msg->drone_id == drone_id_) return;
  std::lock_guard<std::mutex> lock(search_fallback_mutex_);
  auto& slot = received_search_fallback_models_[msg->drone_id];
  slot.healthy = msg->healthy;
  slot.retained_mass_ratio = msg->retained_mass_ratio;
  slot.strong_decay_ratio = msg->strong_decay_ratio;
  slot.neff_ratio = msg->neff_ratio;
  slot.mean_phi = msg->mean_phi;
  slot.health_score = msg->health_score;
  slot.health_stamp = msg->header.stamp;
}

void search_fallback_model_callback(const target_ekf::LocalStats::ConstPtr& msg) {
  if (msg->drone_id == drone_id_) return;
  if (msg->state_dim != 6 || msg->num_components <= 0) return;
  std::lock_guard<std::mutex> lock(search_fallback_mutex_);
  auto& slot = received_search_fallback_models_[msg->drone_id];
  slot.msg = *msg;
  slot.model_stamp = msg->header.stamp;
}

bool tryRecoverSearchBranchFromHealthyNeighbor() {
  if (!dpfPtr_ || !dpfPtr_->initialized_) return false;
  const ros::Time now = ros::Time::now();
  const double max_age = std::max(0.1, search_branch_model_age_sec_);

  int best_id = -1;
  SearchParticlesManager::SearchFallbackGMM6D best_model;
  double best_score = -1.0;
  double best_retained = 0.0;
  double best_strong = 0.0;
  double best_neff = 0.0;
  double best_mean_phi = 0.0;
  {
    std::lock_guard<std::mutex> lock(search_fallback_mutex_);
    for (const auto& kv : received_search_fallback_models_) {
      const int neighbor_id = kv.first;
      const auto& slot = kv.second;
      const double model_age = slot.model_stamp.isZero() ? 1e9 : (now - slot.model_stamp).toSec();
      const double health_age = slot.health_stamp.isZero() ? 1e9 : (now - slot.health_stamp).toSec();
      if (!slot.healthy || model_age > max_age || health_age > max_age) continue;
      if (slot.msg.state_dim != 6 || slot.msg.num_components <= 0) continue;
      const double score = std::max(1e-6, slot.health_score);
      if (score <= best_score) continue;
      SearchParticlesManager::SearchFallbackGMM6D model = parseSearchFallbackModel(slot.msg);
      if (!model.valid) continue;
      best_score = score;
      best_id = neighbor_id;
      best_model = std::move(model);
      best_retained = slot.retained_mass_ratio;
      best_strong = slot.strong_decay_ratio;
      best_neff = slot.neff_ratio;
      best_mean_phi = slot.mean_phi;
    }
  }

  if (best_id < 0 || !best_model.valid) return false;

  search_particles_manager_->sampleParticlesFromSearchGMMModel(
      dpfPtr_->particles_, dpfPtr_->weights_, dpfPtr_->N_, best_model);
  applySearchResampleJitter(dpfPtr_->particles_);

  ROS_WARN("[dpf%d] Search branch recovered from neighbor %d: score=%.3f retained=%.3f strong=%.3f neff=%.3f mean_phi=%.3f",
           drone_id_, best_id, best_score, best_retained, best_strong, best_neff, best_mean_phi);
  return true;
}

void publishSearchBranchHealthMsg(bool healthy,
                                  double retained_mass_ratio,
                                  double strong_decay_ratio,
                                  double neff_ratio,
                                  double mean_phi) {
  target_ekf::SearchBranchHealth msg;
  msg.header.stamp = ros::Time::now();
  msg.drone_id = drone_id_;
  msg.healthy = healthy;
  msg.retained_mass_ratio = retained_mass_ratio;
  msg.strong_decay_ratio = strong_decay_ratio;
  msg.neff_ratio = neff_ratio;
  msg.mean_phi = mean_phi;
  msg.health_score = std::max(0.0, retained_mass_ratio) *
                     std::max(0.0, 1.0 - strong_decay_ratio) *
                     std::max(0.0, neff_ratio);
  search_branch_health_pub_.publish(msg);
}

bool publishSearchFallbackModelFromCurrentParticles() {
  if (!dpfPtr_ || !dpfPtr_->initialized_) return false;
  const int C = std::max(1, search_particles_manager_->getSearchC());
  auto model = search_particles_manager_->buildSearchFallbackGMM(
      dpfPtr_->particles_, dpfPtr_->weights_, dpfPtr_->N_, C);
  if (!model.valid) return false;
  search_fallback_model_pub_.publish(toSearchFallbackModelMsg(model, drone_id_, dpfPtr_->N_));
  return true;
}

// 失锁对比模式：把普通DPF的9D GMM投影为6D(pos+vel)并发布到search_pos_gmm
void publishSearchPosGMMFromLegacyDPF() {
  if (!dpfPtr_ || !dpfPtr_->initialized_) return;
  const int C = dpfPtr_->C_;
  if (C <= 0 || (int)dpfPtr_->gmm_pi_.size() != C) return;

  target_ekf::LocalStats gmm_msg;
  gmm_msg.header.stamp = ros::Time::now();
  gmm_msg.header.frame_id = "world";
  gmm_msg.drone_id = drone_id_;
  gmm_msg.num_components = C;
  gmm_msg.state_dim = 6;
  gmm_msg.obs_dim = 0;
  gmm_msg.has_observation = false;

  gmm_msg.zeta_alpha.resize(C);
  gmm_msg.zeta_a.resize(C * 6, 0.0);
  gmm_msg.zeta_b.resize(C * 36, 0.0);

  for (int c = 0; c < C; ++c) {
    if (c >= (int)dpfPtr_->gmm_mu_.size() || c >= (int)dpfPtr_->gmm_S_.size()) continue;
    const auto& mu9 = dpfPtr_->gmm_mu_[c];
    const auto& S9 = dpfPtr_->gmm_S_[c];
    if (mu9.size() < 6 || S9.rows() < 6 || S9.cols() < 6) continue;

    // 与search_gmm口径对齐：zeta_alpha/a/b，满足 mu = a/alpha, S = b/alpha
    const double alpha = std::max(1e-12, dpfPtr_->gmm_pi_(c) * dpfPtr_->N_);
    gmm_msg.zeta_alpha[c] = alpha;
    for (int i = 0; i < 6; ++i) {
      gmm_msg.zeta_a[c * 6 + i] = mu9(i) * alpha;
    }
    for (int i = 0; i < 6; ++i) {
      for (int j = 0; j < 6; ++j) {
        gmm_msg.zeta_b[c * 36 + i * 6 + j] = S9(i, j) * alpha;
      }
    }
  }

  search_pos_gmm_pub_.publish(gmm_msg);
}

// 发布当前DPF粒子云（真实粒子，不是GMM），用于失锁后直观可视化
void publishSearchParticlesCloudFromDPF() {
  if (!dpfPtr_ || !dpfPtr_->initialized_) return;
  publishParticleCloud(dpfPtr_->particles_, dpfPtr_->weights_, search_particles_vis_pub_);
}

// === 邻居无人机odom回调：存储邻居位置用于搜索热点分配 ===
void neighbor_odom_callback(const nav_msgs::OdometryConstPtr& msg, int neighbor_id) {
  std::lock_guard<std::mutex> lock(neighbor_odom_mutex_);
  neighbor_positions_[neighbor_id] = Eigen::Vector3d(
      msg->pose.pose.position.x,
      msg->pose.pose.position.y,
      msg->pose.pose.position.z);
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
  latest_obs_vel_.x() = target_msg->twist.twist.linear.x;
  latest_obs_vel_.y() = target_msg->twist.twist.linear.y;
  latest_obs_vel_.z() = target_msg->twist.twist.linear.z;
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
  if (!dpfPtr_->initialized_) return;

  if (search_mode_active_ && useSearchGmm6DMode()) {
    // === 搜索模式：事件化6D共识（仅EM会话窗口内迭代） ===
    if (!search_particles_manager_->isSearchGMMInitialized()) return;
    if (!search_em_session_active_) return;
    std::lock_guard<std::mutex> dpf_lock(dpf_mutex_);

    // 收集邻居6D共识状态
    std::vector<NeighborConsensus6D> neighbors;
    {
      std::lock_guard<std::mutex> lock(search_consensus_mutex_);
      ros::Time now = ros::Time::now();
      for (auto& kv : received_search_consensus_) {
        if ((now - kv.second.timestamp).toSec() < 0.5) {
          neighbors.push_back(kv.second);
        }
      }
    }

    // E步 + 共识 + M步
    bool obs_flag = has_recent_obs_.load();
    LocalStat6D ls = search_particles_manager_->emStep6D(
        dpfPtr_->particles_, dpfPtr_->weights_, dpfPtr_->N_,
        drone_id_, obs_flag, neighbors);
    ++search_em_session_done_iters_;

    // 发布6D共识状态给邻居
    search_consensus_pub_.publish(toSearchConsensusMsg(ls, *search_particles_manager_));
    const ros::Time now = ros::Time::now();
    const bool reached_iters =
        search_em_session_done_iters_ >= std::max(1, search_em_session_target_iters_);
    const bool timeout =
        (search_em_session_deadline_.isValid() && now >= search_em_session_deadline_);
    if (reached_iters || timeout) {
      ROS_INFO("[dpf%d] Search EM session completed: done_iters=%d target_iters=%d timeout=%d",
               drone_id_, search_em_session_done_iters_,
               std::max(1, search_em_session_target_iters_), timeout ? 1 : 0);
      search_em_session_active_ = false;
    }
    return;
  }

  // === 普通模式：9D共识（原逻辑）===
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
  updateReacquireBoostState();
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
          if (p_in_body.z() > 0.1 && p_in_body.z() < max_obs_depth_) {
            double x = p_in_body.x() * fx_ / p_in_body.z() + cx_;
            double y = p_in_body.y() * fy_ / p_in_body.z() + cy_;
            if (x >= 0 && x <= width_ && y >= 0 && y <= height_) {
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
  std::unique_lock<std::mutex> dpf_lock(dpf_mutex_);
  // 初始化/重置逻辑
  double update_dt = (ros::Time::now() - last_update_stamp_).toSec();
  bool need_reset = (!dpfPtr_->initialized_ || update_dt > 1.0);
  if (need_reset) {
    if (has_obs) {
      dpfPtr_->reset(obs_pos, obs_rpy);
      dpf_reset_suppress_count_ = 3;
      clearFrozenSnapshotRing("reset_with_obs");
      cacheReliableSnapshotFromDPF("reset_with_obs");
      ROS_WARN("[dpf%d] reset at obs=(%.2f,%.2f,%.2f) dt=%.2fs",
        drone_id_, obs_pos.x(), obs_pos.y(), obs_pos.z(), update_dt);
      last_update_stamp_ = ros::Time::now();
      return;
    } else {
      // 本机无观测且尚未初始化：尝试借用邻机共识状态进行初始化
      if (!dpfPtr_->initialized_) {
        Eigen::Vector3d seed_pos, seed_rpy;
        int src_drone_id = -1;
        if (tryBootstrapFromNeighborConsensus(seed_pos, seed_rpy, src_drone_id)) {
          dpfPtr_->reset(seed_pos, seed_rpy);
          dpf_reset_suppress_count_ = 3;
          has_recent_obs_.store(false);
          ROS_WARN("[dpf%d] bootstrap init from neighbor%d consensus: pos=(%.2f,%.2f,%.2f), rpy=(%.2f,%.2f,%.2f)",
                   drone_id_, src_drone_id,
                   seed_pos.x(), seed_pos.y(), seed_pos.z(),
                   seed_rpy.x(), seed_rpy.y(), seed_rpy.z());
          last_update_stamp_ = ros::Time::now();
          return;
        }
      }
      ROS_DEBUG_THROTTLE(1.0, "[dpf%d] no obs, skip reset (initialized=%d)", 
        drone_id_, dpfPtr_->initialized_);
      if (!dpfPtr_->initialized_) return;
    }
  }
  // ==============================================
  // 【模式判断 - 在回调开始时判断当前应执行的模式】
  // ==============================================
  ROS_DEBUG("[dpf%d] has_obs=%d, initialized=%d", drone_id_, has_obs, dpfPtr_->initialized_);
  if (search_mode_active_ && useSearchGmm6DMode()) {
    // =========================
    // 【搜索模式处理流程】
    // =========================

    // --- 优先退出检查1：本无人机发现目标 → 直接从观测重置，不走共识 ---
    if (has_obs) {
      dpfPtr_->reset(obs_pos, obs_rpy);
      dpf_reset_suppress_count_ = 3;
      clearFrozenSnapshotRing("search_self_reacquired");
      cacheReliableSnapshotFromDPF("search_self_reacquired");
      search_mode_active_ = false;
      consecutive_no_obs_count_ = 0;
      search_mode_frame_count_ = 0;
      rollback_loss_ref_valid_ = false;
      {
        std::lock_guard<std::mutex> lock(invalid_grid_mutex_);
        pending_invalid_grid_deltas_.clear();
      }
      resetCommittedDirectionState();
      activateReacquireBoost("self_reacquired");
      clearSearchEmSession("search_self_reacquired");
      ROS_WARN("[dpf%d] EXITING SEARCH MODE: Target reacquired, resetting from direct observation!", drone_id_);

      // 发布 local_stats（has_obs=true），通知邻居此机已重新观测到目标
      LocalStat local_stat = dpfPtr_->computeLocalStatsOnly(drone_id_, true);
      local_stats_pub_.publish(toMsg(local_stat, *dpfPtr_));

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
          search_mode_frame_count_ = 0;
          rollback_loss_ref_valid_ = false;
          {
            std::lock_guard<std::mutex> lock(invalid_grid_mutex_);
            pending_invalid_grid_deltas_.clear();
          }
          resetCommittedDirectionState();
          activateReacquireBoost("neighbor_reacquired");
          clearFrozenSnapshotRing("search_neighbor_reacquired");
          clearSearchEmSession("search_neighbor_reacquired");
          ROS_WARN("[dpf%d] EXITING SEARCH MODE: Neighbor drone %d reacquired target!", drone_id_, kv.first);
          break;
        }
      }
    }
    if (!search_mode_active_) {
      std_msgs::Bool search_state_msg;
      search_state_msg.data = false;
      search_state_pub_.publish(search_state_msg);
      return;
    }

    // ===== 新搜索方案 =====
    // 搜索模式更新频率与普通DPF主循环一致（dpf_rate）

    const int spread_frames = std::max(0, search_spread_init_frames_);
    if (search_mode_frame_count_ < spread_frames) {
      // 搜索初期几帧：不沿用当前速度方向，强制粒子在XY平面全向扩散
      search_particles_manager_->searchParticlesWideSpreadUpdate(
          dpfPtr_->particles_, dpfPtr_->N_,
          search_mode_frame_count_, spread_frames,
          search_spread_speed_min_ratio_, search_spread_theta_jitter_deg_);
      ROS_INFO_THROTTLE(0.5, "[dpf%d] Search spread phase: frame=%d/%d (speed_min_ratio=%.2f, jitter=%.1fdeg)",
                        drone_id_, search_mode_frame_count_ + 1, spread_frames,
                        search_spread_speed_min_ratio_, search_spread_theta_jitter_deg_);
    } else {
      // 分散期结束后采用“稳定+转向”混合更新（每隔若干帧重选一批持续转向粒子）
      const int mixed_step = std::max(0, search_mode_frame_count_ - spread_frames);
      int turning_count = 0;
      search_particles_manager_->searchParticlesMixedUpdate(
          dpfPtr_->particles_, dpfPtr_->N_,
          mixed_step,
          search_mixed_turn_hold_steps_,
          search_mixed_turn_ratio_,
          search_mixed_turn_theta_sigma_deg_,
          &turning_count);
      ROS_INFO_THROTTLE(0.5,
                        "[dpf%d] Search mixed update: step=%d N=%d turning=%d (ratio=%.2f, theta_sigma=%.1fdeg, hold=%d)",
                        drone_id_, mixed_step, dpfPtr_->N_, turning_count,
                        search_mixed_turn_ratio_, search_mixed_turn_theta_sigma_deg_,
                        search_mixed_turn_hold_steps_);
    }
    ++search_mode_frame_count_;

    if (dbg_visualize_) {
      const SearchVisibilityStats vis_stats = evaluateSearchVisibility(
          dpfPtr_->particles_, dpfPtr_->N_, cam_p, cam_q);
      ROS_INFO_THROTTLE(
          0.5,
          "[dpf%d] Search visibility: total=%d depth=%.1f%% (%d) fov=%.1f%% (%d) los_clear=%.1f%% (%d) los_blocked_in_fov=%.1f%% (%d)",
          drone_id_,
          vis_stats.total_particles,
          100.0 * vis_stats.in_depth_ratio, vis_stats.in_depth_particles,
          100.0 * vis_stats.in_fov_ratio, vis_stats.in_fov_particles,
          100.0 * vis_stats.los_clear_ratio, vis_stats.los_clear_particles,
          100.0 * vis_stats.los_blocked_in_fov_ratio, vis_stats.los_blocked_particles);
    }

    // 用FOV扫描增量更新全局无效栅格（全图执行，不再使用活跃窗口裁剪）
    // 本段是搜索期最贵的计算（全图局部障碍扫描 + fov footprint），以 invalid_grid_update_hz 节流。
    const ros::Time grid_now = ros::Time::now();
    const bool do_invalid_grid_update =
        (invalid_grid_update_hz_ <= 0.0) ||
        !last_invalid_grid_update_stamp_.isValid() ||
        ((grid_now - last_invalid_grid_update_stamp_).toSec() >=
         1.0 / std::max(1e-3, invalid_grid_update_hz_));
    if (do_invalid_grid_update) {
      last_invalid_grid_update_stamp_ = grid_now;
      std::lock_guard<std::mutex> lock(invalid_grid_mutex_);

      const double now_sec = ros::Time::now().toSec();
      InvalidGrid2D& active_grid = global_invalid_grid_;

      // 1. 先清理过期负观测格，保持active只含有效信息
      active_grid.clearExpiredNegObs(now_sec, neg_obs_ttl_sec_);

      // 2. 融合邻机待处理增量（按搜索主循环节拍统一并入）
      for (const auto& kv : pending_invalid_grid_deltas_) {
        const auto& delta = kv.second;
        active_grid.unionWith(delta.frame_grid, delta.recv_stamp_sec);
      }
      pending_invalid_grid_deltas_.clear();

      // 3. 融合本机局部地图障碍增量（全图）
      std::vector<uint8_t> local_occ_2d;
      int local_min_ix = 0, local_max_ix = -1;
      int local_min_iy = 0, local_max_iy = -1;
      {
        std::lock_guard<std::mutex> map_lock(local_map_cache_mutex_);
        local_occ_2d = cached_local_occ_2d_;
        local_min_ix = cached_local_occ_min_ix_;
        local_max_ix = cached_local_occ_max_ix_;
        local_min_iy = cached_local_occ_min_iy_;
        local_max_iy = cached_local_occ_max_iy_;
      }
      if (!local_occ_2d.empty() && local_min_ix <= local_max_ix && local_min_iy <= local_max_iy) {
        const int iy_end = std::min(active_grid.ny - 1, local_max_iy);
        const int ix_end = std::min(active_grid.nx - 1, local_max_ix);
        for (int iy = std::max(0, local_min_iy); iy <= iy_end; ++iy) {
          for (int ix = std::max(0, local_min_ix); ix <= ix_end; ++ix) {
            if (active_grid.get(ix, iy) == 2) continue;
            if (local_occ_2d[iy * grid_nx_ + ix] != 0) {
              active_grid.set(ix, iy, 2, 0.0);
            }
          }
        }
      }

      // 4. 本帧增量：写入临时栅格
      InvalidGrid2D& frame_grid = scratch_frame_grid_;
      frame_grid.origin_x = active_grid.origin_x;
      frame_grid.origin_y = active_grid.origin_y;
      frame_grid.resolution = active_grid.resolution;
      frame_grid.nx = active_grid.nx;
      frame_grid.ny = active_grid.ny;
      if ((int)frame_grid.cells.size() != frame_grid.nx * frame_grid.ny) {
        frame_grid.cells.assign(frame_grid.nx * frame_grid.ny, 0);
        frame_grid.cell_times.assign(frame_grid.nx * frame_grid.ny, 0.0);
        frame_grid.dist_field.clear();
      } else {
        std::fill(frame_grid.cells.begin(), frame_grid.cells.end(), 0);
        std::fill(frame_grid.cell_times.begin(), frame_grid.cell_times.end(), 0.0);
      }
      const double invalid_plane_z = dpfPtr_->pos().z();
      const int marked_cells = search_particles_manager_->markInvalidFromViewFootprint(
          frame_grid, cam_p, cam_q, invalid_plane_z, now_sec);

      // 5. OR 进全局栅格
      active_grid.unionWith(frame_grid);

      // 6. 发布本帧增量给邻居（RLE压缩，通信量小）
      invalid_grid_pub_.publish(toInvalidGridMsg(frame_grid, drone_id_));

      if (search_debug_logs_) {
        ROS_INFO_THROTTLE(1.0,
                          "[dpf%d] Full-grid invalid update: nx=%d ny=%d res=%.2f plane_z=%.2f marked=%d",
                          drone_id_,
                          active_grid.nx, active_grid.ny, active_grid.resolution,
                          invalid_plane_z, marked_cells);
      }
    }

    if (search_debug_logs_) {
      ROS_INFO_THROTTLE(1.0, "[dpf%d] Search: invalid_grid cells marked", drone_id_);
    }

    // 裁剪前快照：用于区分“没跟上”还是“被裁掉”
    if (dbg_visualize_) {
      const ParticleCloudDebugSummary pre_summary =
          summarizeParticleCloud(dpfPtr_->particles_, dpfPtr_->weights_);
      logParticleCloudSummary("pre-prune", pre_summary);
      publishParticleCloud(dpfPtr_->particles_, dpfPtr_->weights_,
                           search_particles_vis_pre_prune_pub_);
    }

    // 搜索期分支健康判定：若多数粒子被强削权，则优先用健康邻机模型接管
    SearchParticlesManager::PruneStats prune_stats;
    {
      std::lock_guard<std::mutex> lock(invalid_grid_mutex_);
      const double now_sec = ros::Time::now().toSec();
      search_particles_manager_->pruneParticlesByInvalidGrid(
          dpfPtr_->particles_, dpfPtr_->weights_, dpfPtr_->N_,
          global_invalid_grid_, now_sec, neg_obs_ttl_sec_,
          invalid_decay_phi_min_, invalid_decay_phi_mid_,
          invalid_decay_r_occ_, invalid_decay_r_safe_,
          &prune_stats, std::max(0.0, std::min(1.0, search_branch_strong_phi_threshold_)));
    }

    // 裁剪后、重采样前快照：观察负观测与距离场是否真的把粒子压下去
    if (dbg_visualize_) {
      const ParticleCloudDebugSummary post_summary =
          summarizeParticleCloud(dpfPtr_->particles_, dpfPtr_->weights_);
      logParticleCloudSummary("post-prune", post_summary);
      publishParticleCloud(dpfPtr_->particles_, dpfPtr_->weights_,
                           search_particles_vis_post_prune_pub_);
    }

    const double neff = computeEffectiveSampleSize(dpfPtr_->weights_);
    const double neff_ratio = (dpfPtr_->N_ > 0) ? neff / static_cast<double>(dpfPtr_->N_) : 0.0;
    const double retained_mass_ratio =
        (prune_stats.weight_sum_before > 1e-300)
            ? (prune_stats.weight_sum_after_raw / prune_stats.weight_sum_before)
            : 0.0;
    const double strong_decay_ratio =
        (prune_stats.total_particles > 0)
            ? (static_cast<double>(prune_stats.strong_decay_particles) /
               static_cast<double>(prune_stats.total_particles))
            : 0.0;
    const bool branch_unhealthy =
        (retained_mass_ratio < search_branch_retained_mass_ratio_) ||
        (strong_decay_ratio > search_branch_strong_decay_ratio_) ||
        (neff_ratio < search_branch_neff_ratio_);

    publishSearchBranchHealthMsg(!branch_unhealthy, retained_mass_ratio,
                                 strong_decay_ratio, neff_ratio, prune_stats.mean_phi);
    if (search_debug_logs_) {
      ROS_INFO_THROTTLE(0.5,
                        "[dpf%d] Search branch health: healthy=%d retained=%.3f strong=%.3f neff_ratio=%.3f mean_phi=%.3f",
                        drone_id_, branch_unhealthy ? 0 : 1,
                        retained_mass_ratio, strong_decay_ratio, neff_ratio, prune_stats.mean_phi);
    }

    bool branch_reseeded_from_neighbor = false;
    if (branch_unhealthy) {
      branch_reseeded_from_neighbor = tryRecoverSearchBranchFromHealthyNeighbor();
      if (!branch_reseeded_from_neighbor) {
        ROS_WARN_THROTTLE(0.5,
                          "[dpf%d] Search branch unhealthy but no healthy neighbor model found; fall back to local ESS resample if needed",
                          drone_id_);
      }
    }

    // 搜索期ESS退化抑制：仅在本机分支仍可用时做本地重采样
    const double neff_thr = std::max(0.0, std::min(1.0, search_ess_resample_ratio_)) * dpfPtr_->N_;
    if (!branch_unhealthy || !branch_reseeded_from_neighbor) {
      if (neff > 0.0 && neff < neff_thr) {
        dpfPtr_->systematicResample();
        applySearchResampleJitter(dpfPtr_->particles_);
        if (search_debug_logs_) {
          ROS_INFO_THROTTLE(0.5,
                            "[dpf%d] Search ESS resample triggered: Neff=%.1f thr=%.1f jitter(pos=%.2f,vel=%.2f)",
                            drone_id_, neff, neff_thr,
                            search_resample_jitter_pos_m_, search_resample_jitter_vel_mps_);
        }
      } else if (search_debug_logs_) {
        ROS_INFO_THROTTLE(1.0,
                          "[dpf%d] Search ESS monitor: Neff=%.1f thr=%.1f (trigger=%d)",
                          drone_id_, neff, neff_thr,
                          (neff > 0.0 && neff < neff_thr) ? 1 : 0);
      }
    } else if (search_debug_logs_) {
      ROS_INFO_THROTTLE(0.5,
                        "[dpf%d] Search branch replaced by healthy neighbor model; skip local ESS resample",
                        drone_id_);
    }

    if (!branch_unhealthy || branch_reseeded_from_neighbor) {
      publishSearchFallbackModelFromCurrentParticles();
    }

    // === 方向承诺式搜索：定期提取热点确定方向，中间沿方向推进 ===
    const ros::Time now = ros::Time::now();
    double time_since_extract = (now - last_hotspot_extract_time_).toSec();
    bool need_extract = !has_committed_direction_ || time_since_extract > current_hotspot_extract_interval_sec_;

    // 收集所有无人机位置
    std::vector<std::pair<int, Eigen::Vector3d>> drone_positions;
    {
      std::lock_guard<std::mutex> lock_self(odom_mutex_);
      drone_positions.push_back({drone_id_, latest_odom_pos_});
    }
    {
      std::lock_guard<std::mutex> lock_neighbor(neighbor_odom_mutex_);
      for (auto& kv : neighbor_positions_) {
        drone_positions.push_back(kv);
      }
    }

    if (need_extract) {
      // 任务重分配前：触发一次短时EM共识会话，但不再阻塞等待——
      // consensus_timer会在后续帧继续增量迭代，本次分配直接使用
      // 当前已收敛的GMM/粒子状态，避免把 0.2~0.5s 的重分配周期
      // 浪费在同步等 EM 上（原 search_em_wait_for_allocation_ 分支已废弃）。
      if (!search_em_session_active_) {
        startSearchEmSession(now, "need_extract");
      }

      // 提取热点（优先基于当前粒子云前沿提取），传入无人机位置做排斥
      std::vector<Eigen::Vector3d> dp_vec;
      for (auto& kv : drone_positions) dp_vec.push_back(kv.second);
      auto hotspots = search_particles_manager_->extractFrontierHotspotsFromParticles(
          dpfPtr_->particles_, dpfPtr_->weights_, dpfPtr_->N_,
          num_drones_, dp_vec, global_invalid_grid_,
          hotspot_min_drone_dist_, hotspot_seed_radius_, hotspot_invalid_reject_ratio_);
      if (hotspots.empty()) {
        hotspots = search_particles_manager_->extractHotspots(num_drones_, dp_vec);
        ROS_WARN_THROTTLE(1.0, "[dpf%d] Frontier hotspots empty, fallback to GMM hotspots",
                          drone_id_);
      }

      if (!hotspots.empty()) {
        hotspots = spreadSearchHotspots(hotspots, num_drones_, search_hotspot_min_separation_m_);
      }

      if (!hotspots.empty()) {
        const std::vector<double> w_norm = normalizeParticleWeights(dpfPtr_->weights_, dpfPtr_->N_);
        const auto view_candidates = buildSearchViewCandidates(
            hotspots, drone_positions, dpfPtr_->particles_, dpfPtr_->weights_, dpfPtr_->N_,
            global_invalid_grid_);

        bool used_view_assignment = false;
        if (!view_candidates.empty()) {
          const auto view_assignments = assignSearchViewsForCoverage(drone_positions, view_candidates, w_norm);
          if (!view_assignments.empty()) {
            used_view_assignment = true;

            if (search_debug_logs_) {
              for (const auto& a : view_assignments) {
                if (a.candidate_index < 0 || a.candidate_index >= static_cast<int>(view_candidates.size())) continue;
                const auto& c = view_candidates[a.candidate_index];
                ROS_INFO("[dpf%d] Search view assign: drone=%d cand=%d hotspot=%d pos=(%.2f,%.2f,%.2f) yaw=%.1fdeg vis=%.3f gain=%.3f overlap=%.3f pos_pen=%.3f same_hotspot_pen=%.1f dist=%.2f util=%.3f",
                         drone_id_, a.drone_id, a.candidate_index, a.hotspot_index,
                         c.pos.x(), c.pos.y(), c.pos.z(), c.yaw * 180.0 / M_PI,
                         a.visible_mass, a.gain_mass, a.overlap_mass,
                         a.position_penalty, a.same_hotspot_penalty,
                         a.distance, a.utility);
              }
            }

            for (const auto& a : view_assignments) {
              if (a.drone_id != drone_id_ || a.candidate_index < 0 ||
                  a.candidate_index >= static_cast<int>(view_candidates.size())) {
                continue;
              }
              Eigen::Vector3d my_pos = Eigen::Vector3d::Zero();
              for (const auto& dp : drone_positions) {
                if (dp.first == drone_id_) {
                  my_pos = dp.second;
                  break;
                }
              }

              const auto& c = view_candidates[a.candidate_index];
              Eigen::Vector3d dir = c.pos - my_pos;
              dir.z() = 0.0;
              if (dir.norm() > 0.5) {
                committed_search_dir_ = dir.normalized();
              }
              committed_target_pos_ = c.pos;
              has_committed_direction_ = true;
              last_hotspot_extract_time_ = now;
              committed_direction_refresh_count_++;
              const double planar_dist = std::max(0.0, dir.head<2>().norm());
              const double vmax = std::max(0.1, search_advance_vmax_);
              const double eta_to_reach_sec = planar_dist / vmax;
              current_hotspot_extract_interval_sec_ =
                  std::max(0.2, std::min(hotspot_extract_interval_base_sec_, 0.8 * eta_to_reach_sec));
              ROS_WARN("[dpf%d] Search view committed: target=(%.2f,%.2f,%.2f) yaw=%.1fdeg vis=%.3f gain=%.3f overlap=%.3f pos_pen=%.3f same_hotspot_pen=%.1f dist=%.2f eta=%.2fs hold=%.1fs refresh=%d",
                       drone_id_, c.pos.x(), c.pos.y(), c.pos.z(),
                       c.yaw * 180.0 / M_PI, a.visible_mass, a.gain_mass,
                       a.overlap_mass, a.position_penalty, a.same_hotspot_penalty,
                       planar_dist, eta_to_reach_sec,
                       current_hotspot_extract_interval_sec_, committed_direction_refresh_count_);
              break;
            }

            geometry_msgs::PoseArray hotspot_msg;
            hotspot_msg.header.stamp = ros::Time::now();
            hotspot_msg.header.frame_id = "world";
            hotspot_msg.poses.resize(num_drones_);
            for (const auto& a : view_assignments) {
              if (a.drone_id < 0 || a.drone_id >= num_drones_) continue;
              const auto& c = view_candidates[a.candidate_index];
              geometry_msgs::Pose p;
              p.position.x = c.pos.x();
              p.position.y = c.pos.y();
              p.position.z = c.pos.z();
              const Eigen::Quaterniond q(Eigen::AngleAxisd(c.yaw, Eigen::Vector3d::UnitZ()));
              p.orientation.w = q.w();
              p.orientation.x = q.x();
              p.orientation.y = q.y();
              p.orientation.z = q.z();
              hotspot_msg.poses[a.drone_id] = p;
            }
            search_targets_pub_.publish(hotspot_msg);
          }
        }

        if (!used_view_assignment) {
          // 回退：仍按热点覆盖分配，确保新策略在候选视位不足时不会中断搜索。
          const auto assignments = assignSearchHotspotsForCoverage(drone_positions, hotspots);

          if (search_debug_logs_) {
            for (const auto& a : assignments) {
              if (a.hotspot_index < 0 || a.hotspot_index >= static_cast<int>(hotspots.size())) continue;
              const auto& h = hotspots[a.hotspot_index];
              ROS_INFO("[dpf%d] Search assign: drone=%d hotspot=%d pos=(%.2f,%.2f,%.2f) w=%.3f utility=%.3f dist=%.2f overlap=%.3f reuse=%.3f",
                       drone_id_, a.drone_id, a.hotspot_index,
                       h.pos.x(), h.pos.y(), h.pos.z(), h.weight,
                       a.utility, a.distance, a.overlap_penalty, a.reuse_penalty);
            }
          }

          for (const auto& a : assignments) {
            if (a.drone_id != drone_id_ || a.hotspot_index < 0 ||
                a.hotspot_index >= static_cast<int>(hotspots.size())) {
              continue;
            }
            Eigen::Vector3d my_pos = Eigen::Vector3d::Zero();
            for (const auto& dp : drone_positions) {
              if (dp.first == drone_id_) {
                my_pos = dp.second;
                break;
              }
            }

            const Eigen::Vector3d hotspot_pos = hotspots[a.hotspot_index].pos;
            const double hotspot_yaw = hotspots[a.hotspot_index].yaw;
            Eigen::Vector3d dir = hotspot_pos - my_pos;
            dir.z() = 0.0;
            if (dir.norm() > 0.5) {
              committed_search_dir_ = dir.normalized();
            }
            committed_target_pos_ = hotspot_pos;
            has_committed_direction_ = true;
            last_hotspot_extract_time_ = now;
            committed_direction_refresh_count_++;
            const double planar_dist = std::max(0.0, dir.head<2>().norm());
            const double vmax = std::max(0.1, search_advance_vmax_);
            const double eta_to_reach_sec = planar_dist / vmax;
            current_hotspot_extract_interval_sec_ =
                std::max(0.2, std::min(hotspot_extract_interval_base_sec_, 0.8 * eta_to_reach_sec));
            ROS_WARN("[dpf%d] Search task committed: target=(%.2f,%.2f,%.2f) yaw=%.1fdeg dir=(%.2f,%.2f) w=%.3f dist=%.2f eta=%.2fs hold=%.1fs refresh=%d",
                     drone_id_, hotspot_pos.x(), hotspot_pos.y(), hotspot_pos.z(),
                     hotspot_yaw * 180.0 / M_PI, committed_search_dir_.x(), committed_search_dir_.y(),
                     hotspots[a.hotspot_index].weight,
                     planar_dist, eta_to_reach_sec,
                     current_hotspot_extract_interval_sec_, committed_direction_refresh_count_);
            break;
          }

          geometry_msgs::PoseArray hotspot_msg;
          hotspot_msg.header.stamp = ros::Time::now();
          hotspot_msg.header.frame_id = "world";
          hotspot_msg.poses.resize(num_drones_);
          for (const auto& a : assignments) {
            if (a.drone_id < 0 || a.drone_id >= num_drones_) continue;
            const auto& h = hotspots[a.hotspot_index];
            geometry_msgs::Pose p;
            p.position.x = h.pos.x();
            p.position.y = h.pos.y();
            p.position.z = h.pos.z();
            const Eigen::Quaterniond q(Eigen::AngleAxisd(h.yaw, Eigen::Vector3d::UnitZ()));
            p.orientation.w = q.w();
            p.orientation.x = q.x();
            p.orientation.y = q.y();
            p.orientation.z = q.z();
            hotspot_msg.poses[a.drone_id] = p;
          }
          search_targets_pub_.publish(hotspot_msg);
        }
      }
    } else if (has_committed_direction_ && !search_hold_assigned_target_) {
      // 兼容旧行为：只有显式关闭“持有任务点”时，才继续沿方向推进虚拟目标。
      double vmax = search_advance_vmax_;
      committed_target_pos_ += committed_search_dir_ * vmax * 0.2; // 5Hz, dt=0.2s
    }

    // 发布搜索目标
    if (has_committed_direction_) {
      double vmax = search_advance_vmax_;
      nav_msgs::Odometry target_odom;
      target_odom.header.stamp = ros::Time::now();
      target_odom.header.frame_id = "world";
      target_odom.pose.pose.position.x = committed_target_pos_.x();
      target_odom.pose.pose.position.y = committed_target_pos_.y();
      target_odom.pose.pose.position.z = committed_target_pos_.z();
      const Eigen::Quaterniond target_q(Eigen::AngleAxisd(
          std::atan2(committed_search_dir_.y(), committed_search_dir_.x()),
          Eigen::Vector3d::UnitZ()));
      target_odom.pose.pose.orientation.w = target_q.w();
      target_odom.pose.pose.orientation.x = target_q.x();
      target_odom.pose.pose.orientation.y = target_q.y();
      target_odom.pose.pose.orientation.z = target_q.z();
      target_odom.twist.twist.linear.x = committed_search_dir_.x() * vmax;
      target_odom.twist.twist.linear.y = committed_search_dir_.y() * vmax;
      target_odom.twist.twist.linear.z = 0;
      target_odom_pub_.publish(target_odom);

      if (search_debug_logs_) {
        ROS_INFO_THROTTLE(1.0, "[dpf%d] Search advancing: pos=(%.2f,%.2f,%.2f) dir=(%.2f,%.2f) t=%.1f/%.1fs",
            drone_id_, committed_target_pos_.x(), committed_target_pos_.y(), committed_target_pos_.z(),
            committed_search_dir_.x(), committed_search_dir_.y(),
            time_since_extract, current_hotspot_extract_interval_sec_);
      }
    }

    // 发布搜索GMM（用于bag录制和可视化）
    {
      int C = search_particles_manager_->getSearchC();
      const auto& za = search_particles_manager_->getSearchZetaAlpha();
      const auto& zav = search_particles_manager_->getSearchZetaA();
      const auto& zbv = search_particles_manager_->getSearchZetaB();
      if (C > 0 && (int)za.size() == C) {
        target_ekf::LocalStats gmm_msg;
        gmm_msg.header.stamp = ros::Time::now();
        gmm_msg.header.frame_id = "world";
        gmm_msg.drone_id = drone_id_;
        gmm_msg.num_components = C;
        gmm_msg.state_dim = 6;
        gmm_msg.obs_dim = 0;
        gmm_msg.has_observation = false;
        // 用 zeta_alpha/a/b 直接作为 GMM 参数发布（M步后 mu_c = zeta_a[c]/zeta_alpha[c]）
        gmm_msg.zeta_alpha.resize(C);
        gmm_msg.zeta_a.resize(C * 6);
        gmm_msg.zeta_b.resize(C * 36);
        for (int c = 0; c < C; ++c) {
          gmm_msg.zeta_alpha[c] = za(c);
          if (c < (int)zav.size() && zav[c].size() == 6) {
            for (int i = 0; i < 6; ++i) gmm_msg.zeta_a[c * 6 + i] = zav[c](i);
          }
          if (c < (int)zbv.size() && zbv[c].rows() == 6) {
            for (int i = 0; i < 6; ++i)
              for (int j = 0; j < 6; ++j)
                gmm_msg.zeta_b[c * 36 + i * 6 + j] = zbv[c](i, j);
          }
        }
        search_pos_gmm_pub_.publish(gmm_msg);
      }
    }

    // 发布搜索状态
    std_msgs::Bool search_state_msg;
    search_state_msg.data = search_mode_active_;
    search_state_pub_.publish(search_state_msg);

    // 发布真实粒子云（搜索模式，仅在可视化调试模式下）
    if (dbg_visualize_) publishSearchParticlesCloudFromDPF();

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
    
    // 失锁对比模式：search_mode_active_时继续普通DPF估计，直到重观测后退出
    if (search_mode_active_ && !useSearchGmm6DMode()) {
      if (has_obs) {
        search_mode_active_ = false;
        consecutive_no_obs_count_ = 0;
        search_mode_frame_count_ = 0;
        rollback_loss_ref_valid_ = false;
        {
          std::lock_guard<std::mutex> lock(invalid_grid_mutex_);
            pending_invalid_grid_deltas_.clear();
        }
        resetCommittedDirectionState();
        activateReacquireBoost("self_reacquired_legacy");
        clearSearchEmSession("legacy_self_reacquired");
        clearFrozenSnapshotRing("legacy_self_reacquired");
        cacheReliableSnapshotFromDPF("legacy_self_reacquired");
        ROS_WARN("[dpf%d] EXIT LEGACY SEARCH MODE: self reacquired.", drone_id_);
      } else {
        for (const auto& nc : neighbor_consensus) {
          if (nc.has_obs) {
            search_mode_active_ = false;
            consecutive_no_obs_count_ = 0;
            search_mode_frame_count_ = 0;
            rollback_loss_ref_valid_ = false;
            {
              std::lock_guard<std::mutex> lock(invalid_grid_mutex_);
              pending_invalid_grid_deltas_.clear();
            }
            resetCommittedDirectionState();
            activateReacquireBoost("neighbor_reacquired_legacy");
            clearSearchEmSession("legacy_neighbor_reacquired");
            clearFrozenSnapshotRing("legacy_neighbor_reacquired");
            ROS_WARN("[dpf%d] EXIT LEGACY SEARCH MODE: neighbor reacquired.", drone_id_);
            break;
          }
        }
      }
    }

    // 集体有观测时持续刷新可靠快照（不要求本机必须直接看到）
    // 这样 freeze(0->1) 时拿到的是“集体失锁前最近帧”而非“本机最后直观测帧”。
    if (!all_drones_no_obs && !search_mode_active_) {
      cacheReliableSnapshotFromDPF(has_obs ? "obs_frame" : "collective_obs_frame");
    }

    // 更新连续无观测计数与进入失锁模式
    const int enter_search_threshold = getCurrentMissDetectionThreshold();
    if (all_drones_no_obs) {
      if (!search_mode_active_) {
        consecutive_no_obs_count_++;
        if (consecutive_no_obs_count_ == 1) {
          freezeReliableSnapshotRing("loss_start");
        }
        if (rollback_validation_enable_ && consecutive_no_obs_count_ == 1) {
          std::lock_guard<std::mutex> obs_lock(obs_mutex_);
          rollback_loss_ref_pos_ = latest_obs_pos_;
          rollback_loss_ref_vel_ = latest_obs_vel_;
          rollback_loss_ref_stamp_ = latest_obs_stamp_;
          rollback_loss_ref_valid_ = true;
          ROS_WARN("[dpf%d][search-init-ref] loss_ref captured: t=%.3f pos=(%.2f,%.2f,%.2f) vel=(%.2f,%.2f,%.2f)",
                   drone_id_, rollback_loss_ref_stamp_.toSec(),
                   rollback_loss_ref_pos_.x(), rollback_loss_ref_pos_.y(), rollback_loss_ref_pos_.z(),
                   rollback_loss_ref_vel_.x(), rollback_loss_ref_vel_.y(), rollback_loss_ref_vel_.z());
        }
      }
      if (consecutive_no_obs_count_ >= enter_search_threshold && !search_mode_active_) {
        // 立即进入失锁模式
        search_mode_active_ = true;
        search_mode_frame_count_ = 0;
        {
          std::lock_guard<std::mutex> lock(invalid_grid_mutex_);
          pending_invalid_grid_deltas_.clear();
        }
        reacquire_boost_active_ = false;
        last_global_obs_time_ = ros::Time::now();
        resetCommittedDirectionState();
        ROS_WARN("[dpf%d] ENTERING SEARCH MODE: All drones lost target for %d consecutive frames (threshold=%d)!",
                 drone_id_, consecutive_no_obs_count_, enter_search_threshold);

        if (useSearchGmm6DMode()) {
          // 新方法：使用失锁开始前冻结的可靠粒子快照初始化；
          // 并用搜索动力学步进h帧到入搜当前时刻（不做无效区裁剪/权重衰减）
          const int h_frames = std::max(0, consecutive_no_obs_count_);
          const auto* frozen = getFrozenSnapshot();
          if (!(loss_snapshot_frozen_ && frozen &&
                frozen->particles.rows() == dpfPtr_->particles_.rows() &&
                frozen->particles.cols() == dpfPtr_->particles_.cols() &&
                frozen->weights.size() == dpfPtr_->weights_.size())) {
            ROS_ERROR("[dpf%d] Frozen reliable snapshot unavailable; abort entering search this frame.",
                      drone_id_);
            search_mode_active_ = false;
            std_msgs::Bool search_state_msg;
            search_state_msg.data = false;
            search_state_pub_.publish(search_state_msg);
            last_update_stamp_ = ros::Time::now();
            return;
          }
          Eigen::MatrixXd init_particles = frozen->particles;
          Eigen::VectorXd init_weights = frozen->weights;

          // 用最后一次可靠观测对冻结快照做一次锚定：
          // frozen ring 里保存的是DPF后验，不一定等于“失锁时真实目标位置”。
          // 如果目标发生急停/急转，这里把粒子云整体平移到 loss_ref 附近，
          // 至少保证搜索初态不会长期停留在明显滞后的估计上。
          if (rollback_validation_enable_ && rollback_loss_ref_valid_) {
            const auto frozen_summary = computeParticleKinematicSummary(init_particles, init_weights);
            const Eigen::Vector3d pos_delta = rollback_loss_ref_pos_ - frozen_summary.pos_mean;
            const Eigen::Vector3d vel_delta = rollback_loss_ref_vel_ - frozen_summary.vel_mean;
            const double pos_err = pos_delta.norm();
            const double vel_err = vel_delta.norm();
            if (pos_err > 0.25 || vel_err > 0.25) {
              init_particles.topRows(3).colwise() += pos_delta;
              if (init_particles.rows() >= 6) {
                init_particles.block(3, 0, 3, init_particles.cols()).colwise() += vel_delta;
              }
              ROS_WARN("[dpf%d][search-init-align] anchor frozen snapshot to loss_ref: pos_err=%.3f vel_err=%.3f",
                       drone_id_, pos_err, vel_err);
            }
          }

          const int spread_frames = std::max(0, search_spread_init_frames_);
          const int replay_spread_steps = std::min(h_frames, spread_frames);
          const int replay_mixed_steps = std::max(0, h_frames - replay_spread_steps);
          for (int step = 0; step < replay_spread_steps; ++step) {
            search_particles_manager_->searchParticlesWideSpreadUpdate(
                init_particles, dpfPtr_->N_,
                step, spread_frames,
                search_spread_speed_min_ratio_, search_spread_theta_jitter_deg_);
          }
          for (int step = 0; step < replay_mixed_steps; ++step) {
            search_particles_manager_->searchParticlesMixedUpdate(
                init_particles, dpfPtr_->N_,
                step,
                search_mixed_turn_hold_steps_,
                search_mixed_turn_ratio_,
                search_mixed_turn_theta_sigma_deg_,
                nullptr);
          }

          if (rollback_validation_enable_ && rollback_loss_ref_valid_) {
            const auto init_summary = computeParticleKinematicSummary(init_particles, init_weights);
            const double init_pos_err = (init_summary.pos_mean - rollback_loss_ref_pos_).norm();
            const double init_vel_err = (init_summary.vel_mean - rollback_loss_ref_vel_).norm();
            ROS_WARN("[dpf%d][search-init-val] h=%d dt=%.3f | replay(reliable-snapshot) pos_err=%.3f vel_err=%.3f",
                     drone_id_, h_frames, dpfPtr_->dt_, init_pos_err, init_vel_err);
          }
          dpfPtr_->particles_ = init_particles;
          dpfPtr_->weights_ = init_weights;
          search_mode_frame_count_ = h_frames;
          const size_t frozen_count = frozen_loss_snapshot_ring_.size();
          clearFrozenSnapshotRing("search_init_consumed");
          ROS_WARN("[dpf%d] Search init replay done: source=frozen-reliable-ring, frozen=%zu, h=%d, spread_replay=%d, mixed_replay=%d (dt=%.3f, spread_total=%d)",
                   drone_id_, frozen_count, h_frames, replay_spread_steps, replay_mixed_steps, dpfPtr_->dt_, spread_frames);
          search_particles_manager_->initSearchGMM6D(
              dpfPtr_->particles_, dpfPtr_->weights_, dpfPtr_->N_, 4 * num_drones_);

          // 发布搜索状态并跳过本帧其余普通DPF流程
          std_msgs::Bool search_state_msg;
          search_state_msg.data = search_mode_active_;
          search_state_pub_.publish(search_state_msg);
          return;
        } else {
          // 对比模式：仅置位失锁状态，估计继续沿用普通DPF链路
          ROS_WARN("[dpf%d] LEGACY SEARCH ESTIMATION MODE: keep original DPF state estimation during loss.",
                   drone_id_);
        }
      }
    } else {
      // 有任意无人机重新观测到目标（且未处于失锁模式）时，清零计数
      if (!search_mode_active_) {
        if (consecutive_no_obs_count_ > 0 || loss_snapshot_frozen_) {
          clearFrozenSnapshotRing("loss_cancelled_before_threshold");
        }
        consecutive_no_obs_count_ = 0;
        search_mode_frame_count_ = 0;
        rollback_loss_ref_valid_ = false;
      }
    }

    // --- EM迭代由 consensus_timer_callback 高频执行 ---

    // --- 数值有效性检查 ---
    if (!dpfPtr_->isValid()) {
      ROS_ERROR("[dpf%d] update invalid! NaN/Inf detected, resetting.", drone_id_);
      if (has_obs) dpfPtr_->reset(obs_pos, obs_rpy);
      dpf_reset_suppress_count_ = 3;
      return;
    }

    // 对比模式：失锁期间把普通DPF分布投影并发布为search_pos_gmm，便于统一评估ES曲线
    if (search_mode_active_ && !useSearchGmm6DMode()) {
      publishSearchPosGMMFromLegacyDPF();
      if (dbg_visualize_) publishSearchParticlesCloudFromDPF();
    }
    
    // 发布搜索状态
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
  if (search_debug_logs_) {
    ROS_INFO_THROTTLE(0.5, "[dpf%d][est] pos=(%.2f,%.2f,%.2f) vel=(%.2f,%.2f,%.2f) | GMM0=(%.2f,%.2f,%.2f) | obs_err=%.3fm | has_obs=%d",
      drone_id_,
      est_pos.x(), est_pos.y(), est_pos.z(),
      est_vel.x(), est_vel.y(), est_vel.z(),
      gmm_mu0(0), gmm_mu0(1), gmm_mu0(2),
      pos_err, has_obs);
  }
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

  if (search_debug_logs_) {
    ROS_INFO("[dpf%d] Task assignment: %d drones, %d clusters", drone_id_, num_drones, num_clusters);
  }

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
      if (search_debug_logs_) {
        ROS_INFO("  Drone %d -> Cluster %d (GMM_ID=%d, weight=%.3f, dist=%.2f)",
                 d, cluster_idx, gmm_ids[cluster_idx], gmm_pi(cluster_idx), dist);
      }
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
        if (search_debug_logs_) {
          ROS_INFO("  Drone %d -> Cluster %d (GMM_ID=%d, weight=%.3f, dist=%.2f)",
                   d, best_c, gmm_ids[best_c], gmm_pi(best_c), best_dist);
        }
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
        if (search_debug_logs_) {
          ROS_INFO("  Drone %d -> Cluster %d (GMM_ID=%d, weight=%.3f, dist=%.2f)",
                   best_d, c, gmm_ids[c], gmm_pi(c), best_dist);
        }
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
        if (search_debug_logs_) {
          ROS_INFO("  Drone %d -> Cluster %d (GMM_ID=%d, weight=%.3f, dist=%.2f) [extra]",
                   d, max_weight_cluster, gmm_ids[max_weight_cluster], gmm_pi(max_weight_cluster), dist);
        }
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
  nh.getParam("max_obs_depth", max_obs_depth_);
  // 搜索模式参数
  nh.getParam("miss_detection_num", miss_detection_num_);
  std::string search_estimation_mode_str = "search_gmm6d";
  nh.param<std::string>("search_estimation_mode", search_estimation_mode_str, search_estimation_mode_str);
  std::transform(search_estimation_mode_str.begin(), search_estimation_mode_str.end(),
                 search_estimation_mode_str.begin(),
                 [](unsigned char ch) { return static_cast<char>(std::tolower(ch)); });
  if (search_estimation_mode_str == "legacy_dpf") {
    search_estimation_mode_ = SearchEstimationMode::kLegacyDpf;
  } else {
    search_estimation_mode_ = SearchEstimationMode::kSearchGmm6D;
    if (search_estimation_mode_str != "search_gmm6d") {
      ROS_WARN("[dpf%d] Unknown search_estimation_mode='%s', fallback to 'search_gmm6d'",
               drone_id_, search_estimation_mode_str.c_str());
    }
  }
  nh.param("reacquire_boost_duration_sec", reacquire_boost_duration_sec_, 3.0);
  nh.param("reacquire_boost_miss_scale", reacquire_boost_miss_scale_, 3.0);
  nh.param("reacquire_boost_miss_min", reacquire_boost_miss_min_, 0);
  reacquire_boost_duration_sec_ = std::max(0.0, reacquire_boost_duration_sec_);
  reacquire_boost_miss_scale_ = std::max(1.0, reacquire_boost_miss_scale_);
  nh.param("search_spread_init_frames", search_spread_init_frames_, 8);
  nh.param("reliable_snapshot_history_size", reliable_snapshot_history_size_, 12);
  nh.param("search_spread_speed_min_ratio", search_spread_speed_min_ratio_, 0.8);
  nh.param("search_spread_theta_jitter_deg", search_spread_theta_jitter_deg_, 10.0);
  nh.param("search_aggressive_ratio", search_mixed_turn_ratio_, 0.50);
  nh.param("search_aggressive_theta_sigma_deg", search_mixed_turn_theta_sigma_deg_, 90.0);
  nh.param("search_mixed_turn_hold_steps", search_mixed_turn_hold_steps_, 5);
  nh.param("search_ess_resample_ratio", search_ess_resample_ratio_, 1.00);
  nh.param("search_resample_jitter_pos_m", search_resample_jitter_pos_m_, 0.15);
  nh.param("search_resample_jitter_vel_mps", search_resample_jitter_vel_mps_, 0.20);
  nh.param("search_branch_retained_mass_ratio", search_branch_retained_mass_ratio_, 0.35);
  nh.param("search_branch_strong_decay_ratio", search_branch_strong_decay_ratio_, 0.55);
  nh.param("search_branch_strong_phi_threshold", search_branch_strong_phi_threshold_, 0.35);
  nh.param("search_branch_neff_ratio", search_branch_neff_ratio_, 0.25);
  nh.param("search_branch_model_age_sec", search_branch_model_age_sec_, 0.60);
  nh.param("search_em_session_iters", search_em_session_target_iters_, 12);
  nh.param("search_em_session_timeout_sec", search_em_session_timeout_sec_, 0.25);
  nh.param("search_em_wait_for_allocation", search_em_wait_for_allocation_, true);
  search_spread_init_frames_ = std::max(0, search_spread_init_frames_);
  reliable_snapshot_history_size_ = std::max(1, reliable_snapshot_history_size_);
  search_spread_speed_min_ratio_ = std::max(0.0, std::min(1.0, search_spread_speed_min_ratio_));
  search_spread_theta_jitter_deg_ = std::max(0.0, search_spread_theta_jitter_deg_);
  search_mixed_turn_ratio_ = std::max(0.0, std::min(1.0, search_mixed_turn_ratio_));
  search_mixed_turn_theta_sigma_deg_ = std::max(0.0, search_mixed_turn_theta_sigma_deg_);
  search_mixed_turn_hold_steps_ = std::max(1, search_mixed_turn_hold_steps_);
  search_ess_resample_ratio_ = std::max(0.0, std::min(1.0, search_ess_resample_ratio_));
  search_resample_jitter_pos_m_ = std::max(0.0, search_resample_jitter_pos_m_);
  search_resample_jitter_vel_mps_ = std::max(0.0, search_resample_jitter_vel_mps_);
  search_branch_retained_mass_ratio_ = std::max(0.0, std::min(1.0, search_branch_retained_mass_ratio_));
  search_branch_strong_decay_ratio_ = std::max(0.0, std::min(1.0, search_branch_strong_decay_ratio_));
  search_branch_strong_phi_threshold_ = std::max(0.0, std::min(1.0, search_branch_strong_phi_threshold_));
  search_branch_neff_ratio_ = std::max(0.0, std::min(1.0, search_branch_neff_ratio_));
  search_branch_model_age_sec_ = std::max(0.1, search_branch_model_age_sec_);
  search_em_session_target_iters_ = std::max(1, search_em_session_target_iters_);
  search_em_session_timeout_sec_ = std::max(0.05, search_em_session_timeout_sec_);
  nh.param("neg_obs_ttl_sec", neg_obs_ttl_sec_, 0.5);
  nh.param("dbg_visualize", dbg_visualize_, false);
  nh.param("search_debug_logs", search_debug_logs_, false);
  nh.param("invalid_grid_update_hz", invalid_grid_update_hz_, 5.0);
  nh.param("rollback_validation_enable", rollback_validation_enable_, true);
  nh.param("invalid_decay_phi_min", invalid_decay_phi_min_, 0.08);
  nh.param("invalid_decay_phi_mid", invalid_decay_phi_mid_, 0.60);
  nh.param("invalid_decay_r_occ", invalid_decay_r_occ_, -0.50);
  nh.param("invalid_decay_r_safe", invalid_decay_r_safe_, 1.00);
  invalid_decay_phi_min_ = std::max(0.0, std::min(1.0, invalid_decay_phi_min_));
  invalid_decay_phi_mid_ = std::max(invalid_decay_phi_min_, std::min(1.0, invalid_decay_phi_mid_));
  if (!(invalid_decay_r_occ_ < 0.0)) invalid_decay_r_occ_ = -0.50;
  if (!(invalid_decay_r_safe_ > 0.0)) invalid_decay_r_safe_ = 1.00;
  nh.param("hotspot_min_drone_dist", hotspot_min_drone_dist_, 2.0);
  nh.param("hotspot_seed_radius", hotspot_seed_radius_, 1.5);
  nh.param("hotspot_invalid_reject_ratio", hotspot_invalid_reject_ratio_, 1.0);
  nh.param("search_hotspot_min_separation", search_hotspot_min_separation_m_, 4.0);
  nh.param("search_view_standoff_radius", search_view_standoff_radius_m_, 4.0);
  nh.param("search_view_candidate_azimuths", search_view_candidate_azimuths_, 8);
  nh.param("search_view_gain_scale", search_view_gain_scale_, 100.0);
  nh.param("search_view_overlap_penalty", search_view_overlap_penalty_, 140.0);
  nh.param("search_view_position_penalty_radius", search_view_position_penalty_radius_m_, 6.0);
  nh.param("search_view_position_penalty", search_view_position_penalty_, 80.0);
  nh.param("search_view_same_hotspot_penalty", search_view_same_hotspot_penalty_, 120.0);
  nh.param("search_view_travel_cost_scale", search_view_travel_cost_scale_, 1.0);
  nh.param("search_view_min_visible_mass", search_view_min_visible_mass_, 0.01);
  if (search_view_candidate_azimuths_ < 1) search_view_candidate_azimuths_ = 1;
  if (search_hotspot_min_separation_m_ < 0.0) search_hotspot_min_separation_m_ = 0.0;
  if (search_view_standoff_radius_m_ < 0.5) search_view_standoff_radius_m_ = 0.5;
  if (search_view_position_penalty_radius_m_ < 0.5) search_view_position_penalty_radius_m_ = 0.5;
  if (search_view_min_visible_mass_ < 0.0) search_view_min_visible_mass_ = 0.0;
  if (!nh.getParam("hotspot_extract_interval_sec", hotspot_extract_interval_base_sec_)) {
    // 兼容旧参数名
    nh.param("hotspot_extract_interval", hotspot_extract_interval_base_sec_, hotspot_extract_interval_base_sec_);
  }
  if (hotspot_extract_interval_base_sec_ < 0.2) hotspot_extract_interval_base_sec_ = 0.2;
  hotspot_extract_interval_growth_sec_ = 0.0;
  hotspot_extract_interval_max_sec_ = hotspot_extract_interval_base_sec_;
  current_hotspot_extract_interval_sec_ = hotspot_extract_interval_base_sec_;

  ROS_INFO("[dpf%d] Search frontier params: neg_obs_ttl=%.1fs, phi_min=%.2f, phi_mid=%.2f, r_occ=%.2fm, r_safe=%.2fm, min_drone_dist=%.2fm, seed_radius=%.2fm, invalid_reject_ratio=%.2f",
           drone_id_, neg_obs_ttl_sec_, invalid_decay_phi_min_, invalid_decay_phi_mid_,
           invalid_decay_r_occ_, invalid_decay_r_safe_,
           hotspot_min_drone_dist_, hotspot_seed_radius_, hotspot_invalid_reject_ratio_);
  ROS_INFO("[dpf%d] rollback_validation_enable=%d", drone_id_, rollback_validation_enable_);
  ROS_INFO("[dpf%d] Reacquire boost: hold=%.2fs, miss_scale=%.2f, miss_min=%d, base_miss=%d",
           drone_id_, reacquire_boost_duration_sec_, reacquire_boost_miss_scale_,
           reacquire_boost_miss_min_, miss_detection_num_);
  ROS_INFO("[dpf%d] Search commitment interval: fixed=%.2fs (growth disabled)",
           drone_id_, hotspot_extract_interval_base_sec_);
  ROS_INFO("[dpf%d] Search view params: hotspot_sep=%.2fm standoff=%.2fm azimuths=%d gain_scale=%.1f overlap_penalty=%.1f pos_penalty_r=%.2fm pos_penalty=%.1f same_hotspot_penalty=%.1f travel_cost=%.2f min_visible_mass=%.3f",
           drone_id_, search_hotspot_min_separation_m_,
           search_view_standoff_radius_m_, search_view_candidate_azimuths_,
           search_view_gain_scale_, search_view_overlap_penalty_,
           search_view_position_penalty_radius_m_, search_view_position_penalty_,
           search_view_same_hotspot_penalty_,
           search_view_travel_cost_scale_, search_view_min_visible_mass_);
  ROS_INFO("[dpf%d] Search spread init: frames=%d, speed_min_ratio=%.2f, theta_jitter=%.1fdeg",
           drone_id_, search_spread_init_frames_, search_spread_speed_min_ratio_,
           search_spread_theta_jitter_deg_);
  ROS_INFO("[dpf%d] Reliable snapshot ring: history_size=%d",
           drone_id_, reliable_snapshot_history_size_);
  ROS_INFO("[dpf%d] Search mixed update: turn_ratio=%.2f, theta_sigma=%.1fdeg, hold_steps=%d",
           drone_id_, search_mixed_turn_ratio_, search_mixed_turn_theta_sigma_deg_,
           search_mixed_turn_hold_steps_);
  ROS_INFO("[dpf%d] Search ESS resample: ratio=%.2f, jitter_pos=%.2fm, jitter_vel=%.2fm/s",
           drone_id_, search_ess_resample_ratio_,
           search_resample_jitter_pos_m_, search_resample_jitter_vel_mps_);
  ROS_INFO("[dpf%d] Search EM session: iters=%d timeout=%.3fs wait_for_allocation=%d",
           drone_id_, search_em_session_target_iters_,
           search_em_session_timeout_sec_, search_em_wait_for_allocation_ ? 1 : 0);
  ROS_WARN("[dpf%d] search_estimation_mode=%s",
           drone_id_, useSearchGmm6DMode() ? "search_gmm6d" : "legacy_dpf");
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
  search_particles_manager_->setSearchDt(dpfPtr_->dt_);
  search_particles_manager_->setCameraParams(
      fx_, fy_, cx_, cy_, width_, height_, max_obs_depth_);
  ROS_INFO("[dpf%d] Align search_dt to dpf dt: %.3fs (%.1fHz)",
           drone_id_, dpfPtr_->dt_, 1.0 / std::max(1e-6, dpfPtr_->dt_));
  ROS_INFO("[dpf%d] Align invalid camera params to camera.yaml: fx=%.2f fy=%.2f cx=%.2f cy=%.2f w=%.0f h=%.0f max_obs_depth=%.2f",
           drone_id_, fx_, fy_, cx_, cy_, width_, height_, max_obs_depth_);

  // 初始化无效区域虚拟栅格
  nh.param("grid_resolution", grid_resolution_, 0.15);
  nh.param("grid_map_size_x", grid_map_size_x_, -1.0);
  nh.param("grid_map_size_y", grid_map_size_y_, -1.0);
  nh.param("grid_origin_x", grid_origin_x_, -10.0);
  nh.param("grid_origin_y", grid_origin_y_, -15.0);
  nh.param("grid_nx", grid_nx_, 80);
  nh.param("grid_ny", grid_ny_, 80);
  if (grid_map_size_x_ > 0.0 && grid_map_size_y_ > 0.0 && grid_resolution_ > 1e-6) {
    // Full-map mode: cover the known map extents with a single global grid.
    grid_origin_x_ = -0.5 * grid_map_size_x_;
    grid_origin_y_ = -0.5 * grid_map_size_y_;
    grid_nx_ = std::max(1, (int)std::ceil(grid_map_size_x_ / grid_resolution_));
    grid_ny_ = std::max(1, (int)std::ceil(grid_map_size_y_ / grid_resolution_));
  }
  {
    auto initGrid = [&](InvalidGrid2D& g) {
      g.origin_x = grid_origin_x_;
      g.origin_y = grid_origin_y_;
      g.resolution = grid_resolution_;
      g.nx = grid_nx_;
      g.ny = grid_ny_;
      g.cells.assign(grid_nx_ * grid_ny_, 0);
      g.cell_times.assign(grid_nx_ * grid_ny_, 0.0);
    };
    initGrid(global_invalid_grid_);
    initGrid(scratch_frame_grid_);
  }
  if (grid_map_size_x_ > 0.0 && grid_map_size_y_ > 0.0) {
    ROS_INFO("[dpf%d] Invalid grid full-map mode: map_size=(%.1f,%.1f)m origin=(%.1f,%.1f) res=%.2f nx=%d ny=%d",
             drone_id_, grid_map_size_x_, grid_map_size_y_,
             grid_origin_x_, grid_origin_y_, grid_resolution_, grid_nx_, grid_ny_);
  } else {
    ROS_INFO("[dpf%d] Invalid grid legacy mode: origin=(%.1f,%.1f) res=%.2f nx=%d ny=%d",
             drone_id_, grid_origin_x_, grid_origin_y_, grid_resolution_, grid_nx_, grid_ny_);
  }

  // 搜索粒子边界约束：与无效栅格边界一致，避免粒子长期漂移到无裁剪区域之外。
  {
    const double eps = std::max(1e-6, 1e-3 * grid_resolution_);
    const double x_min = grid_origin_x_;
    const double x_max = grid_origin_x_ + grid_nx_ * grid_resolution_ - eps;
    const double y_min = grid_origin_y_;
    const double y_max = grid_origin_y_ + grid_ny_ * grid_resolution_ - eps;
    search_particles_manager_->setSearchXYBounds(x_min, x_max, y_min, y_max, true);
    ROS_INFO("[dpf%d] Search particle XY bounds enabled: x=[%.2f, %.2f], y=[%.2f, %.2f]",
             drone_id_, x_min, x_max, y_min, y_max);
  }

  // 搜索推进速度：只读取当前无人机自己的 planning/vmax；缺参时保留本地默认值
  std::string ns = ros::this_node::getNamespace();
  if (ns.empty()) {
    ns = "/";
  }
  if (ns.back() != '/') {
    ns += "/";
  }
  const std::string planning_vmax_param = ns + "planning/vmax";
  if (!ros::param::get(planning_vmax_param, search_advance_vmax_)) {
    ROS_WARN("[dpf%d] Param %s not found, keep default search_advance_vmax=%.2f",
             drone_id_, planning_vmax_param.c_str(), search_advance_vmax_);
  } else {
    ROS_INFO("[dpf%d] search_advance_vmax=%.2f from %s",
             drone_id_, search_advance_vmax_, planning_vmax_param.c_str());
  }

  // ✅ 设置视线检查函数
  search_particles_manager_->setLineOfSightCheckFn(
  [](const Eigen::Vector3d& start, const Eigen::Vector3d& end) {
    return isLineOfSightClear(start, end);
  }
);
  
  // 订阅/发布
  target_odom_pub_ = nh.advertise<nav_msgs::Odometry>("target_odom", 1);
  local_stats_pub_ = nh.advertise<target_ekf::LocalStats>("local_stats", 1);
  search_state_pub_ = nh.advertise<std_msgs::Bool>("search_state", 1);
  search_gmm_vis_pub_ = nh.advertise<visualization_msgs::MarkerArray>("search_gmm_vis", 1);
  search_pos_gmm_pub_ = nh.advertise<target_ekf::LocalStats>("search_pos_gmm", 1);
  search_gmm_vis_pub_ = nh.advertise<visualization_msgs::MarkerArray>("search_gmm_vis", 1);
  search_particles_vis_pre_prune_pub_ = nh.advertise<sensor_msgs::PointCloud2>("search_particles_vis_pre_prune", 1);
  search_particles_vis_pub_ = nh.advertise<sensor_msgs::PointCloud2>("search_particles_vis", 1);
  search_particles_vis_post_prune_pub_ = nh.advertise<sensor_msgs::PointCloud2>("search_particles_vis_post_prune", 1);
  search_branch_health_pub_ = nh.advertise<target_ekf::SearchBranchHealth>("search_branch_health", 1);
  search_targets_pub_ = nh.advertise<geometry_msgs::PoseArray>("search_targets", 1);
  invalid_grid_pub_ = nh.advertise<target_ekf::InvalidRegionGMM>("invalid_region_gmm", 1);
  search_consensus_pub_ = nh.advertise<target_ekf::LocalStats>("search_consensus", 1);
  search_fallback_model_pub_ = nh.advertise<target_ekf::LocalStats>("search_fallback_model", 1);
  
  // 地图订阅
  ros::Subscriber global_map_sub = nh.subscribe("global_map", 10, &global_map_callback);
  ros::Subscriber local_map_sub = nh.subscribe<quadrotor_msgs::OccMap3d>("gridmap_inflate", 1, &local_map_callback, ros::TransportHints().tcpNoDelay());
  // 无人机位姿订阅（单独订阅，不再同步）
  ros::Subscriber odom_sub = nh.subscribe("odom", 100, &odom_callback, ros::TransportHints().tcpNoDelay());
  // YOLO目标订阅（单独订阅，不再同步）
  ros::Subscriber yolo_sub = nh.subscribe("yolo", 1, &yolo_callback, ros::TransportHints().tcpNoDelay());
  // 订阅其他无人机的局部统计量和odom
  for (int i = 0; i < num_drones_; ++i) {
    if (i == drone_id_) continue;
    std::string topic = "/drone" + std::to_string(i) + "/drone" + std::to_string(i) + "_target_dpf/local_stats";
    stats_subs_.push_back(nh.subscribe(topic, 10, &stats_callback));

    // 订阅邻居无人机的odom（用于搜索热点分配）
    std::string odom_topic = "/drone" + std::to_string(i) + "/odom";
    neighbor_odom_subs_.push_back(
        nh.subscribe<nav_msgs::Odometry>(odom_topic, 10,
            boost::bind(&neighbor_odom_callback, _1, i)));
    ROS_INFO("[dpf%d] Subscribing to neighbor %d: stats, odom (%s)", drone_id_, i, odom_topic.c_str());

    // 订阅邻居的无效区域栅格增量（消息类型沿用InvalidRegionGMM）
    std::string invalid_grid_topic = "/drone" + std::to_string(i) + "/drone" + std::to_string(i) + "_target_dpf/invalid_region_gmm";
    invalid_grid_subs_.push_back(nh.subscribe(invalid_grid_topic, 10, &invalid_grid_callback));

    // 订阅邻居的6D搜索共识
    std::string search_cons_topic = "/drone" + std::to_string(i) + "/drone" + std::to_string(i) + "_target_dpf/search_consensus";
    search_consensus_subs_.push_back(nh.subscribe(search_cons_topic, 10, &search_consensus_callback));

    // 订阅邻居的分支健康与fallback模型
    std::string search_health_topic = "/drone" + std::to_string(i) + "/drone" + std::to_string(i) + "_target_dpf/search_branch_health";
    search_branch_health_subs_.push_back(nh.subscribe(search_health_topic, 10, &search_branch_health_callback));
    std::string search_fallback_topic = "/drone" + std::to_string(i) + "/drone" + std::to_string(i) + "_target_dpf/search_fallback_model";
    search_fallback_model_subs_.push_back(nh.subscribe(search_fallback_topic, 10, &search_fallback_model_callback));
  }
  // 【核心】观测Timer（20Hz）+ 共识Timer（100Hz）
  ros::Timer dpf_core_timer = nh.createTimer(ros::Duration(1.0 / dpf_rate_), &dpf_core_timer_callback);
  ros::Timer consensus_timer = nh.createTimer(ros::Duration(1.0 / consensus_rate_), &consensus_timer_callback);
  ros::MultiThreadedSpinner spinner(4); // 多线程Spinner，避免回调阻塞
  spinner.spin();
  return 0;
}
