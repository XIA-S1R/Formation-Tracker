// target_kf_qp_sim_node
//
// 三机协同 KF + QP 目标估计节点，作为 DPF 的对比基线：
//   - 追踪阶段：谁有本机观测就由谁运行 KF predict+update；其他
//     两机直接采纳该机的后验（pos, vel）作为 target_odom，三机
//     共享同一目标估计，支撑编队跟踪。
//   - 当本机 + 所有邻机都连续 miss_detection_num_ 帧没有观测，节点
//     进入搜索/重捕模式，各机独立用本地 QP 滑动窗口拟合正则化多项
//     式外推 target；规划端按各自估计自主搜索。
//   - 任一无人机（本机或邻机）恢复观测即退出搜索模式，QP 窗口清空。
//
// 协同链路：每机发布 /drone{id}/kfqp_shared_state (nav_msgs/Odometry)，
// pose.covariance[0] == 1.0 表示该帧有真实 yolo 观测（FOV+LOS 通过），
// 否则是 KF 预测 / 采纳邻机状态的派生值。
//
// 观测链路（odom、yolo、FOV、LOS）完全沿用 target_dpf_sim_node，以
// 保证 DPF 与 KF+QP 对比实验条件一致。

#include <deque>
#include <map>
#include <mutex>
#include <string>
#include <unordered_set>
#include <vector>

#include <Eigen/Dense>
#include <boost/bind.hpp>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Bool.h>

// ==================== 参数 / 状态 ====================
namespace {

int drone_id_ = 0;
int num_drones_ = 3;
double kf_rate_ = 20.0;                     // Hz
double obs_timeout_ = 0.2;                  // yolo obs 超时
int miss_detection_num_ = 5;                // 连续几帧无观测算失锁
double max_obs_depth_ = 8.0;                // 相机最大观测距离
bool check_fov_ = true;
double cam_fx_ = 0, cam_fy_ = 0, cam_cx_ = 0, cam_cy_ = 0;
double cam_width_ = 0, cam_height_ = 0;
Eigen::Matrix3d cam2body_R_ = Eigen::Matrix3d::Identity();
Eigen::Vector3d cam2body_p_ = Eigen::Vector3d::Zero();

// QP 拟合 / 外推
int qp_window_size_ = 30;                   // 缓存最近多少条后验
int qp_poly_order_ = 3;                     // 多项式阶数
double qp_ridge_lambda_ = 1e-3;             // 正则系数
double qp_extrap_horizon_ = 1.0;            // 外推最长 horizon (s)，超过后保持最后值
double search_advance_vmax_ = 1.5;          // 发布 target_odom 时 twist 的速度幅值上限

// KF 过程/观测噪声
double kf_sigma_a_ = 1.5;                   // 过程加速度白噪声强度 (m/s^2)
double kf_sigma_z_ = 0.15;                  // 观测位置标准差 (m)

// ==================== odom / yolo 缓存 ====================
std::mutex odom_mutex_;
bool has_latest_odom_ = false;
Eigen::Vector3d latest_odom_pos_ = Eigen::Vector3d::Zero();
Eigen::Quaterniond latest_odom_q_ = Eigen::Quaterniond::Identity();

std::mutex obs_mutex_;
bool has_latest_obs_ = false;
Eigen::Vector3d latest_obs_pos_ = Eigen::Vector3d::Zero();
ros::Time latest_obs_stamp_;

// ==================== 邻机共享的目标估计 ====================
struct NeighborSharedState {
  ros::Time stamp;
  Eigen::Vector3d pos = Eigen::Vector3d::Zero();
  Eigen::Vector3d vel = Eigen::Vector3d::Zero();
  bool has_obs = false;
};
std::mutex shared_state_mutex_;
std::map<int, NeighborSharedState> received_shared_states_;
double shared_state_timeout_ = 0.5;         // 邻机 shared_state 超时(s)
ros::Publisher shared_state_pub_;
std::vector<ros::Subscriber> shared_state_subs_;

// ==================== 简单占据栅格 (LOS 检查) ====================
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

bool isLineOfSightClear(const Eigen::Vector3d& s, const Eigen::Vector3d& e) {
  if (!occMap_.received) return true;
  double dist = (e - s).norm();
  if (dist > 15.0) return false;
  int steps = std::max(1, (int)(dist / (occMap_.resolution * 0.5)));
  for (int i = 1; i < steps; ++i) {
    double t = (double)i / steps;
    if (occMap_.isOccupied(s + t * (e - s))) return false;
  }
  return true;
}

}  // namespace

// ==================== Kalman Filter (6D CV) ====================
// state = [px, py, pz, vx, vy, vz]^T
namespace {
bool kf_initialized_ = false;
Eigen::Matrix<double, 6, 1> kf_x_ = Eigen::Matrix<double, 6, 1>::Zero();
Eigen::Matrix<double, 6, 6> kf_P_ = Eigen::Matrix<double, 6, 6>::Identity();

void kfReset(const Eigen::Vector3d& p, double pos_var = 0.25, double vel_var = 1.0) {
  kf_x_.setZero();
  kf_x_.head<3>() = p;
  kf_P_.setZero();
  kf_P_.topLeftCorner<3, 3>() = Eigen::Matrix3d::Identity() * pos_var;
  kf_P_.bottomRightCorner<3, 3>() = Eigen::Matrix3d::Identity() * vel_var;
  kf_initialized_ = true;
}

void kfPredict(double dt) {
  if (!kf_initialized_ || dt <= 0.0) return;
  // F = [ I  dt*I ; 0  I ]
  Eigen::Matrix<double, 6, 6> F = Eigen::Matrix<double, 6, 6>::Identity();
  F.block<3, 3>(0, 3) = Eigen::Matrix3d::Identity() * dt;
  kf_x_ = F * kf_x_;

  // Q = sigma_a^2 * [ dt^4/4 I  dt^3/2 I ; dt^3/2 I  dt^2 I ]
  const double q = kf_sigma_a_ * kf_sigma_a_;
  const double dt2 = dt * dt, dt3 = dt2 * dt, dt4 = dt2 * dt2;
  Eigen::Matrix<double, 6, 6> Q = Eigen::Matrix<double, 6, 6>::Zero();
  Q.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity() * (q * dt4 / 4.0);
  Q.block<3, 3>(0, 3) = Eigen::Matrix3d::Identity() * (q * dt3 / 2.0);
  Q.block<3, 3>(3, 0) = Eigen::Matrix3d::Identity() * (q * dt3 / 2.0);
  Q.block<3, 3>(3, 3) = Eigen::Matrix3d::Identity() * (q * dt2);

  kf_P_ = F * kf_P_ * F.transpose() + Q;
}

void kfUpdate(const Eigen::Vector3d& z) {
  if (!kf_initialized_) { kfReset(z); return; }
  // H = [ I  0 ]
  Eigen::Matrix<double, 3, 6> H = Eigen::Matrix<double, 3, 6>::Zero();
  H.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity();
  Eigen::Matrix3d R = Eigen::Matrix3d::Identity() * (kf_sigma_z_ * kf_sigma_z_);

  Eigen::Vector3d y = z - H * kf_x_;
  Eigen::Matrix3d S = H * kf_P_ * H.transpose() + R;
  Eigen::Matrix<double, 6, 3> K = kf_P_ * H.transpose() * S.inverse();
  kf_x_ = kf_x_ + K * y;
  kf_P_ = (Eigen::Matrix<double, 6, 6>::Identity() - K * H) * kf_P_;
}
}  // namespace

// ==================== QP 滑动窗口 + 正则化多项式拟合 ====================
namespace {
struct QpSample {
  double t_sec;                // 相对窗口起点
  Eigen::Vector3d pos;         // KF 后验位置
};
std::deque<QpSample> qp_window_;
double qp_window_anchor_sec_ = 0.0;   // 相对 0 点（进入失锁时冻结）
bool qp_coef_valid_ = false;
Eigen::MatrixXd qp_coef_(0, 3);       // (d+1) x 3，列对应 xyz
double qp_last_fit_t_ = 0.0;          // 拟合覆盖到的最晚样本时间（相对 anchor）

void pushQpSample(double stamp_sec, const Eigen::Vector3d& pos) {
  qp_window_.push_back({stamp_sec, pos});
  while ((int)qp_window_.size() > std::max(4, qp_window_size_)) qp_window_.pop_front();
}

bool fitQpPolynomial() {
  const int n = (int)qp_window_.size();
  const int d = std::max(1, qp_poly_order_);
  if (n < d + 2) return false;

  // 相对时间：把窗口第一个样本当做 0
  const double t0 = qp_window_.front().t_sec;

  Eigen::MatrixXd A(n, d + 1);
  Eigen::MatrixXd B(n, 3);
  for (int i = 0; i < n; ++i) {
    double tt = qp_window_[i].t_sec - t0;
    double tp = 1.0;
    for (int k = 0; k <= d; ++k) { A(i, k) = tp; tp *= tt; }
    B.row(i) = qp_window_[i].pos.transpose();
  }

  // 正则化最小二乘: (A^T A + lambda I) c = A^T b
  Eigen::MatrixXd AtA = A.transpose() * A;
  AtA.diagonal().array() += qp_ridge_lambda_;
  Eigen::MatrixXd AtB = A.transpose() * B;
  qp_coef_ = AtA.ldlt().solve(AtB);   // (d+1) x 3
  qp_coef_valid_ = true;
  qp_window_anchor_sec_ = t0;
  qp_last_fit_t_ = qp_window_.back().t_sec - t0;
  return true;
}

// 按绝对时间 stamp_sec 外推。越界保持最后值，防止多项式外推爆炸。
Eigen::Vector3d evalQp(double stamp_sec) {
  if (!qp_coef_valid_) return Eigen::Vector3d::Zero();
  double tt = stamp_sec - qp_window_anchor_sec_;
  const double t_cap = qp_last_fit_t_ + qp_extrap_horizon_;
  if (tt > t_cap) tt = t_cap;
  Eigen::Vector3d p = Eigen::Vector3d::Zero();
  double tp = 1.0;
  for (int k = 0; k < qp_coef_.rows(); ++k) {
    p += qp_coef_.row(k).transpose() * tp;
    tp *= tt;
  }
  return p;
}

Eigen::Vector3d evalQpVel(double stamp_sec) {
  if (!qp_coef_valid_ || qp_coef_.rows() < 2) return Eigen::Vector3d::Zero();
  double tt = stamp_sec - qp_window_anchor_sec_;
  const double t_cap = qp_last_fit_t_ + qp_extrap_horizon_;
  if (tt > t_cap) tt = t_cap;
  Eigen::Vector3d v = Eigen::Vector3d::Zero();
  double tp = 1.0;
  for (int k = 1; k < qp_coef_.rows(); ++k) {
    v += qp_coef_.row(k).transpose() * (k * tp);
    tp *= tt;
  }
  return v;
}

void clearQpWindow() {
  qp_window_.clear();
  qp_coef_valid_ = false;
  qp_coef_.resize(0, 3);
}
}  // namespace

// ==================== 运行时状态 ====================
namespace {
ros::Publisher target_odom_pub_;
ros::Publisher search_state_pub_;

ros::Time last_predict_stamp_;
int consecutive_no_obs_count_ = 0;
bool search_mode_active_ = false;

// 失锁前最后一次有效观测时的 KF 速度（世界系），用于重捕获前作为 fallback
Eigen::Vector3d last_obs_vel_world_ = Eigen::Vector3d::Zero();
bool has_last_obs_vel_ = false;

ros::Time search_start_stamp_;
}  // namespace

// ==================== 回调 ====================
void odom_callback(const nav_msgs::OdometryConstPtr& msg) {
  std::lock_guard<std::mutex> lock(odom_mutex_);
  latest_odom_pos_.x() = msg->pose.pose.position.x;
  latest_odom_pos_.y() = msg->pose.pose.position.y;
  latest_odom_pos_.z() = msg->pose.pose.position.z;
  latest_odom_q_.w() = msg->pose.pose.orientation.w;
  latest_odom_q_.x() = msg->pose.pose.orientation.x;
  latest_odom_q_.y() = msg->pose.pose.orientation.y;
  latest_odom_q_.z() = msg->pose.pose.orientation.z;
  has_latest_odom_ = true;
}

void yolo_callback(const nav_msgs::OdometryConstPtr& msg) {
  std::lock_guard<std::mutex> lock(obs_mutex_);
  latest_obs_pos_.x() = msg->pose.pose.position.x;
  latest_obs_pos_.y() = msg->pose.pose.position.y;
  latest_obs_pos_.z() = msg->pose.pose.position.z;
  latest_obs_stamp_ = msg->header.stamp;
  has_latest_obs_ = true;
}

void global_map_callback(const sensor_msgs::PointCloud2ConstPtr& msg) {
  pcl::PointCloud<pcl::PointXYZ> cloud;
  pcl::fromROSMsg(*msg, cloud);
  occMap_.fromPointCloud(cloud, 0.3);
  ROS_INFO_ONCE("[kfqp%d] Global map received, %zu pts.", drone_id_, cloud.size());
}

// ==================== 邻机 shared_state 回调 ====================
void shared_state_callback(const nav_msgs::OdometryConstPtr& msg, int neighbor_id) {
  if (neighbor_id == drone_id_) return;  // ignore self-echo
  NeighborSharedState st;
  st.stamp = msg->header.stamp.isZero() ? ros::Time::now() : msg->header.stamp;
  st.pos.x() = msg->pose.pose.position.x;
  st.pos.y() = msg->pose.pose.position.y;
  st.pos.z() = msg->pose.pose.position.z;
  st.vel.x() = msg->twist.twist.linear.x;
  st.vel.y() = msg->twist.twist.linear.y;
  st.vel.z() = msg->twist.twist.linear.z;
  // pose.covariance[0] == 1.0 编码“本帧有真实观测”
  st.has_obs = (msg->pose.covariance.size() > 0 && msg->pose.covariance[0] > 0.5);
  std::lock_guard<std::mutex> lock(shared_state_mutex_);
  received_shared_states_[neighbor_id] = st;
}

// ==================== 主回调 ====================
// 每帧 20Hz 执行一次。状态机与 target_dpf_sim_node 对齐：
//   swarm_has_obs = (本机 has_obs) || (任一邻机 shared_state.has_obs && 未超时)
//   all_drones_no_obs = !swarm_has_obs
// 追踪模式（!search_mode_active_）：
//   · 本机有观测 → 本地 KF predict+update
//   · 否则采纳最新的“有观测邻机后验” → 覆写 KF 状态
//   · 全员无观测且未到阈值 → KF 匀速外推
//   只有 all_drones_no_obs 连续 miss_detection_num_ 帧才切到搜索模式。
// 搜索模式（search_mode_active_）：
//   · 本机重捕（has_obs）或任一邻机重捕 → 立即退出，KF 从该观测/邻机状态重置
//   · 否则三机各自独立用本地 QP 外推
// 每帧末尾统一发布 target_odom（跟踪用）+ shared_state（共识用）+ search_state。
void kf_core_timer_callback(const ros::TimerEvent& /*ev*/) {
  if (!has_latest_odom_) return;

  // ==== 读本机位姿 / 相机外参 ====
  Eigen::Vector3d cam_p;
  Eigen::Quaterniond cam_q;
  {
    std::lock_guard<std::mutex> lock(odom_mutex_);
    cam_p = latest_odom_q_.toRotationMatrix() * cam2body_p_ + latest_odom_pos_;
    cam_q = latest_odom_q_ * Eigen::Quaterniond(cam2body_R_);
  }

  // ==== 1) 本机观测（FOV + LOS）====
  bool has_obs = false;
  Eigen::Vector3d obs_pos = Eigen::Vector3d::Zero();
  {
    std::lock_guard<std::mutex> lock(obs_mutex_);
    if (has_latest_obs_ &&
        (ros::Time::now() - latest_obs_stamp_).toSec() < obs_timeout_) {
      bool in_fov = true;
      if (check_fov_) {
        Eigen::Vector3d p_body = cam_q.inverse() * (latest_obs_pos_ - cam_p);
        in_fov = false;
        if (p_body.z() > 0.1 && p_body.z() < max_obs_depth_) {
          double u = p_body.x() * cam_fx_ / p_body.z() + cam_cx_;
          double v = p_body.y() * cam_fy_ / p_body.z() + cam_cy_;
          in_fov = (u >= 0 && u <= cam_width_ && v >= 0 && v <= cam_height_);
        }
      }
      if (in_fov && isLineOfSightClear(cam_p, latest_obs_pos_)) {
        has_obs = true;
        obs_pos = latest_obs_pos_;
      }
    }
    has_latest_obs_ = false;
  }

  const ros::Time now = ros::Time::now();
  double dt = (last_predict_stamp_.isValid())
                ? (now - last_predict_stamp_).toSec()
                : 1.0 / std::max(1.0, kf_rate_);
  if (dt <= 0.0 || dt > 1.0) dt = 1.0 / std::max(1.0, kf_rate_);
  last_predict_stamp_ = now;

  // ==== 2) 挑选最新的“有观测邻机”（对齐 DPF 的 stats_callback） ====
  bool neighbor_has_obs = false;
  NeighborSharedState neighbor_src;
  {
    std::lock_guard<std::mutex> lock(shared_state_mutex_);
    ros::Time best_stamp(0);
    for (const auto& kv : received_shared_states_) {
      const auto& st = kv.second;
      if ((now - st.stamp).toSec() > shared_state_timeout_) continue;
      if (!st.has_obs) continue;
      if (st.stamp > best_stamp) {
        best_stamp = st.stamp;
        neighbor_src = st;
        neighbor_has_obs = true;
      }
    }
  }
  const bool swarm_has_obs = has_obs || neighbor_has_obs;
  const bool all_drones_no_obs = !swarm_has_obs;

  // ==== 3) 搜索模式优先退出检查（对齐 DPF 里 "优先退出检查1/2") ====
  if (search_mode_active_) {
    if (has_obs) {
      // 本机重捕 → 用本机观测重置 KF
      kfReset(obs_pos);
      clearQpWindow();
      search_mode_active_ = false;
      consecutive_no_obs_count_ = 0;
      ROS_WARN("[kfqp%d] EXIT SEARCH MODE: self reacquired at (%.2f,%.2f,%.2f)",
               drone_id_, obs_pos.x(), obs_pos.y(), obs_pos.z());
    } else if (neighbor_has_obs) {
      // 邻机重捕 → 采纳邻机后验作为本地 KF 初值
      kf_x_.head<3>() = neighbor_src.pos;
      kf_x_.tail<3>() = neighbor_src.vel;
      kf_P_.setZero();
      kf_P_.topLeftCorner<3, 3>() = Eigen::Matrix3d::Identity() * 0.25;
      kf_P_.bottomRightCorner<3, 3>() = Eigen::Matrix3d::Identity() * 1.0;
      kf_initialized_ = true;
      last_obs_vel_world_ = neighbor_src.vel;
      has_last_obs_vel_ = true;
      clearQpWindow();
      search_mode_active_ = false;
      consecutive_no_obs_count_ = 0;
      ROS_WARN("[kfqp%d] EXIT SEARCH MODE: neighbor reacquired at (%.2f,%.2f,%.2f)",
               drone_id_, neighbor_src.pos.x(), neighbor_src.pos.y(), neighbor_src.pos.z());
    }
    // 没任何人有观测 → 搜索模式继续
  }

  // ==== 4) 更新 no-obs 计数与进入搜索判定（与 DPF 对齐：all_drones_no_obs 才计数） ====
  if (all_drones_no_obs) {
    if (!search_mode_active_) {
      consecutive_no_obs_count_++;
      if (consecutive_no_obs_count_ >= miss_detection_num_ && kf_initialized_) {
        search_mode_active_ = true;
        search_start_stamp_ = now;
        if (!fitQpPolynomial()) {
          ROS_WARN("[kfqp%d] ENTER SEARCH MODE: QP fit failed (n=%zu), "
                   "fallback to linear extrap.", drone_id_, qp_window_.size());
        } else {
          ROS_WARN("[kfqp%d] ENTER SEARCH MODE: QP fit OK, window=%zu, poly_order=%d.",
                   drone_id_, qp_window_.size(), qp_poly_order_);
        }
      }
    }
  } else {
    // 有任意无人机看到目标：清零计数
    consecutive_no_obs_count_ = 0;
  }

  // ==== 5) 估计更新 ====
  if (search_mode_active_) {
    // 搜索期 KF 冻结，由 QP 外推 —— 什么都不做
  } else if (has_obs) {
    // 本机有观测 → 本地 KF predict + update
    if (!kf_initialized_) {
      kfReset(obs_pos);
    } else {
      kfPredict(dt);
      kfUpdate(obs_pos);
    }
    last_obs_vel_world_ = kf_x_.tail<3>();
    has_last_obs_vel_ = true;
    pushQpSample(now.toSec(), kf_x_.head<3>());
  } else if (neighbor_has_obs) {
    // 本机无观测但邻机有 → 直接采纳邻机后验
    kf_x_.head<3>() = neighbor_src.pos;
    kf_x_.tail<3>() = neighbor_src.vel;
    kf_P_.setZero();
    kf_P_.topLeftCorner<3, 3>() = Eigen::Matrix3d::Identity() * 0.25;
    kf_P_.bottomRightCorner<3, 3>() = Eigen::Matrix3d::Identity() * 1.0;
    kf_initialized_ = true;
    last_obs_vel_world_ = neighbor_src.vel;
    has_last_obs_vel_ = true;
    pushQpSample(now.toSec(), kf_x_.head<3>());
  } else if (kf_initialized_) {
    // 全员无观测但未到搜索阈值 → KF 匀速外推
    kfPredict(dt);
  }

  // ==== 6) 还没初始化就只广播 search_state/shared_state(has_obs=false)，不发 target_odom ====
  if (!kf_initialized_) {
    std_msgs::Bool flag;
    flag.data = search_mode_active_;
    search_state_pub_.publish(flag);
    return;
  }

  // ==== 7) 组装输出 ====
  Eigen::Vector3d out_pos, out_vel;
  if (search_mode_active_) {
    if (qp_coef_valid_) {
      out_pos = evalQp(now.toSec());
      out_vel = evalQpVel(now.toSec());
    } else {
      double elapsed = (now - search_start_stamp_).toSec();
      out_vel = has_last_obs_vel_ ? last_obs_vel_world_ : Eigen::Vector3d::Zero();
      out_pos = kf_x_.head<3>() + out_vel * elapsed;
    }
    double vn = out_vel.norm();
    if (vn > search_advance_vmax_ && vn > 1e-9) {
      out_vel *= (search_advance_vmax_ / vn);
    }
  } else {
    out_pos = kf_x_.head<3>();
    out_vel = kf_x_.tail<3>();
  }

  nav_msgs::Odometry odom;
  odom.header.stamp = now;
  odom.header.frame_id = "world";
  odom.pose.pose.position.x = out_pos.x();
  odom.pose.pose.position.y = out_pos.y();
  odom.pose.pose.position.z = out_pos.z();
  const double yaw = std::atan2(out_vel.y(), out_vel.x());
  Eigen::Quaterniond q_yaw(Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()));
  odom.pose.pose.orientation.w = q_yaw.w();
  odom.pose.pose.orientation.x = q_yaw.x();
  odom.pose.pose.orientation.y = q_yaw.y();
  odom.pose.pose.orientation.z = q_yaw.z();
  odom.twist.twist.linear.x = out_vel.x();
  odom.twist.twist.linear.y = out_vel.y();
  odom.twist.twist.linear.z = out_vel.z();
  target_odom_pub_.publish(odom);

  // shared_state：内容与 target_odom 一致；covariance[0] 标记 has_obs
  // 搜索模式下所有机都发 has_obs=false（外推产物），不会把搜索期的外推误当观测扩散。
  nav_msgs::Odometry shared = odom;
  shared.pose.covariance[0] = has_obs ? 1.0 : 0.0;
  shared_state_pub_.publish(shared);

  std_msgs::Bool flag;
  flag.data = search_mode_active_;
  search_state_pub_.publish(flag);
}

// ==================== main ====================
int main(int argc, char** argv) {
  ros::init(argc, argv, "target_kf_qp_sim_node");
  ros::NodeHandle nh("~");

  nh.param<int>("drone_id", drone_id_, 0);
  nh.param("num_drones", num_drones_, 3);
  nh.param("kf_rate", kf_rate_, 20.0);
  nh.param("obs_timeout", obs_timeout_, 0.2);
  nh.param("miss_detection_num", miss_detection_num_, 5);
  nh.param("max_obs_depth", max_obs_depth_, 8.0);
  nh.param("check_fov", check_fov_, true);
  nh.param("shared_state_timeout", shared_state_timeout_, 0.5);

  nh.getParam("cam_fx", cam_fx_);
  nh.getParam("cam_fy", cam_fy_);
  nh.getParam("cam_cx", cam_cx_);
  nh.getParam("cam_cy", cam_cy_);
  nh.getParam("cam_width", cam_width_);
  nh.getParam("cam_height", cam_height_);
  {
    std::vector<double> tmp;
    if (nh.param<std::vector<double>>("cam2body_R", tmp, std::vector<double>()) &&
        tmp.size() == 9) {
      cam2body_R_ = Eigen::Map<const Eigen::Matrix<double, -1, -1, Eigen::RowMajor>>(
          tmp.data(), 3, 3);
    }
    if (nh.param<std::vector<double>>("cam2body_p", tmp, std::vector<double>()) &&
        tmp.size() == 3) {
      cam2body_p_ = Eigen::Vector3d(tmp[0], tmp[1], tmp[2]);
    }
  }

  nh.param("qp_window_size", qp_window_size_, 30);
  nh.param("qp_poly_order", qp_poly_order_, 3);
  nh.param("qp_ridge_lambda", qp_ridge_lambda_, 1e-3);
  nh.param("qp_extrap_horizon", qp_extrap_horizon_, 1.0);
  nh.param("search_advance_vmax", search_advance_vmax_, 1.5);
  nh.param("kf_sigma_a", kf_sigma_a_, 1.5);
  nh.param("kf_sigma_z", kf_sigma_z_, 0.15);

  target_odom_pub_ = nh.advertise<nav_msgs::Odometry>("target_odom", 1);
  search_state_pub_ = nh.advertise<std_msgs::Bool>("search_state", 1);
  // shared_state 发布到 /drone{id}/kfqp_shared_state，供邻机订阅
  {
    ros::NodeHandle gh;  // global
    const std::string self_topic =
        "/drone" + std::to_string(drone_id_) + "/kfqp_shared_state";
    shared_state_pub_ = gh.advertise<nav_msgs::Odometry>(self_topic, 1);

    // 订阅所有其他 drone 的 shared_state
    shared_state_subs_.reserve(std::max(0, num_drones_ - 1));
    for (int i = 0; i < num_drones_; ++i) {
      if (i == drone_id_) continue;
      const std::string topic = "/drone" + std::to_string(i) + "/kfqp_shared_state";
      shared_state_subs_.push_back(gh.subscribe<nav_msgs::Odometry>(
          topic, 1,
          boost::bind(&shared_state_callback, _1, i),
          ros::VoidConstPtr(), ros::TransportHints().tcpNoDelay()));
    }
  }

  ros::Subscriber odom_sub = nh.subscribe("odom", 100, &odom_callback,
                                          ros::TransportHints().tcpNoDelay());
  ros::Subscriber yolo_sub = nh.subscribe("yolo", 1, &yolo_callback,
                                          ros::TransportHints().tcpNoDelay());
  ros::Subscriber map_sub = nh.subscribe("global_map", 10, &global_map_callback);

  ros::Timer timer = nh.createTimer(ros::Duration(1.0 / std::max(1.0, kf_rate_)),
                                    &kf_core_timer_callback);

  ROS_INFO("[kfqp%d] started: rate=%.1fHz miss_det=%d qp_win=%d poly=%d lambda=%.3g "
           "horizon=%.2fs vmax=%.2f sigma_a=%.2f sigma_z=%.2f swarm=%d shared_to=%.2fs",
           drone_id_, kf_rate_, miss_detection_num_, qp_window_size_, qp_poly_order_,
           qp_ridge_lambda_, qp_extrap_horizon_, search_advance_vmax_,
           kf_sigma_a_, kf_sigma_z_, num_drones_, shared_state_timeout_);

  ros::spin();
  return 0;
}
