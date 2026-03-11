// 分布式粒子滤波 (DPF) 仿真节点
// 100%对齐 Gu 2007 "Distributed Particle Filter for Target Tracking" Algorithm 1
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/time_synchronizer.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <unordered_set>
#include <cmath>
#include <target_ekf/target_ekf.hpp>
#include <target_ekf/target_dpf.hpp>
#include <target_ekf/LocalStats.h>

typedef message_filters::sync_policies::ApproximateTime<nav_msgs::Odometry, nav_msgs::Odometry>
    YoloOdomSyncPolicy;
typedef message_filters::Synchronizer<YoloOdomSyncPolicy>
    YoloOdomSynchronizer;

// === 全局变量 ===
ros::Publisher target_odom_pub_;
ros::Publisher local_stats_pub_;
std::vector<ros::Subscriber> stats_subs_;
Eigen::Matrix3d cam2body_R_;
Eigen::Vector3d cam2body_p_;
double fx_, fy_, cx_, cy_, width_, height_;
ros::Time last_update_stamp_;
double pitch_thr_ = 30;
bool check_fov_ = false;
bool dpf_initialized_ = false;
int dpf_reset_suppress_count_ = 0;
int drone_id_ = 0;
int num_drones_ = 3;
std::shared_ptr<DistributedPF> dpfPtr_;
// 存储从其他无人机收到的局部统计量
std::map<int, LocalStat> received_stats_;
std::mutex stats_mutex_;

// === 占据栅格 ===
struct SimpleOccMap {
  double resolution = 0.3;
  std::unordered_set<int64_t> occ_cells;
  bool received = false;
  int64_t toKey(int x, int y, int z) const {
    return ((int64_t)(x + 32768) << 32) | ((int64_t)(y + 32768) << 16) | (int64_t)(z + 32768);
  }
  void fromPointCloud(const pcl::PointCloud<pcl::PointXYZ>& cloud, double res) {
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
    int x = (int)std::floor(p.x() / resolution);
    int y = (int)std::floor(p.y() / resolution);
    int z = (int)std::floor(p.z() / resolution);
    return occ_cells.count(toKey(x, y, z)) > 0;
  }
} occMap_;

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

void global_map_callback(const sensor_msgs::PointCloud2ConstPtr& msg) {
  pcl::PointCloud<pcl::PointXYZ> cloud;
  pcl::fromROSMsg(*msg, cloud);
  occMap_.fromPointCloud(cloud, 0.3);
  ROS_INFO_ONCE("[dpf%d] Global map received, %zu obstacle points.", drone_id_, cloud.size());
}

// === LocalStats 消息 <-> 结构体转换 ===
LocalStat fromMsg(const target_ekf::LocalStats::ConstPtr& msg) {
  LocalStat ls;
  ls.drone_id = msg->drone_id;
  ls.C = msg->num_components;
  ls.nz = msg->obs_dim;
  ls.has_obs = msg->has_observation;
  ls.timestamp = msg->header.stamp;
  ls.alpha = Eigen::Map<const Eigen::VectorXd>(msg->alpha_local.data(), ls.C);
  ls.a.resize(ls.C);
  ls.b.resize(ls.C);
  for (int c = 0; c < ls.C; ++c) {
    ls.a[c] = Eigen::Map<const Eigen::VectorXd>(msg->a_local.data() + c * ls.nz, ls.nz);
    ls.b[c] = Eigen::Map<const Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>(
        msg->b_local.data() + c * ls.nz * ls.nz, ls.nz, ls.nz);
  }
  return ls;
}

target_ekf::LocalStats toMsg(const LocalStat& ls) {
  target_ekf::LocalStats msg;
  msg.header.stamp = ls.timestamp.isZero() ? ros::Time::now() : ls.timestamp;
  msg.drone_id = ls.drone_id;
  msg.num_components = ls.C;
  msg.obs_dim = ls.nz;
  msg.has_observation = ls.has_obs;
  msg.alpha_local.resize(ls.C);
  msg.a_local.resize(ls.C * ls.nz);
  msg.b_local.resize(ls.C * ls.nz * ls.nz);
  for (int c = 0; c < ls.C; ++c) {
    msg.alpha_local[c] = ls.alpha(c);
    for (int i = 0; i < ls.nz; ++i) {
      msg.a_local[c * ls.nz + i] = ls.a[c](i);
    }
    for (int i = 0; i < ls.nz; ++i) {
      for (int j = 0; j < ls.nz; ++j) {
        msg.b_local[c * ls.nz * ls.nz + i * ls.nz + j] = ls.b[c](i, j);
      }
    }
  }
  return msg;
}

// === 收到其他无人机局部统计量的回调 ===
void stats_callback(const target_ekf::LocalStats::ConstPtr& msg) {
  std::lock_guard<std::mutex> lock(stats_mutex_);
  received_stats_[msg->drone_id] = fromMsg(msg);
}

// === 定时器回调：仅发布状态，不执行预测 ===
void predict_state_callback(const ros::TimerEvent& event) {
  if (dpf_reset_suppress_count_ > 0) {
    dpf_reset_suppress_count_--;
    return;
  }
  if (!dpf_initialized_) return;

  Eigen::Vector3d est_pos = dpfPtr_->pos();
  Eigen::Vector3d est_vel = dpfPtr_->vel();
  Eigen::Vector3d est_rpy = dpfPtr_->rpy();
  ROS_DEBUG_THROTTLE(1.0, "[dpf%d][publish] pos=(%.2f,%.2f,%.2f) vel=(%.2f,%.2f,%.2f) yaw=%.2f",
    drone_id_, est_pos.x(), est_pos.y(), est_pos.z(),
    est_vel.x(), est_vel.y(), est_vel.z(), est_rpy.z());
  
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
}

// === 核心回调：严格对齐论文Algorithm 1流程 ===
void update_state_callback(const nav_msgs::OdometryConstPtr& target_msg,
  const nav_msgs::OdometryConstPtr& odom_msg) {
  // 坐标变换逻辑完全保留
  Eigen::Vector3d odom_p, p;
  Eigen::Quaterniond odom_q, q;
  odom_p(0) = odom_msg->pose.pose.position.x;
  odom_p(1) = odom_msg->pose.pose.position.y;
  odom_p(2) = odom_msg->pose.pose.position.z;
  odom_q.w() = odom_msg->pose.pose.orientation.w;
  odom_q.x() = odom_msg->pose.pose.orientation.x;
  odom_q.y() = odom_msg->pose.pose.orientation.y;
  odom_q.z() = odom_msg->pose.pose.orientation.z;
  Eigen::Vector3d cam_p = odom_q.toRotationMatrix() * cam2body_p_ + odom_p;
  Eigen::Quaterniond cam_q = odom_q * Eigen::Quaterniond(cam2body_R_);
  p.x() = target_msg->pose.pose.position.x;
  p.y() = target_msg->pose.pose.position.y;
  p.z() = target_msg->pose.pose.position.z;
  q.w() = target_msg->pose.pose.orientation.w;
  q.x() = target_msg->pose.pose.orientation.x;
  q.y() = target_msg->pose.pose.orientation.y;
  q.z() = target_msg->pose.pose.orientation.z;
  Eigen::Vector3d rpy = quaternion2euler(q);

  // FOV检查
  if (check_fov_) {
    Eigen::Vector3d p_in_body = cam_q.inverse() * (p - cam_p);
    if (p_in_body.z() < 0.1 || p_in_body.z() > 5.0) return;
    double x = p_in_body.x() * fx_ / p_in_body.z() + cx_;
    if (x < 0 || x > height_) return;
    double y = p_in_body.y() * fy_ / p_in_body.z() + cy_;
    if (y < 0 || y > width_) return;
  }
  if (!occMap_.received) return;

  // === 核心修正：先判断是否有有效观测，不提前return ===
  bool has_obs = isLineOfSightClear(cam_p, p);
  double update_dt = (ros::Time::now() - last_update_stamp_).toSec();

  // === 初始化/重置逻辑（彻底修复，绝不阻断无观测节点）===
  bool need_reset = (!dpf_initialized_ || update_dt > 1.0);
  if (need_reset) {
    if (has_obs) {
      // 有观测节点：执行重置/初始化
      dpfPtr_->reset(p, rpy);
      dpf_initialized_ = true;
      dpf_reset_suppress_count_ = 3;
      ROS_WARN("[dpf%d] reset at obs=(%.2f,%.2f,%.2f) rpy=(%.2f,%.2f,%.2f) dt=%.2fs",
        drone_id_, p.x(), p.y(), p.z(), rpy.x(), rpy.y(), rpy.z(), update_dt);
      last_update_stamp_ = ros::Time::now();
      // 【关键】重置后不return！让节点继续执行协同逻辑
    } else {
      // 无观测节点：不重置，但也不return！仅打印提示
      ROS_DEBUG_THROTTLE(1.0, "[dpf%d] no obs, skip reset (dt=%.2fs, initialized=%d)", 
        drone_id_, update_dt, dpf_initialized_);
      // 【关键】移除return，让后续协同逻辑执行
    }
  }

  // === 额外保护：未初始化的无观测节点，等待有观测节点初始化后同步 ===
  if (!dpf_initialized_) {
    ROS_DEBUG_THROTTLE(1.0, "[dpf%d] waiting for initialization (no obs yet)", drone_id_);
    // 【关键】不return！继续执行后续逻辑，尝试通过共识同步全局信息
    // 注意：未初始化时GMM/粒子集是空的，但共识滤波会尝试同步邻居信息
  }


  // ==============================================
  // 【100%对齐论文Algorithm 1 核心流程，严格按顺序执行】
  // ==============================================
  ROS_DEBUG("[dpf%d] has_obs=%d", drone_id_, has_obs);

  // --- 步骤1：计算本地统计量（有观测节点执行E步）---
  LocalStat my_stats;
  if (has_obs) {
    my_stats = dpfPtr_->computeLocalStatsOnly(drone_id_);
    local_stats_pub_.publish(toMsg(my_stats)); // 有观测：发布有效统计量
    ROS_DEBUG("[dpf%d] publish valid local stats (has_obs=1)", drone_id_);
  } else {
    my_stats = dpfPtr_->getEmptyStats(drone_id_); // 无观测：生成空统计量
    local_stats_pub_.publish(toMsg(my_stats)); // 【关键修复】无观测也发布空统计量！
    ROS_DEBUG("[dpf%d] publish empty local stats (has_obs=0)", drone_id_);
  }

  // --- 步骤2：收集邻居统计量，执行EM迭代+共识滤波（全节点执行）---
  std::vector<LocalStat> neighbor_stats;
  {
    std::lock_guard<std::mutex> lock(stats_mutex_);
    ros::Time current_time = ros::Time::now();
    double time_threshold = 0.3; // 放宽时间阈值，适配通信延迟
    for (auto& kv : received_stats_) {
      if (kv.second.has_obs) {
        double time_diff = (current_time - kv.second.timestamp).toSec();
        if (time_diff < time_threshold) {
          neighbor_stats.push_back(kv.second);
        }
      }
    }
  }

  // 论文Section V：两次观测之间EM迭代10次（核心）
  dpfPtr_->emIterate(drone_id_, neighbor_stats, my_stats);
  ROS_DEBUG("[dpf%d][EM] completed %d iterations, fused %zu neighbors",
    drone_id_, dpfPtr_->num_em_iters_, neighbor_stats.size());

  // --- 步骤3：从全局GMM采样新粒子（全节点执行，保证粒子分布同源）---
  dpfPtr_->sampleParticlesFromGMM();

  // --- 步骤4：状态预测（全节点执行，论文公式1）---
  dpfPtr_->predict();

  // --- 步骤5：权重更新（仅有观测节点执行，论文公式9）---
  if (has_obs) {
    dpfPtr_->updateWeights(p, rpy, has_obs);
    // 打印有效粒子数
    double sum_w2 = dpfPtr_->weights_.squaredNorm();
    double Neff = (sum_w2 > 1e-300) ? 1.0 / sum_w2 : 0.0;
    ROS_DEBUG("[dpf%d][weights] Neff=%.1f/%d", drone_id_, Neff, dpfPtr_->N_);

    // --- 步骤6：系统重采样（仅有观测节点执行，论文Selection步骤）---
    if (Neff < dpfPtr_->N_ * 0.5) {
      ROS_DEBUG("[dpf%d] systematic resample, Neff=%.1f", drone_id_, Neff);
      dpfPtr_->systematicResample();
    }
  }

  // --- 数值有效性检查 ---
  if (!dpfPtr_->isValid()) {
    ROS_ERROR("[dpf%d] update invalid! NaN/Inf detected, resetting.", drone_id_);
    if (has_obs) dpfPtr_->reset(p, rpy);
    dpf_reset_suppress_count_ = 3;
    return;
  }

  // --- 打印结果 ---
  Eigen::Vector3d est_pos = dpfPtr_->pos();
  Eigen::Vector3d est_vel = dpfPtr_->vel();
  Eigen::Vector3d est_rpy = dpfPtr_->rpy();
  double pos_err = has_obs ? (est_pos - p).norm() : -1.0;
  double yaw_err = has_obs ? angleDiff(est_rpy.z(), rpy.z()) * 180.0 / M_PI : -1.0;
  ROS_INFO_THROTTLE(0.5, "[dpf%d][est] pos=(%.2f,%.2f,%.2f) vel=(%.2f,%.2f,%.2f) | obs_err=%.3fm yaw_err=%.2fdeg | has_obs=%d",
    drone_id_,
    est_pos.x(), est_pos.y(), est_pos.z(),
    est_vel.x(), est_vel.y(), est_vel.z(),
    pos_err, yaw_err, has_obs);

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

  int dpf_rate = 20;
  int num_particles = 300;
  int num_components = 4;
  int num_em_iters = 10;
  nh.getParam("dpf_rate", dpf_rate);
  nh.getParam("num_particles", num_particles);
  nh.getParam("num_components", num_components);
  nh.getParam("num_em_iters", num_em_iters);

  dpfPtr_ = std::make_shared<DistributedPF>(1.0 / dpf_rate, num_particles, num_components, num_em_iters);

  // 订阅/发布
  message_filters::Subscriber<nav_msgs::Odometry> yolo_sub_;
  message_filters::Subscriber<nav_msgs::Odometry> odom_sub_;
  std::shared_ptr<YoloOdomSynchronizer> yolo_odom_sync_Ptr_;

  target_odom_pub_ = nh.advertise<nav_msgs::Odometry>("target_odom", 1);
  local_stats_pub_ = nh.advertise<target_ekf::LocalStats>("local_stats", 1);
  ros::Subscriber global_map_sub = nh.subscribe("global_map", 10, &global_map_callback);

  // 订阅其他无人机的局部统计量
  for (int i = 0; i < num_drones_; ++i) {
    if (i == drone_id_) continue;
    std::string topic = "/drone" + std::to_string(i) + "/drone" + std::to_string(i) + "_target_dpf/local_stats";
    stats_subs_.push_back(nh.subscribe(topic, 10, &stats_callback));
    ROS_INFO("[dpf%d] Subscribing to %s", drone_id_, topic.c_str());
  }

  yolo_sub_.subscribe(nh, "yolo", 1, ros::TransportHints().tcpNoDelay());
  odom_sub_.subscribe(nh, "odom", 100, ros::TransportHints().tcpNoDelay());
  yolo_odom_sync_Ptr_ = std::make_shared<YoloOdomSynchronizer>(YoloOdomSyncPolicy(200), yolo_sub_, odom_sub_);
  yolo_odom_sync_Ptr_->registerCallback(boost::bind(&update_state_callback, _1, _2));

  ros::Timer dpf_publish_timer_ = nh.createTimer(ros::Duration(1.0 / dpf_rate), &predict_state_callback);
  ros::spin();
  return 0;
}
