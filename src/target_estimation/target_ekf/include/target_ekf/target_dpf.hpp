#pragma once
#include <Eigen/Dense>
#include <vector>
#include <random>
#include <cmath>
#include <algorithm>
#include <numeric>
#include <string>
#include <ros/ros.h>

// 角度归一化到 [-pi, pi]
inline double wrapAngle(double a) {
  while (a > M_PI) a -= 2.0 * M_PI;
  while (a < -M_PI) a += 2.0 * M_PI;
  return a;
}
// 计算两个角度之差 (a - b)，结果在 [-pi, pi]
inline double angleDiff(double a, double b) {
  return wrapAngle(a - b);
}

// 局部统计量结构体，严格对应论文公式(21)
struct LocalStat {
  int drone_id;
  int C;        // GMM 分量数
  int nz;       // 观测维度 (6: x,y,z,roll,pitch,yaw)
  Eigen::VectorXd alpha;              // [C] 对应 kα_{m,c}^t
  std::vector<Eigen::VectorXd> a;     // C 个 [nz] 向量 对应 a_{m,c}^t
  std::vector<Eigen::MatrixXd> b;     // C 个 [nz x nz] 矩阵 对应 b_{m,c}^t
  bool has_obs;
  ros::Time timestamp;  // 时间戳，用于时间同步
};

// DPF 核心类：100%对齐 Gu 2007 论文 Algorithm 1
// 状态维度 9: [x, y, z, vx, vy, vz, roll, pitch, yaw]
// 观测维度 6: [x, y, z, roll, pitch, yaw]
struct DistributedPF {
  // === 论文核心参数 ===
  int N_;       // 粒子数
  int nx_;      // 状态维度 = 9
  int nz_;      // 观测维度 = 6
  int C_;       // GMM 分量数
  double dt_;   // 时间步长
  int num_em_iters_;  // 论文Section V：两次观测之间EM迭代10次
  double consensus_epsilon_; // 论文公式(27)：共识滤波更新率 ε ≤ 1/d_max
  int num_consensus_iters_;  // 共识滤波迭代次数，保证收敛

  // === 粒子集 ===
  Eigen::MatrixXd particles_;   // [nx_ x N_] 每列一个粒子
  Eigen::VectorXd weights_;     // [N_]

  // === 运动模型 (论文公式1) ===
  Eigen::MatrixXd A_;           // [nx_ x nx_] 状态转移
  Eigen::MatrixXd Q_;           // [nx_ x nx_] 过程噪声协方差

  // === 观测模型 (论文公式2) ===
  Eigen::MatrixXd R_;           // [nz_ x nz_] 观测噪声协方差

  // === GMM 参数 (论文公式30，M步输出) ===
  Eigen::VectorXd gmm_pi_;              // [C_] 混合权重
  std::vector<Eigen::VectorXd> gmm_mu_; // C_ 个 [nz_] 均值
  std::vector<Eigen::MatrixXd> gmm_S_;  // C_ 个 [nz_ x nz_] 协方差

  // === 共识滤波状态 (论文公式26) ===
  Eigen::VectorXd zeta_alpha_;    // [C_] 全局α的共识估计
  std::vector<Eigen::VectorXd> zeta_a_; // C个[nz] 全局a的共识估计
  std::vector<Eigen::MatrixXd> zeta_b_; // C个[nz×nz] 全局b的共识估计

  // === 状态 ===
  bool initialized_;
  std::mt19937 rng_;
  
  // 历史位置记录
  Eigen::Vector3d last_pos_;
  ros::Time last_pos_time_;
  
  // 移动平均滤波参数
  Eigen::Vector3d pos_filtered_;
  Eigen::Vector3d vel_filtered_;
  Eigen::Vector3d rpy_filtered_;
  double filter_alpha_ = 0.9;

  // === 构造函数：严格按论文参数初始化 ===
  DistributedPF(double dt, int num_particles = 300, int num_components = 4, int num_em_iters = 10)
      : N_(num_particles), nx_(9), nz_(6), C_(num_components), dt_(dt),
        num_em_iters_(num_em_iters),
        consensus_epsilon_(0.1), // 论文公式(27)，默认≤1/d_max，d_max为节点最大度
        num_consensus_iters_(5), // 共识迭代次数，保证收敛
        initialized_(false), rng_(std::random_device{}()) {
    // 初始化滤波值
    pos_filtered_.setZero();
    vel_filtered_.setZero();
    rpy_filtered_.setZero();

    // 状态转移矩阵A (论文公式32，匀速模型)
    A_.setIdentity(nx_, nx_);
    A_(0, 3) = dt_;
    A_(1, 4) = dt_;
    A_(2, 5) = dt_;

    // 过程噪声Q (论文Section V仿真参数，适配9维状态)
    Q_.setZero(nx_, nx_);
    double t2 = dt_ * dt_;
    // 位置噪声
    Q_(0, 0) = 0.2 * t2;  Q_(1, 1) = 0.2 * t2;  Q_(2, 2) = 0.1 * t2;
    // 速度噪声 (论文仿真用0.25，适配场景调整)
    Q_(3, 3) = 0.1 * t2;  Q_(4, 4) = 0.1 * t2;  Q_(5, 5) = 0.05 * t2;
    // 姿态噪声
    Q_(6, 6) = 0.1 * t2;  Q_(7, 7) = 0.1 * t2;  Q_(8, 8) = 0.01 * t2;

    // 观测噪声R (论文Section V仿真参数)
    R_.setIdentity(nz_, nz_);
    R_(0, 0) = 0.1;  R_(1, 1) = 0.1;  R_(2, 2) = 0.1;
    R_(3, 3) = 0.01; R_(4, 4) = 0.01; R_(5, 5) = 0.01;

    // 粒子和权重初始化
    particles_.setZero(nx_, N_);
    weights_.setConstant(N_, 1.0 / N_);

    // GMM初始化
    gmm_pi_.setConstant(C_, 1.0 / C_);
    gmm_mu_.resize(C_);
    gmm_S_.resize(C_);
    for (int c = 0; c < C_; ++c) {
      gmm_mu_[c].setZero(nz_);
      gmm_S_[c].setIdentity(nz_, nz_);
    }

    // 共识滤波状态初始化
    zeta_alpha_.setZero(C_);
    zeta_a_.resize(C_);
    zeta_b_.resize(C_);
    for (int c = 0; c < C_; ++c) {
      zeta_a_[c].setZero(nz_);
      zeta_b_[c].setZero(nz_, nz_);
    }
  }

  // === 初始化：严格对应论文Algorithm 1 Initialization步骤 ===
  inline void init(const Eigen::Vector3d& z0, const Eigen::Vector3d& z0_rpy) {
    std::normal_distribution<double> dist(0.0, 1.0);
    for (int i = 0; i < N_; ++i) {
      particles_(0, i) = z0.x() + dist(rng_) * 0.5;
      particles_(1, i) = z0.y() + dist(rng_) * 0.5;
      particles_(2, i) = z0.z() + dist(rng_) * 0.3;
      particles_(3, i) = dist(rng_) * 0.1;
      particles_(4, i) = dist(rng_) * 0.1;
      particles_(5, i) = dist(rng_) * 0.05;
      particles_(6, i) = wrapAngle(z0_rpy.x() + dist(rng_) * 0.1);
      particles_(7, i) = wrapAngle(z0_rpy.y() + dist(rng_) * 0.1);
      particles_(8, i) = wrapAngle(z0_rpy.z() + dist(rng_) * 0.05);
    }
    weights_.setConstant(N_, 1.0 / N_);

    // GMM初始化
    Eigen::VectorXd z_full(nz_);
    z_full.head(3) = z0;
    z_full.tail(3) = z0_rpy;
    for (int c = 0; c < C_; ++c) {
      gmm_mu_[c] = z_full;
      gmm_mu_[c](0) += dist(rng_) * 0.3;
      gmm_mu_[c](1) += dist(rng_) * 0.3;
      gmm_mu_[c](2) += dist(rng_) * 0.1;
      gmm_mu_[c](3) += dist(rng_) * 0.05;
      gmm_mu_[c](4) += dist(rng_) * 0.05;
      gmm_mu_[c](5) += dist(rng_) * 0.02;
      gmm_S_[c] = R_ * 2.0;
    }
    gmm_pi_.setConstant(C_, 1.0 / C_);

    // 共识状态初始化
    zeta_alpha_ = gmm_pi_ * N_;
    for (int c = 0; c < C_; ++c) {
      zeta_a_[c] = gmm_mu_[c] * zeta_alpha_(c);
      zeta_b_[c] = gmm_S_[c] * zeta_alpha_(c);
    }

    initialized_ = true;
    last_pos_ = z0;
    last_pos_time_ = ros::Time::now();
  }

  inline void reset(const Eigen::Vector3d& z0, const Eigen::Vector3d& z0_rpy) {
    init(z0, z0_rpy);
  }

  // === 观测函数h(x)：严格对应论文公式(31) ===
  inline Eigen::VectorXd h(const Eigen::VectorXd& x) const {
    Eigen::VectorXd z(nz_);
    z.head(3) = x.head(3);
    z.tail(3) = x.tail(3);
    return z;
  }

  // === 角度感知残差计算 ===
  inline Eigen::VectorXd obsDiff(const Eigen::VectorXd& z_obs, const Eigen::VectorXd& z_pred) const {
    Eigen::VectorXd diff(nz_);
    diff.head(3) = z_obs.head(3) - z_pred.head(3);
    diff(3) = angleDiff(z_obs(3), z_pred(3));
    diff(4) = angleDiff(z_obs(4), z_pred(4));
    diff(5) = angleDiff(z_obs(5), z_pred(5));
    return diff;
  }

  // === 论文公式(21)：EM-E步，纯本地统计量计算，不修改GMM ===
  inline LocalStat computeLocalStatsOnly(int drone_id) {
    LocalStat ls;
    ls.drone_id = drone_id;
    ls.C = C_;
    ls.nz = nz_;
    ls.has_obs = true;
    ls.timestamp = ros::Time::now();
    ls.alpha.setZero(C_);
    ls.a.resize(C_);
    ls.b.resize(C_);
    for (int c = 0; c < C_; ++c) {
      ls.a[c].setZero(nz_);
      ls.b[c].setZero(nz_, nz_);
    }

    // 严格按论文公式(17)计算责任度，公式(21)累加统计量
    for (int n = 0; n < N_; ++n) {
      Eigen::VectorXd z_n = h(particles_.col(n));
      double w_n = weights_(n);
      Eigen::VectorXd resp(C_);
      double resp_sum = 0.0;

      for (int c = 0; c < C_; ++c) {
        Eigen::MatrixXd S_inv = gmm_S_[c].inverse();
        double S_det = gmm_S_[c].determinant();
        if (S_det < 1e-300) S_det = 1e-300;
        Eigen::VectorXd diff = obsDiff(z_n, gmm_mu_[c]);
        double exponent = -0.5 * diff.transpose() * S_inv * diff;
        double nc = 1.0 / std::sqrt(std::pow(2.0 * M_PI, nz_) * S_det);
        resp(c) = gmm_pi_(c) * nc * std::exp(exponent);
        resp_sum += resp(c);
      }
      if (resp_sum > 1e-300) resp /= resp_sum;
      else resp.setConstant(1.0 / C_);

      // 论文公式(21)累加局部统计量
      for (int c = 0; c < C_; ++c) {
        double alpha_nc = w_n * resp(c);
        ls.alpha(c) += alpha_nc;
        ls.a[c] += alpha_nc * z_n;
        Eigen::VectorXd diff_c = obsDiff(z_n, gmm_mu_[c]);
        ls.b[c] += alpha_nc * diff_c * diff_c.transpose();
      }
    }
    return ls;
  }

  // === 无观测节点：生成空统计量，不参与本地统计量计算 ===
  inline LocalStat getEmptyStats(int drone_id) {
    LocalStat ls;
    ls.drone_id = drone_id;
    ls.C = C_;
    ls.nz = nz_;
    ls.has_obs = false;
    ls.timestamp = ros::Time::now();
    ls.alpha.setZero(C_);
    ls.a.resize(C_);
    ls.b.resize(C_);
    for (int c = 0; c < C_; ++c) {
      ls.a[c].setZero(nz_);
      ls.b[c].setZero(nz_, nz_);
    }
    return ls;
  }

  // === 论文公式(26)：平均共识滤波，迭代估计全局统计量 ===
  // 核心：无观测节点通过此步骤同步全局统计量
  // === 论文公式(26)：平均共识滤波，迭代估计全局统计量 ===
  inline void consensusFilter(const std::vector<LocalStat>& neighbor_stats, const LocalStat& local_stat) {
    // 【新增保护】未初始化时，用邻居统计量初始化共识状态
    if (!initialized_ && !neighbor_stats.empty()) {
      // 用第一个有观测的邻居统计量初始化共识状态
      for (const auto& stat : neighbor_stats) {
        if (stat.has_obs) {
          zeta_alpha_ = stat.alpha;
          zeta_a_ = stat.a;
          zeta_b_ = stat.b;
          initialized_ = true; // 标记为已初始化（通过共识同步）
          ROS_INFO("[dpf%d] initialized via consensus from neighbor %d", 
            local_stat.drone_id, stat.drone_id);
          break;
        }
      }
      if (!initialized_) return; // 仍未初始化，直接返回
    }
    // 原有共识滤波逻辑（保留）
    //int M = neighbor_stats.size() + 1; // 总节点数=邻居数+自身
    for (int iter = 0; iter < num_consensus_iters_; ++iter) {
      // 论文公式(26)：ζ_{t+1} = ζ_t + ε[Σ(ζ_j - ζ_m) + (u_m - ζ_m)]
      for (int c = 0; c < C_; ++c) {
        // 1. 邻居与自身的差值和
        Eigen::VectorXd alpha_diff = Eigen::VectorXd::Zero(C_);
        std::vector<Eigen::VectorXd> a_diff(C_, Eigen::VectorXd::Zero(nz_));
        std::vector<Eigen::MatrixXd> b_diff(C_, Eigen::MatrixXd::Zero(nz_, nz_));

        for (const auto& stat : neighbor_stats) {
          if (!stat.has_obs) continue; // 仅用有观测节点的统计量
          alpha_diff(c) += stat.alpha(c) - zeta_alpha_(c);
          a_diff[c] += stat.a[c] - zeta_a_[c];
          b_diff[c] += stat.b[c] - zeta_b_[c];
        }

        // 2. 自身本地统计量与共识状态的差值
        double local_alpha = local_stat.has_obs ? local_stat.alpha(c) : 0.0;
        Eigen::VectorXd local_a = local_stat.has_obs ? local_stat.a[c] : Eigen::VectorXd::Zero(nz_);
        Eigen::MatrixXd local_b = local_stat.has_obs ? local_stat.b[c] : Eigen::MatrixXd::Zero(nz_, nz_);

        // 3. 论文公式(26)迭代更新
        zeta_alpha_(c) += consensus_epsilon_ * (alpha_diff(c) + (local_alpha - zeta_alpha_(c)));
        zeta_a_[c] += consensus_epsilon_ * (a_diff[c] + (local_a - zeta_a_[c]));
        zeta_b_[c] += consensus_epsilon_ * (b_diff[c] + (local_b - zeta_b_[c]));
      }
    }
  }

  // === 论文公式(30)：全局M步，用共识后的全局统计量更新GMM ===
  inline void globalMStep() {
    double alpha_sum = zeta_alpha_.sum();
    if (alpha_sum < 1e-300) return;

    for (int c = 0; c < C_; ++c) {
      if (zeta_alpha_(c) < 1e-300) continue;
      // 论文公式(30)更新GMM参数
      gmm_pi_(c) = zeta_alpha_(c) / alpha_sum;
      gmm_mu_[c] = zeta_a_[c] / zeta_alpha_(c);
      // 归一化角度
      gmm_mu_[c](3) = wrapAngle(gmm_mu_[c](3));
      gmm_mu_[c](4) = wrapAngle(gmm_mu_[c](4));
      gmm_mu_[c](5) = wrapAngle(gmm_mu_[c](5));
      gmm_S_[c] = zeta_b_[c] / zeta_alpha_(c);
      // 保证协方差正定
      gmm_S_[c] += Eigen::MatrixXd::Identity(nz_, nz_) * 1e-4;
    }
    // 归一化混合权重
    double pi_sum = gmm_pi_.sum();
    if (pi_sum > 1e-300) gmm_pi_ /= pi_sum;
  }

  // === 论文Algorithm 1：从全局GMM采样新粒子 ===
  inline void sampleParticlesFromGMM() {
    std::uniform_real_distribution<double> uniform(0.0, 1.0);
    std::normal_distribution<double> normal(0.0, 1.0);
    Eigen::MatrixXd new_particles(nx_, N_);

    for (int i = 0; i < N_; ++i) {
      // 按混合权重选择GMM分量
      double u = uniform(rng_);
      double cum = 0.0;
      int c_sel = C_ - 1;
      for (int c = 0; c < C_; ++c) {
        cum += gmm_pi_(c);
        if (u <= cum) { c_sel = c; break; }
      }
      // 从GMM采样观测维度（位置+姿态）
      Eigen::LLT<Eigen::MatrixXd> llt(gmm_S_[c_sel]);
      Eigen::MatrixXd L = llt.matrixL();
      Eigen::VectorXd noise_z(nz_);
      for (int j = 0; j < nz_; ++j) noise_z(j) = normal(rng_);
      Eigen::VectorXd z_sample = gmm_mu_[c_sel] + L * noise_z;

      // 赋值位置+姿态
      new_particles(0, i) = z_sample(0);
      new_particles(1, i) = z_sample(1);
      new_particles(2, i) = z_sample(2);
      new_particles(6, i) = wrapAngle(z_sample(3));
      new_particles(7, i) = wrapAngle(z_sample(4));
      new_particles(8, i) = wrapAngle(z_sample(5));

      // 速度维度：从当前粒子集的速度分布采样，保证物理连续性
      std::uniform_int_distribution<int> idx_dist(0, N_-1);
      int rand_idx = idx_dist(rng_);
      new_particles(3, i) = particles_(3, rand_idx) + normal(rng_) * std::sqrt(Q_(3, 3));
      new_particles(4, i) = particles_(4, rand_idx) + normal(rng_) * std::sqrt(Q_(4, 4));
      new_particles(5, i) = particles_(5, rand_idx) + normal(rng_) * std::sqrt(Q_(5, 5));
    }
    particles_ = new_particles;
    weights_.setConstant(N_, 1.0 / N_);
  }

  // === 论文公式(1)：状态预测 ===
  inline void predict() {
    std::normal_distribution<double> dist(0.0, 1.0);
    Eigen::LLT<Eigen::MatrixXd> llt(Q_);
    Eigen::MatrixXd L = llt.matrixL();

    for (int i = 0; i < N_; ++i) {
      Eigen::VectorXd x = particles_.col(i);
      // 论文公式(32)匀速模型状态转移
      x(0) += x(3) * dt_;
      x(1) += x(4) * dt_;
      x(2) += x(5) * dt_;
      // 添加过程噪声
      Eigen::VectorXd noise(nx_);
      for (int j = 0; j < nx_; ++j) noise(j) = dist(rng_);
      x += L * noise;
      // 归一化角度
      x(6) = wrapAngle(x(6));
      x(7) = wrapAngle(x(7));
      x(8) = wrapAngle(x(8));
      particles_.col(i) = x;
    }
  }

  // === 论文公式(9)：权重更新，仅有观测节点执行 ===
  inline void updateWeights(const Eigen::Vector3d& z_pos, const Eigen::Vector3d& z_rpy, bool has_obs) {
    if (!has_obs) return;
    Eigen::VectorXd z(nz_);
    z.head(3) = z_pos;
    z.tail(3) = z_rpy;
    Eigen::MatrixXd R_inv = R_.inverse();
    double R_det = R_.determinant();
    double norm_const = 1.0 / std::sqrt(std::pow(2.0 * M_PI, nz_) * R_det);

    for (int i = 0; i < N_; ++i) {
      Eigen::VectorXd z_pred = h(particles_.col(i));
      Eigen::VectorXd diff = obsDiff(z, z_pred);
      double exponent = -0.5 * diff.transpose() * R_inv * diff;
      double likelihood = norm_const * std::exp(exponent);
      weights_(i) *= likelihood;
    }
    // 归一化权重
    double sum_w = weights_.sum();
    if (sum_w > 1e-300) {
      weights_ /= sum_w;
    } else {
      weights_.setConstant(N_, 1.0 / N_);
    }
  }

  // === 论文Algorithm 1 Selection步骤：低方差重采样 ===
  inline void systematicResample() {
    // 速度合理性约束：降低异常速度粒子的权重
    double max_vel = 5.0;
    for (int i = 0; i < N_; ++i) {
      Eigen::Vector3d vel = particles_.col(i).segment(3, 3);
      if (vel.norm() > max_vel) {
        weights_(i) *= 0.1;
      }
    }
    // 重新归一化
    double sum_w = weights_.sum();
    if (sum_w > 1e-300) weights_ /= sum_w;
    else weights_.setConstant(N_, 1.0 / N_);

    // 低方差重采样
    std::uniform_real_distribution<double> uniform(0.0, 1.0 / N_);
    double r = uniform(rng_);
    Eigen::MatrixXd new_particles(nx_, N_);
    double c = weights_(0);
    int idx = 0;
    for (int i = 0; i < N_; ++i) {
      double u = r + (double)i / N_;
      while (u > c && idx < N_ - 1) {
        idx++;
        c += weights_(idx);
      }
      new_particles.col(i) = particles_.col(idx);
    }
    particles_ = new_particles;
    weights_.setConstant(N_, 1.0 / N_);
  }

  // === 论文Section V：两次观测之间的EM迭代 ===
  inline void emIterate(int drone_id, const std::vector<LocalStat>& neighbor_stats, const LocalStat& local_stat) {
    for (int iter = 0; iter < num_em_iters_; ++iter) {
      // 1. 共识滤波，估计全局统计量
      consensusFilter(neighbor_stats, local_stat);
      // 2. 全局M步，更新GMM
      globalMStep();
      // 3. 有观测节点：重新计算本地统计量，为下一轮迭代做准备
      if (local_stat.has_obs) {
        LocalStat new_ls = computeLocalStatsOnly(drone_id);
        const_cast<LocalStat&>(local_stat) = new_ls;
      }
    }
  }

  // === 数值有效性检查 ===
  inline bool isValid() const {
    for (int i = 0; i < N_; ++i) {
      for (int j = 0; j < nx_; ++j) {
        if (!std::isfinite(particles_(j, i))) return false;
      }
      if (!std::isfinite(weights_(i))) return false;
    }
    for (int c = 0; c < C_; ++c) {
      if (!std::isfinite(gmm_pi_(c))) return false;
      for (int j = 0; j < nz_; ++j) {
        if (!std::isfinite(gmm_mu_[c](j))) return false;
      }
      for (int i = 0; i < nz_; ++i) {
        for (int j = 0; j < nz_; ++j) {
          if (!std::isfinite(gmm_S_[c](i, j))) return false;
        }
      }
    }
    return true;
  }

  // === 状态估计输出 ===
  inline Eigen::Vector3d pos() {
    Eigen::Vector3d p = Eigen::Vector3d::Zero();
    for (int i = 0; i < N_; ++i) {
      p += weights_(i) * particles_.col(i).head(3);
    }
    pos_filtered_ = filter_alpha_ * p + (1.0 - filter_alpha_) * pos_filtered_;
    return pos_filtered_;
  }

  inline Eigen::Vector3d vel() {
    Eigen::Vector3d v = Eigen::Vector3d::Zero();
    for (int i = 0; i < N_; ++i) {
      v(0) += weights_(i) * particles_(3, i);
      v(1) += weights_(i) * particles_(4, i);
      v(2) += weights_(i) * particles_(5, i);
    }
    vel_filtered_ = filter_alpha_ * v + (1.0 - filter_alpha_) * vel_filtered_;
    return vel_filtered_;
  }

  inline Eigen::Vector3d rpy() {
    double sr = 0, cr = 0, sp = 0, cp = 0, sy = 0, cy = 0;
    for (int i = 0; i < N_; ++i) {
      double w = weights_(i);
      sr += w * std::sin(particles_(6, i));
      cr += w * std::cos(particles_(6, i));
      sp += w * std::sin(particles_(7, i));
      cp += w * std::cos(particles_(7, i));
      sy += w * std::sin(particles_(8, i));
      cy += w * std::cos(particles_(8, i));
    }
    Eigen::Vector3d rpy_out;
    rpy_out.x() = std::atan2(sr, cr);
    rpy_out.y() = std::atan2(sp, cp);
    rpy_out.z() = std::atan2(sy, cy);
    rpy_filtered_.x() = wrapAngle(filter_alpha_ * rpy_out.x() + (1.0 - filter_alpha_) * rpy_filtered_.x());
    rpy_filtered_.y() = wrapAngle(filter_alpha_ * rpy_out.y() + (1.0 - filter_alpha_) * rpy_filtered_.y());
    rpy_filtered_.z() = wrapAngle(filter_alpha_ * rpy_out.z() + (1.0 - filter_alpha_) * rpy_filtered_.z());
    return rpy_filtered_;
  }
};
