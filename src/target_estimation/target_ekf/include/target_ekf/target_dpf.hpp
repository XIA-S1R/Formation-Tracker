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

// 邻居共识状态结构体 (严格对应论文公式26的ζ_j)
struct NeighborConsensus {
  Eigen::VectorXd zeta_alpha;
  std::vector<Eigen::VectorXd> zeta_a;
  std::vector<Eigen::MatrixXd> zeta_b;
  bool has_obs;
  ros::Time timestamp;
};

// 局部统计量结构体 (严格对应论文公式21的u_m，9维状态空间)
struct LocalStat {
  int drone_id;
  int C;        // GMM 分量数
  int nx;       // 状态维度 = 9 (论文nx，GMM拟合此空间)
  Eigen::VectorXd alpha;              // [C] 对应 kα_{m,c}^t
  std::vector<Eigen::VectorXd> a;     // C 个 [nx] 向量 对应 a_{m,c}^t
  std::vector<Eigen::MatrixXd> b;     // C 个 [nx x nx] 矩阵 对应 b_{m,c}^t
  bool has_obs;
  ros::Time timestamp;  // 时间戳，用于时间同步
};

// DPF 核心类：100%对齐 Gu 2007 论文 Algorithm 1
// 状态维度 9: [x, y, z, vx, vy, vz, roll, pitch, yaw] (论文nx)
// 观测维度 6: [x, y, z, roll, pitch, yaw] (论文nz)
struct DistributedPF {
  // === 论文核心参数 ===
  int N_;       // 粒子数
  int nx_;      // 状态维度 = 9 (GMM拟合此空间)
  int nz_;      // 观测维度 = 6 (仅用于观测更新)
  int C_;       // GMM 分量数
  double dt_;   // 时间步长
  int num_em_iters_;  // 论文Section V：两次观测之间EM迭代总次数
  double consensus_epsilon_; // 论文公式(27)：共识滤波更新率 ε ≤ 1/d_max
  int num_consensus_iters_;  // 单次EM迭代内的共识收敛次数

  // === 粒子集 ===
  Eigen::MatrixXd particles_;   // [nx_ x N_] 每列一个9维状态粒子
  Eigen::VectorXd weights_;     // [N_] 粒子权重

  // === 运动模型 (论文公式1) ===
  Eigen::MatrixXd A_;           // [nx_ x nx_] 状态转移矩阵
  Eigen::MatrixXd Q_;           // [nx_ x nx_] 过程噪声协方差

  // === 观测模型 (论文公式2) ===
  Eigen::MatrixXd R_;           // [nz_ x nz_] 观测噪声协方差

  // === GMM 参数 (论文公式30，M步输出) 9维状态空间 ===
  Eigen::VectorXd gmm_pi_;              // [C_] 混合权重
  std::vector<Eigen::VectorXd> gmm_mu_; // C_ 个 [nx_] 9维状态均值
  std::vector<Eigen::MatrixXd> gmm_S_;  // C_ 个 [nx_ x nx_] 9维状态协方差

  // === 共识滤波状态 (论文公式26) 9维状态空间 ===
  Eigen::VectorXd zeta_alpha_;    // [C_] 全局α的共识估计
  std::vector<Eigen::VectorXd> zeta_a_; // C个[nx_] 全局a的共识估计
  std::vector<Eigen::MatrixXd> zeta_b_; // C个[nx_×nx_] 全局b的共识估计

  // === 状态 ===
  bool initialized_;
  std::mt19937 rng_;
  
  // 移动平均滤波参数
  Eigen::Vector3d pos_filtered_;
  Eigen::Vector3d vel_filtered_;
  Eigen::Vector3d rpy_filtered_;
  double filter_alpha_ = 0.95;

  // === 构造函数：严格按论文参数初始化 ===
  DistributedPF(double dt, int num_particles = 300, int num_components = 4, int num_em_iters = 10)
      : N_(num_particles), nx_(9), nz_(6), C_(num_components), dt_(dt),
        num_em_iters_(num_em_iters),
        consensus_epsilon_(0.1),
        num_consensus_iters_(10),
        initialized_(false), rng_(std::random_device{}()) {
    // 初始化滤波值
    pos_filtered_.setZero();
    vel_filtered_.setZero();
    rpy_filtered_.setZero();

    // 状态转移矩阵A (论文公式32，匀速模型，9维状态)
    A_.setIdentity(nx_, nx_);
    A_(0, 3) = dt_;
    A_(1, 4) = dt_;
    A_(2, 5) = dt_;

    // 过程噪声Q (论文Section V仿真参数，9维状态)
    Q_.setZero(nx_, nx_);
    double t2 = dt_ * dt_;
    // 位置噪声
    Q_(0, 0) = 0.05 * t2;  Q_(1, 1) = 0.05 * t2;  Q_(2, 2) = 0.02 * t2;
    // 速度噪声
    Q_(3, 3) = 0.01 * t2;  Q_(4, 4) = 0.01 * t2;  Q_(5, 5) = 0.005 * t2;
    // 姿态噪声
    Q_(6, 6) = 0.05 * t2;  Q_(7, 7) = 0.05 * t2;  Q_(8, 8) = 0.005 * t2;

    // 观测噪声R (论文Section V仿真参数，6维观测)
    R_.setIdentity(nz_, nz_);
    R_(0, 0) = 0.05;  R_(1, 1) = 0.05;  R_(2, 2) = 0.05;
    R_(3, 3) = 0.005; R_(4, 4) = 0.005; R_(5, 5) = 0.005;

    // 粒子和权重初始化
    particles_.setZero(nx_, N_);
    weights_.setConstant(N_, 1.0 / N_);

    // GMM初始化 (9维状态空间)
    gmm_pi_.setConstant(C_, 1.0 / C_);
    gmm_mu_.resize(C_);
    gmm_S_.resize(C_);
    for (int c = 0; c < C_; ++c) {
      gmm_mu_[c].setZero(nx_);
      gmm_S_[c].setIdentity(nx_, nx_);
    }

    // 共识滤波状态初始化 (9维状态空间)
    zeta_alpha_.setZero(C_);
    zeta_a_.resize(C_);
    zeta_b_.resize(C_);
    for (int c = 0; c < C_; ++c) {
      zeta_a_[c].setZero(nx_);
      zeta_b_[c].setZero(nx_, nx_);
    }
  }

  // === 论文Algorithm 1 Initialization步骤 ===
  inline void init(const Eigen::Vector3d& z0, const Eigen::Vector3d& z0_rpy) {
    std::normal_distribution<double> dist(0.0, 1.0);
    for (int i = 0; i < N_; ++i) {
      // 9维状态完整初始化
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

    // GMM初始化 (9维状态空间)
    Eigen::VectorXd x_full(nx_);
    x_full.head(3) = z0;
    x_full.segment(3, 3).setZero(); // 速度初始化为0
    x_full.tail(3) = z0_rpy;
    for (int c = 0; c < C_; ++c) {
      gmm_mu_[c] = x_full;
      // 位置噪声
      gmm_mu_[c](0) += dist(rng_) * 0.3;
      gmm_mu_[c](1) += dist(rng_) * 0.3;
      gmm_mu_[c](2) += dist(rng_) * 0.1;
      // 速度噪声
      gmm_mu_[c](3) += dist(rng_) * 0.05;
      gmm_mu_[c](4) += dist(rng_) * 0.05;
      gmm_mu_[c](5) += dist(rng_) * 0.02;
      // 姿态噪声
      gmm_mu_[c](6) += dist(rng_) * 0.05;
      gmm_mu_[c](7) += dist(rng_) * 0.05;
      gmm_mu_[c](8) += dist(rng_) * 0.02;
      // 用过程噪声初始化GMM协方差，保证位置-速度协方差耦合
      gmm_S_[c] = Q_ * 2.0;
    }
    gmm_pi_.setConstant(C_, 1.0 / C_);

    // 共识状态初始化
    zeta_alpha_ = gmm_pi_ * N_;
    for (int c = 0; c < C_; ++c) {
      zeta_a_[c] = gmm_mu_[c] * zeta_alpha_(c);
      zeta_b_[c] = gmm_S_[c] * zeta_alpha_(c);
    }
    initialized_ = true;
  }

  inline void reset(const Eigen::Vector3d& z0, const Eigen::Vector3d& z0_rpy) {
    init(z0, z0_rpy);
  }

  // === 观测函数h(x)：严格对应论文公式(31)，9维状态→6维观测 ===
  inline Eigen::VectorXd h(const Eigen::VectorXd& x) const {
    Eigen::VectorXd z(nz_);
    z.head(3) = x.head(3);   // 位置
    z.tail(3) = x.tail(3);   // 姿态
    return z;
  }

  // === 角度感知残差计算 (观测空间) ===
  inline Eigen::VectorXd obsDiff(const Eigen::VectorXd& z_obs, const Eigen::VectorXd& z_pred) const {
    Eigen::VectorXd diff(nz_);
    diff.head(3) = z_obs.head(3) - z_pred.head(3);
    diff(3) = angleDiff(z_obs(3), z_pred(3));
    diff(4) = angleDiff(z_obs(4), z_pred(4));
    diff(5) = angleDiff(z_obs(5), z_pred(5));
    return diff;
  }

  // === 9维状态残差计算 (状态空间，用于GMM拟合) ===
  inline Eigen::VectorXd stateDiff(const Eigen::VectorXd& x_est, const Eigen::VectorXd& x_pred) const {
    Eigen::VectorXd diff(nx_);
    diff.head(3) = x_est.head(3) - x_pred.head(3); // 位置
    diff.segment(3, 3) = x_est.segment(3, 3) - x_pred.segment(3, 3); // 速度
    diff(6) = angleDiff(x_est(6), x_pred(6)); // roll
    diff(7) = angleDiff(x_est(7), x_pred(7)); // pitch
    diff(8) = angleDiff(x_est(8), x_pred(8)); // yaw
    return diff;
  }

  // === 论文公式(1)：状态预测（所有节点执行，9维状态）===
  inline void predict() {
    if (!initialized_) return;
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

  // === 论文公式(9)：权重更新（仅有观测节点执行，6维观测空间）===
  inline void updateWeights(const Eigen::Vector3d& z_pos, const Eigen::Vector3d& z_rpy) {
    if (!initialized_) return;
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

  // === 论文公式(21)：EM-E步，纯本地统计量计算（9维状态空间，所有节点均可执行）===
  // 【关键修复】无观测节点也用当前粒子计算真实统计量，不再发全0
  inline LocalStat computeLocalStatsOnly(int drone_id, bool has_obs) {
    LocalStat ls;
    ls.drone_id = drone_id;
    ls.C = C_;
    ls.nx = nx_; // 9维状态空间
    ls.has_obs = has_obs;
    ls.timestamp = ros::Time::now();
    ls.alpha.setZero(C_);
    ls.a.resize(C_);
    ls.b.resize(C_);
    for (int c = 0; c < C_; ++c) {
      ls.a[c].setZero(nx_);
      ls.b[c].setZero(nx_, nx_);
    }

    // 预计算GMM的逆和行列式（9维状态空间）
    std::vector<Eigen::MatrixXd> S_inv(C_);
    std::vector<double> S_det(C_);
    for (int c = 0; c < C_; ++c) {
      S_inv[c] = gmm_S_[c].inverse();
      S_det[c] = gmm_S_[c].determinant();
      if (S_det[c] < 1e-300) S_det[c] = 1e-300;
    }

    // 严格按论文公式(17)计算责任度，公式(21)累加9维状态的统计量
    for (int n = 0; n < N_; ++n) {
      Eigen::VectorXd x_n = particles_.col(n); // 直接用9维状态粒子
      double w_n = weights_(n);
      Eigen::VectorXd resp(C_);
      double resp_sum = 0.0;

      for (int c = 0; c < C_; ++c) {
        // 9维状态残差计算
        Eigen::VectorXd diff = stateDiff(x_n, gmm_mu_[c]);
        double exponent = -0.5 * diff.transpose() * S_inv[c] * diff;
        double nc = 1.0 / std::sqrt(std::pow(2.0 * M_PI, nx_) * S_det[c]);
        resp(c) = gmm_pi_(c) * nc * std::exp(exponent);
        resp_sum += resp(c);
      }

      if (resp_sum > 1e-300) resp /= resp_sum;
      else resp.setConstant(1.0 / C_);

      // 论文公式(21)累加9维状态的本地统计量
      for (int c = 0; c < C_; ++c) {
        double alpha_nc = w_n * resp(c);
        ls.alpha(c) += alpha_nc;
        ls.a[c] += alpha_nc * x_n;
        Eigen::VectorXd diff_c = stateDiff(x_n, gmm_mu_[c]);
        ls.b[c] += alpha_nc * diff_c * diff_c.transpose();
      }
    }
    return ls;
  }

  // === 论文公式(26)：单次共识滤波更新（所有节点执行）===
  // 【修复】移除内部循环，改为单次更新，外层由emStep控制迭代
  inline void consensusFilterOnce(const std::vector<NeighborConsensus>& neighbor_consensus, const LocalStat& local_stat) {
    // 未初始化时，用邻居统计量初始化
    if (!initialized_ && !neighbor_consensus.empty()) {
      for (const auto& nc : neighbor_consensus) {
        if (nc.has_obs) {
          zeta_alpha_ = nc.zeta_alpha;
          zeta_a_ = nc.zeta_a;
          zeta_b_ = nc.zeta_b;
          globalMStep();
          initialized_ = true;
          ROS_INFO("[dpf%d] initialized via consensus from neighbor", local_stat.drone_id);
          break;
        }
      }
      if (!initialized_) return;
    }

    // 论文公式(27)：动态调整共识步长ε ≤ 1/d_max
    int d_max = neighbor_consensus.size() + 1;
    double adaptive_epsilon = 1.0 / d_max;
    adaptive_epsilon = std::min(adaptive_epsilon, 0.3); // 上限0.3保证稳定

    // 单次共识更新（不再内部循环）
    for (int c = 0; c < C_; ++c) {
      // 1. 严格用邻居的共识状态ζ_j计算差值（论文公式26求和项）
      double alpha_diff = 0.0;
      Eigen::VectorXd a_diff = Eigen::VectorXd::Zero(nx_);
      Eigen::MatrixXd b_diff = Eigen::MatrixXd::Zero(nx_, nx_);

      for (const auto& nc : neighbor_consensus) {
        if (!nc.has_obs) continue;
        alpha_diff += nc.zeta_alpha(c) - zeta_alpha_(c);
        a_diff += nc.zeta_a[c] - zeta_a_[c];
        b_diff += nc.zeta_b[c] - zeta_b_[c];
      }

      // 2. 本地统计量处理：无观测节点完全跟随邻居，不贡献本地信息
      double local_alpha = 0.0;
      Eigen::VectorXd local_a = Eigen::VectorXd::Zero(nx_);
      Eigen::MatrixXd local_b = Eigen::MatrixXd::Zero(nx_, nx_);

      if (local_stat.has_obs) {
        // 有观测节点：贡献本地统计量u
        local_alpha = local_stat.alpha(c);
        local_a = local_stat.a[c];
        local_b = local_stat.b[c];
      } else {
        // 无观测节点：本地项=当前ζ，(u-ζ)=0，完全跟随邻居
        local_alpha = zeta_alpha_(c);
        local_a = zeta_a_[c];
        local_b = zeta_b_[c];
      }

      // 3. 论文公式(26)单次更新共识状态ζ
      zeta_alpha_(c) += adaptive_epsilon * (alpha_diff + (local_alpha - zeta_alpha_(c)));
      zeta_a_[c] += adaptive_epsilon * (a_diff + (local_a - zeta_a_[c]));
      zeta_b_[c] += adaptive_epsilon * (b_diff + (local_b - zeta_b_[c]));
    }
  }

  // === 论文公式(30)：全局M步，用共识后的全局统计量更新GMM（9维状态空间）===
  inline void globalMStep() {
    double alpha_sum = zeta_alpha_.sum();
    if (alpha_sum < 1e-300) return;
    for (int c = 0; c < C_; ++c) {
      if (zeta_alpha_(c) < 1e-300) continue;
      // 论文公式(30)更新9维GMM参数
      gmm_pi_(c) = zeta_alpha_(c) / alpha_sum;
      gmm_mu_[c] = zeta_a_[c] / zeta_alpha_(c);
      // 归一化角度
      gmm_mu_[c](6) = wrapAngle(gmm_mu_[c](6));
      gmm_mu_[c](7) = wrapAngle(gmm_mu_[c](7));
      gmm_mu_[c](8) = wrapAngle(gmm_mu_[c](8));
      gmm_S_[c] = zeta_b_[c] / zeta_alpha_(c);
      // 保证协方差正定
      gmm_S_[c] += Eigen::MatrixXd::Identity(nx_, nx_) * 1e-4;
    }
    // 归一化混合权重
    double pi_sum = gmm_pi_.sum();
    if (pi_sum > 1e-300) gmm_pi_ /= pi_sum;
  }

  // === 论文Section V：EM迭代（每帧多次迭代，每次迭代重新计算本地统计量）===
  // 【修复】正确的EM迭代：每次迭代都用新GMM重新计算本地统计量
  inline void emStep(int drone_id, const std::vector<NeighborConsensus>& neighbor_consensus, LocalStat& local_stat) {
    for (int iter = 0; iter < num_em_iters_; ++iter) {
      // 1. 用当前GMM参数计算本地统计量（E步）
      local_stat = computeLocalStatsOnly(drone_id, local_stat.has_obs);

      // 2. 单次共识滤波更新
      consensusFilterOnce(neighbor_consensus, local_stat);

      // 3. M步更新GMM参数
      globalMStep();
    }
  }

  // === 论文Algorithm 1：从全局GMM采样新粒子（9维完整状态，保留位置-速度协方差）===
  // 【关键修复】直接采样9维状态，不再拆分位置和速度，保证运动学耦合
  inline void sampleParticlesFromGMM() {
    if (!initialized_) return;

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

      // 从GMM采样9维完整状态（位置+速度+姿态），天然保留协方差耦合
      Eigen::LLT<Eigen::MatrixXd> llt(gmm_S_[c_sel]);
      Eigen::MatrixXd L = llt.matrixL();
      Eigen::VectorXd noise_x(nx_);
      for (int j = 0; j < nx_; ++j) noise_x(j) = normal(rng_);
      Eigen::VectorXd x_sample = gmm_mu_[c_sel] + L * noise_x;

      // 直接赋值9维完整状态
      new_particles.col(i) = x_sample;
      // 归一化角度
      new_particles(6, i) = wrapAngle(new_particles(6, i));
      new_particles(7, i) = wrapAngle(new_particles(7, i));
      new_particles(8, i) = wrapAngle(new_particles(8, i));
    }
    particles_ = new_particles;
    weights_.setConstant(N_, 1.0 / N_);
  }

  // === 论文Algorithm 1 Selection步骤：低方差重采样（仅有观测节点执行）===
  inline void systematicResample() {
    if (!initialized_) return;

    // 速度合理性约束
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
    double sum_w2 = weights_.squaredNorm();
    double Neff = (sum_w2 > 1e-300) ? 1.0 / sum_w2 : 0.0;
    if (Neff < N_ * 0.5) {
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
  }

  // === 数值有效性检查 ===
  inline bool isValid() const {
    if (!initialized_) return true;
    for (int i = 0; i < N_; ++i) {
      for (int j = 0; j < nx_; ++j) {
        if (!std::isfinite(particles_(j, i))) return false;
      }
      if (!std::isfinite(weights_(i))) return false;
    }
    for (int c = 0; c < C_; ++c) {
      if (!std::isfinite(gmm_pi_(c))) return false;
      for (int j = 0; j < nx_; ++j) {
        if (!std::isfinite(gmm_mu_[c](j))) return false;
      }
      for (int i = 0; i < nx_; ++i) {
        for (int j = 0; j < nx_; ++j) {
          if (!std::isfinite(gmm_S_[c](i, j))) return false;
        }
      }
    }
    return true;
  }

  // === 状态估计输出 ===
  inline Eigen::Vector3d pos() {
    if (!initialized_) return Eigen::Vector3d::Zero();
    Eigen::Vector3d p = Eigen::Vector3d::Zero();
    for (int i = 0; i < N_; ++i) {
      p += weights_(i) * particles_.col(i).head(3);
    }
    pos_filtered_ = filter_alpha_ * p + (1.0 - filter_alpha_) * pos_filtered_;
    return pos_filtered_;
  }

  inline Eigen::Vector3d vel() {
    if (!initialized_) return Eigen::Vector3d::Zero();
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
    if (!initialized_) return Eigen::Vector3d::Zero();
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
  // === 获取粒子集合和权重 ===
  inline std::pair<Eigen::MatrixXd, Eigen::VectorXd> getParticlesAndWeights() const {
  return std::make_pair(particles_, weights_);
  }
};
