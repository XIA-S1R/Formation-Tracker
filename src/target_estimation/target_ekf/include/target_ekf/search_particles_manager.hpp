#pragma once
#include <Eigen/Dense>
#include <vector>
#include <random>
#include <functional>
#include <memory>
#include <algorithm>
#include <cmath>
#include <limits>
#include <numeric>
#include <ros/time.h>
#include <ros/node_handle.h>
#include <map>
#include <array>

// 【修复1】删除重复定义的wrapAngle和angleDiff，直接使用target_dpf.hpp中的版本

// 搜索意图枚举
enum SearchIntent {
  STRAIGHT = 0,   // 直行
  LEFT_TURN = 1,  // 左转
  RIGHT_TURN = 2  // 右转
};

// 搜索粒子结构体
struct SearchParticle {
  Eigen::VectorXd state;  // 9维状态 [x, y, z, vx, vy, vz, roll, pitch, yaw]
  double weight;
  SearchIntent intent;
  int particle_id;
};

// 搜索粒子管理类
class SearchParticlesManager {
public:
  // 构造函数
  SearchParticlesManager(int drone_id = 0, int num_components = 4, ros::NodeHandle* nh = nullptr)
    : drone_id_(drone_id),
      num_components_(num_components),
      search_particles_initialized_(false),
      search_dt_(0.2),  // 搜索模式更新周期，5Hz
      search_vmax_(2),        // 粒子扩展速度上限（默认值，后续可由 /target/planning/vmax 覆盖）
      search_vmin_(0.5),        // 最小速度 m/s
      search_amax_(4),        // 最大加速度 m/s² (默认值)
      intent_keep_prob_(0.8),   // 意图保持概率 80%
      search_pos_noise_(0.1),   // 位置噪声 m
      search_vel_noise_(0.2),   // 速度噪声 m/s
      last_yaw_(0.0),
      cam_fx_(320.0), cam_fy_(320.0), cam_cx_(320.0), cam_cy_(240.0),
      cam_width_(640.0), cam_height_(480.0), cam_max_range_(5.0) {
    // 从参数服务器读取动力学参数
    if (nh) {
      nh->param("/target/planning/vmax", search_vmax_, search_vmax_);
      nh->param("/target/planning/amax", search_amax_, search_amax_);
      // 直接读取相机参数到成员变量
      nh->param("/target/camera/fx", cam_fx_, cam_fx_);
      nh->param("/target/camera/fy", cam_fy_, cam_fy_);
      nh->param("/target/camera/cx", cam_cx_, cam_cx_);
      nh->param("/target/camera/cy", cam_cy_, cam_cy_);
      nh->param("/target/camera/width", cam_width_, cam_width_);
      nh->param("/target/camera/height", cam_height_, cam_height_);
      nh->param("/target/camera/max_range", cam_max_range_, cam_max_range_);
    }
    
    rng_.seed(std::random_device{}());
    
    // 初始化标签化共识状态和GMM参数
    initializeLabeledConsensusStates();
  }
  
  // 初始化标签化共识状态和GMM参数
  void initializeLabeledConsensusStates() {
    for (int label_int = STRAIGHT; label_int <= RIGHT_TURN; ++label_int) {
      SearchIntent label = static_cast<SearchIntent>(label_int);
      
      // 初始化共识状态
      zeta_alpha_[label].setZero(num_components_);
      zeta_a_[label].resize(num_components_);
      zeta_b_[label].resize(num_components_);
      
      // 初始化GMM参数
      gmm_pi_[label].setConstant(num_components_, 1.0 / num_components_);
      gmm_mu_[label].resize(num_components_);
      gmm_S_[label].resize(num_components_);
      
      for (int c = 0; c < num_components_; ++c) {
        zeta_a_[label][c].setZero(9);  // 9维状态
        zeta_b_[label][c].setZero(9, 9);
        gmm_mu_[label][c].setZero(9);
        gmm_S_[label][c].setIdentity(9, 9);
      }
    }
  }

  // 从DPF粒子继承并分配意图标签
  template<typename DPFType>
  void inheritAndLabelParticles(DPFType* dpfPtr) {
    if (!dpfPtr || !dpfPtr->initialized_) {
      ROS_WARN("[sp_mgr%d] Cannot inherit particles: DPF not initialized", drone_id_);
      return;
    }
    
    // 获取当前粒子和权重
    auto particles_and_weights = dpfPtr->getParticlesAndWeights();
    const Eigen::MatrixXd& dpf_particles = particles_and_weights.first;
    const Eigen::VectorXd& dpf_weights = particles_and_weights.second;
    
    int num_particles = dpf_particles.cols();
    
    // 清空之前的搜索粒子
    search_particles_.clear();
    search_particles_.reserve(num_particles);
    
    // 继承粒子
    for (int i = 0; i < num_particles; ++i) {
      SearchParticle particle;
      particle.state = dpf_particles.col(i);
      particle.weight = dpf_weights(i);
      particle.particle_id = i;
      particle.intent = STRAIGHT;  // 默认值，后面会重新分配
      search_particles_.push_back(particle);
    }
    
    ROS_INFO("[sp_mgr%d] Inherited %d particles from DPF", drone_id_, num_particles);
    
    // 按权重分层，每层均匀分配三个意图，保证高权重粒子均匀分布
    std::vector<size_t> sorted_indices(num_particles);
    std::iota(sorted_indices.begin(), sorted_indices.end(), 0);
    
    std::sort(sorted_indices.begin(), sorted_indices.end(), 
              [&] (size_t a, size_t b) {
                return search_particles_[a].weight > search_particles_[b].weight;
              });
    
    // 意图分配比例: 40% 直行, 30% 左转, 30% 右转
    int straight_count = static_cast<int>(num_particles * 0.4);
    int left_turn_count = static_cast<int>(num_particles * 0.3);
    int right_turn_count = num_particles - straight_count - left_turn_count;
    
    // 轮询分配，保证高权重粒子均匀分配到三个意图
    int s_idx = 0, l_idx = 0, r_idx = 0;
    for (int i = 0; i < num_particles; ++i) {
      size_t particle_idx = sorted_indices[i];
      if (i % 10 < 4 && s_idx < straight_count) {
        search_particles_[particle_idx].intent = STRAIGHT;
        s_idx++;
      } else if (i % 10 < 7 && l_idx < left_turn_count) {
        search_particles_[particle_idx].intent = LEFT_TURN;
        l_idx++;
      } else if (r_idx < right_turn_count) {
        search_particles_[particle_idx].intent = RIGHT_TURN;
        r_idx++;
      } else {
        search_particles_[particle_idx].intent = STRAIGHT;
      }
    }
    
    // 统计分配结果
    int straight_assigned = 0, left_assigned = 0, right_assigned = 0;
    for (const auto& p : search_particles_) {
      switch(p.intent) {
        case STRAIGHT: straight_assigned++; break;
        case LEFT_TURN: left_assigned++; break;
        case RIGHT_TURN: right_assigned++; break;
      }
    }
    ROS_INFO("[sp_mgr%d] Intent distribution - Straight: %d, Left: %d, Right: %d", 
             drone_id_, straight_assigned, left_assigned, right_assigned);
    
    // 用继承的粒子初始化每个标签的GMM参数
    initializeLabeledGMMFromParticles();
    search_particles_initialized_ = true;
  }

  // 用继承的粒子初始化每个标签的GMM参数
  void initializeLabeledGMMFromParticles() {
    int C = num_components_;
    int nx = 9;
    std::map<SearchIntent, std::vector<size_t>> label_to_indices;
    
    for (size_t i = 0; i < search_particles_.size(); ++i) {
      label_to_indices[search_particles_[i].intent].push_back(i);
    }

    for (int label_int = STRAIGHT; label_int <= RIGHT_TURN; ++label_int) {
      SearchIntent label = static_cast<SearchIntent>(label_int);
      auto it = label_to_indices.find(label);
      if (it == label_to_indices.end() || it->second.empty()) continue;

      // 计算该标签粒子的加权均值和协方差
      Eigen::VectorXd mean_state = Eigen::VectorXd::Zero(nx);
      double total_weight = 0.0;

      for (size_t idx : it->second) {
        const auto& p = search_particles_[idx];
        mean_state += p.weight * p.state;
        total_weight += p.weight;
      }
      if (total_weight > 1e-300) mean_state /= total_weight;

      // 初始化GMM中心，围绕均值分散
      std::normal_distribution<double> dist(0.0, 0.3);
      for (int c = 0; c < C; ++c) {
        gmm_mu_[label][c] = mean_state;
        for (int j = 0; j < nx; ++j) {
          gmm_mu_[label][c](j) += dist(rng_);
        }
        gmm_S_[label][c] = Eigen::MatrixXd::Identity(nx, nx) * 0.5;
      }

      // 初始化共识状态
      zeta_alpha_[label].setConstant(C, total_weight / C);
      for (int c = 0; c < C; ++c) {
        zeta_a_[label][c] = gmm_mu_[label][c] * zeta_alpha_[label](c);
        zeta_b_[label][c] = gmm_S_[label][c] * zeta_alpha_[label](c);
      }
    }
  }

  // 标签切换：高概率保持原标签，低概率切换
  void switchSearchIntents() {
    if (!search_particles_initialized_ || search_particles_.empty()) {
      return;
    }
    
    std::uniform_real_distribution<double> uniform(0.0, 1.0);
    const double keep_prob = intent_keep_prob_; // 0.8
    const double switch_prob = (1.0 - keep_prob) / 2.0; // 每个切换方向0.1
    
    for (auto& particle : search_particles_) {
      double rand_val = uniform(rng_);
      SearchIntent old_intent = particle.intent;
      SearchIntent new_intent = old_intent;

      // 严格按80%保持，10%切换到另外两个标签
      if (rand_val < keep_prob) {
        continue;
      } else if (rand_val < keep_prob + switch_prob) {
        if (old_intent == STRAIGHT) new_intent = LEFT_TURN;
        else if (old_intent == LEFT_TURN) new_intent = STRAIGHT;
        else new_intent = STRAIGHT;
      } else {
        if (old_intent == STRAIGHT) new_intent = RIGHT_TURN;
        else if (old_intent == LEFT_TURN) new_intent = RIGHT_TURN;
        else new_intent = LEFT_TURN;
      }
      
      particle.intent = new_intent;
    }
  }

  // 【修复3】手动实现clamp函数，兼容C++11
  template<typename T>
  T clamp(const T& val, const T& min_val, const T& max_val) const {
    return std::max(min_val, std::min(val, max_val));
  }

  inline bool inSearchXYBounds(double x, double y) const {
    if (!search_bounds_enabled_) return true;
    return (x >= search_x_min_ && x <= search_x_max_ &&
            y >= search_y_min_ && y <= search_y_max_);
  }

  // 若第k帧更新后越界，则回退到k-1位置并重定向（优先±90°）重新生成k帧粒子。
  // 仅当候选方向都不可行时退化到贴边。
  inline void enforceBoundsForMatrixParticle(Eigen::MatrixXd& particles, int i) const {
    if (!search_bounds_enabled_) return;
    if (particles.rows() < 5 || i < 0 || i >= particles.cols()) return;
    if (inSearchXYBounds(particles(0, i), particles(1, i))) return;

    const double dt = std::max(1e-6, search_dt_);
    const double vx_k = particles(3, i);
    const double vy_k = particles(4, i);
    const double speed = std::hypot(vx_k, vy_k);
    const double x_prev = particles(0, i) - vx_k * dt;
    const double y_prev = particles(1, i) - vy_k * dt;

    if (speed > 1e-6) {
      const double theta0 = std::atan2(vy_k, vx_k);
      const std::array<double, 6> candidates = {
          M_PI / 2.0, -M_PI / 2.0, 3.0 * M_PI / 4.0,
          -3.0 * M_PI / 4.0, M_PI, 0.0};
      for (double dtheta : candidates) {
        const double th = theta0 + dtheta;
        const double vx_new = speed * std::cos(th);
        const double vy_new = speed * std::sin(th);
        const double x_new = x_prev + vx_new * dt;
        const double y_new = y_prev + vy_new * dt;
        if (inSearchXYBounds(x_new, y_new)) {
          particles(0, i) = x_new;
          particles(1, i) = y_new;
          particles(3, i) = vx_new;
          particles(4, i) = vy_new;
          return;
        }
      }
    }

    // 候选方向均不可行时，退化到贴边并尽量保持速度连续。
    const double x_clip = clamp(particles(0, i), search_x_min_, search_x_max_);
    const double y_clip = clamp(particles(1, i), search_y_min_, search_y_max_);
    particles(0, i) = x_clip;
    particles(1, i) = y_clip;
    particles(3, i) = (x_clip - x_prev) / dt;
    particles(4, i) = (y_clip - y_prev) / dt;
  }

  inline void enforceBoundsForSearchParticle(SearchParticle& particle) const {
    if (!search_bounds_enabled_ || particle.state.size() < 5) return;
    if (inSearchXYBounds(particle.state(0), particle.state(1))) return;

    const double dt = std::max(1e-6, search_dt_);
    const double vx_k = particle.state(3);
    const double vy_k = particle.state(4);
    const double speed = std::hypot(vx_k, vy_k);
    const double x_prev = particle.state(0) - vx_k * dt;
    const double y_prev = particle.state(1) - vy_k * dt;

    if (speed > 1e-6) {
      const double theta0 = std::atan2(vy_k, vx_k);
      const std::array<double, 6> candidates = {
          M_PI / 2.0, -M_PI / 2.0, 3.0 * M_PI / 4.0,
          -3.0 * M_PI / 4.0, M_PI, 0.0};
      for (double dtheta : candidates) {
        const double th = theta0 + dtheta;
        const double vx_new = speed * std::cos(th);
        const double vy_new = speed * std::sin(th);
        const double x_new = x_prev + vx_new * dt;
        const double y_new = y_prev + vy_new * dt;
        if (inSearchXYBounds(x_new, y_new)) {
          particle.state(0) = x_new;
          particle.state(1) = y_new;
          particle.state(3) = vx_new;
          particle.state(4) = vy_new;
          return;
        }
      }
    }

    const double x_clip = clamp(particle.state(0), search_x_min_, search_x_max_);
    const double y_clip = clamp(particle.state(1), search_y_min_, search_y_max_);
    particle.state(0) = x_clip;
    particle.state(1) = y_clip;
    particle.state(3) = (x_clip - x_prev) / dt;
    particle.state(4) = (y_clip - y_prev) / dt;
  }

  // 更新单个粒子的状态
  void updateSearchParticleState(SearchParticle& particle) {
    std::normal_distribution<double> normal(0.0, 1.0);
    std::uniform_real_distribution<double> uniform(0.0, 1.0);

    Eigen::Vector3d pos = particle.state.segment(0, 3);
    Eigen::Vector3d vel = particle.state.segment(3, 3);
    double current_yaw = particle.state(8); // 直接用粒子的yaw，而非速度方向
    double v = vel.head(2).norm(); // 仅水平速度

    // 速度方向单位矢量
    Eigen::Vector3d ev = Eigen::Vector3d::Zero();
    if (v > search_vmin_) {
      ev.head(2) = vel.head(2).normalized();
    } else {
      ev.x() = std::cos(current_yaw);
      ev.y() = std::sin(current_yaw);
    }

    // 加速度计算
    Eigen::Vector3d acc = Eigen::Vector3d::Zero();
    std::normal_distribution<double> acc_noise(0.0, 0.1);

    switch(particle.intent) {
      case STRAIGHT: {
        // 直行：主要沿速度方向加速，偏向正加速度以维持前进
        // 使用偏向正值的分布：均值为 amax/2，确保平均加速度为正
        std::normal_distribution<double> acc_dist(search_amax_ * 0.3, search_amax_ * 0.3);
        double amag = acc_dist(rng_) + acc_noise(rng_);
        amag = clamp(amag, -search_amax_ * 0.3, search_amax_);
        // 速度上限约束
        if (v + amag * search_dt_ > search_vmax_) amag = (search_vmax_ - v) / search_dt_;
        // 速度下限约束：如果速度太低，强制加速
        if (v < search_vmin_ * 1.5) amag = std::max(amag, search_amax_ * 0.5);
        acc.head(2) = amag * ev.head(2);
        break;
      }
      case LEFT_TURN: {
        // 左转：横向加速度 + 正向前进加速度
        Eigen::Vector3d eleft = Eigen::Vector3d::Zero();
        eleft.x() = -ev.y();
        eleft.y() = ev.x();

        // 横向加速度（向左）
        std::uniform_real_distribution<double> uniform(0.0, 1.0);
        double u = uniform(rng_);
        double aturn = search_amax_ * 0.7 * std::pow(u, 0.5);  // 偏向较大值
        aturn = clamp(aturn + acc_noise(rng_), 0.0, search_amax_ * 0.8);

        // 前向加速度：保持正值以维持速度
        double akeep = search_amax_ * 0.3;  // 固定正向加速度
        if (v > search_vmax_ * 0.8) akeep = 0;  // 速度够快时不再加速
        if (v < search_vmin_ * 1.5) akeep = search_amax_ * 0.5;  // 速度太慢时加速

        acc.head(2) = aturn * eleft.head(2) + akeep * ev.head(2);
        if (acc.norm() > search_amax_) acc = acc.normalized() * search_amax_;
        break;
      }
      case RIGHT_TURN: {
        // 右转：横向加速度 + 正向前进加速度
        Eigen::Vector3d eright = Eigen::Vector3d::Zero();
        eright.x() = ev.y();
        eright.y() = -ev.x();

        // 横向加速度（向右）
        std::uniform_real_distribution<double> uniform(0.0, 1.0);
        double u = uniform(rng_);
        double aturn = search_amax_ * 0.7 * std::pow(u, 0.5);
        aturn = clamp(aturn + acc_noise(rng_), 0.0, search_amax_ * 0.8);

        // 前向加速度：保持正值以维持速度
        double akeep = search_amax_ * 0.3;
        if (v > search_vmax_ * 0.8) akeep = 0;
        if (v < search_vmin_ * 1.5) akeep = search_amax_ * 0.5;

        acc.head(2) = aturn * eright.head(2) + akeep * ev.head(2);
        if (acc.norm() > search_amax_) acc = acc.normalized() * search_amax_;
        break;
      }
    }

    // 保留z轴状态，仅添加小噪声，不强制置0
    Eigen::Vector3d vel_noise(
      search_vel_noise_ * normal(rng_),
      search_vel_noise_ * normal(rng_),
      search_vel_noise_ * 0.2 * normal(rng_) // z轴噪声更小
    );
    Eigen::Vector3d pos_noise(
      search_pos_noise_ * normal(rng_),
      search_pos_noise_ * normal(rng_),
      search_pos_noise_ * 0.2 * normal(rng_)
    );

    // 更新速度和位置
    vel = vel + acc * search_dt_ + vel_noise;
    // 水平速度上限约束
    double v_horiz = vel.head(2).norm();
    if (v_horiz > search_vmax_) vel.head(2) = vel.head(2).normalized() * search_vmax_;
    // z轴速度限制
    vel.z() = clamp(vel.z(), -1.0, 1.0);

    pos = pos + vel * search_dt_ + pos_noise;

    // 更新粒子状态
    particle.state.segment(0, 3) = pos;
    particle.state.segment(3, 3) = vel;
    enforceBoundsForSearchParticle(particle);
    // 更新yaw：与速度方向一致
    if (v_horiz > search_vmin_) {
      particle.state(8) = std::atan2(vel.y(), vel.x());
    }
    // 角度归一化（使用target_dpf.hpp中的wrapAngle）
    particle.state(6) = wrapAngle(particle.state(6));
    particle.state(7) = wrapAngle(particle.state(7));
    particle.state(8) = wrapAngle(particle.state(8));
  }

  // 更新所有搜索粒子的状态
  void updateAllSearchParticles() {
    if (!search_particles_initialized_ || search_particles_.empty()) {
      return;
    }
    
    // 步骤1：先切换意图标签
    switchSearchIntents();
    
    // 步骤2：更新所有粒子状态（意图驱动的运动学预测）
    for (auto& particle : search_particles_) {
      updateSearchParticleState(particle);
    }
    
    // 步骤3：检查有效粒子数，仅当n_eff过低时执行重采样
    double n_eff = computeEffectiveSampleSize();
    double threshold = 0.5 * search_particles_.size();
    
    if (n_eff < threshold) {
      sampleParticlesFromGMM();
      ROS_INFO_THROTTLE(1.0, "[sp_mgr%d] GMM resampling triggered: n_eff=%.2f, threshold=%.2f", 
                      drone_id_, n_eff, threshold);
    }
    
    ROS_DEBUG("[sp_mgr%d] Updated %zu search particles, n_eff=%.2f", 
              drone_id_, search_particles_.size(), n_eff);
  }

  // 搜索粒子负观测权重更新函数
  void updateSearchParticlesWithNegativeObservation(const Eigen::Vector3d& cam_p, 
                                                  const Eigen::Quaterniond& cam_q, 
                                                  double p_fa = 0.05) {
    if (!search_particles_initialized_ || search_particles_.empty()) return;

    // 使用参数服务器传入的相机内参
    double fx = cam_fx_;
    double fy = cam_fy_;
    double cx = cam_cx_;
    double cy = cam_cy_;
    double max_range = cam_max_range_;
    double min_range = 0.1;
    double fov_width = cam_width_;
    double fov_height = cam_height_;
    
    // 对每个搜索粒子进行视场检查
    for (auto& particle : search_particles_) {
      Eigen::Vector3d particle_pos = particle.state.head(3);
      
      // 将粒子位置转换到相机坐标系
      Eigen::Vector3d p_in_cam = cam_q.inverse() * (particle_pos - cam_p);
      
      // 检查是否在视场距离范围内
      bool in_range = (p_in_cam.z() > min_range && p_in_cam.z() < max_range);
      
      if (in_range) {
        // 投影到图像平面
        double x_img = p_in_cam.x() * fx / p_in_cam.z() + cx;
        double y_img = p_in_cam.y() * fy / p_in_cam.z() + cy;
        
        // 检查是否在图像范围内
        if (x_img >= 0 && x_img <= fov_width && y_img >= 0 && y_img <= fov_height) {
        // ✅ 如果有视线检查函数，则检查；否则直接降低权重
        bool los_clear = !los_check_fn_ || los_check_fn_(cam_p, particle_pos);
        if (los_clear) {
          particle.weight *= p_fa;
        }
      }
    }
  }
    
    // 归一化权重
    double sum_w = 0.0;
    for (const auto& p : search_particles_) {
      sum_w += p.weight;
    }
    if (sum_w > 1e-300) {
      for (auto& p : search_particles_) {
        p.weight /= sum_w;
      }
    } else {
      // 如果权重和太小，重置为均匀分布
      for (auto& p : search_particles_) {
        p.weight = 1.0 / search_particles_.size();
      }
    }
  }

  // ========== 以下为旧的标签化共识滤波接口（已弃用）==========
  // std::vector<LabeledLocalStat> computeLabeledLocalStats();
  // void labeledConsensusFilter(const std::vector<LabeledNeighborConsensus>& neighbor_consensus,
  //                            const std::vector<LabeledLocalStat>& local_stats);
  // std::map<SearchIntent, std::vector<Eigen::VectorXd>> getLabeledGMMMeans() const;
  // std::map<SearchIntent, std::vector<Eigen::MatrixXd>> getLabeledGMMCovs() const;
  // std::map<SearchIntent, Eigen::VectorXd> getLabeledGMMPis() const;
  // std::map<SearchIntent, Eigen::VectorXd> getLabeledZetaAlpha() const;
  // std::map<SearchIntent, std::vector<Eigen::VectorXd>> getLabeledZetaA() const;
  // std::map<SearchIntent, std::vector<Eigen::MatrixXd>> getLabeledZetaB() const;
  // void consensusFilterForLabel(SearchIntent label, ...);
  // void globalMStepForLabel(SearchIntent label);
  // ========== 旧接口结束 ==========

  // 从GMM采样新粒子（基于标准DPF实现）
  void sampleParticlesFromGMM();

  // 计算有效粒子数（用于判断是否需要重采样）
  double computeEffectiveSampleSize() const;

  // 获取粒子和权重（用于同步回DPF）
  inline std::pair<Eigen::MatrixXd, Eigen::VectorXd> getParticlesAndWeights() const {
    Eigen::MatrixXd particles(9, search_particles_.size());
    Eigen::VectorXd weights(search_particles_.size());
    for (size_t i = 0; i < search_particles_.size(); ++i) {
      particles.col(i) = search_particles_[i].state;
      weights(i) = search_particles_[i].weight;
    }
    return std::make_pair(particles, weights);
  }

  // Getter methods
  const std::vector<SearchParticle>& getSearchParticles() const {
    return search_particles_;
  }
  
  bool isInitialized() const {
    return search_particles_initialized_;
  }
  
  int getNumComponents() const {
    return num_components_;
  }

  // 新搜索方案：粒子动力学更新
  // 水平速度方向加扰动，速度大小从[0.5*vmax, vmax]按递增密度采样（高速度更高概率），步进dt
  void searchParticlesDynamicsUpdate(Eigen::MatrixXd& particles, int N) {
    const double sigma_theta = 30.0 * M_PI / 180.0;
    const double dt = search_dt_;
    std::normal_distribution<double> theta_dist(0.0, sigma_theta);
    std::uniform_real_distribution<double> unit01(0.0, 1.0);
    const double v_min = 0.5 * search_vmax_;
    const double v_span = 0.5 * search_vmax_;

    for (int i = 0; i < N; ++i) {
      double vx = particles(3, i);
      double vy = particles(4, i);
      double theta_old = std::atan2(vy, vx);
      double theta_new = theta_old + theta_dist(rng_);
      const double u = unit01(rng_);
      const double v_new = v_min + v_span * std::sqrt(u);  // p(v) \propto (v - v_min)
      particles(3, i) = v_new * std::cos(theta_new);
      particles(4, i) = v_new * std::sin(theta_new);
      particles(0, i) += particles(3, i) * dt;
      particles(1, i) += particles(4, i) * dt;
      enforceBoundsForMatrixParticle(particles, i);
    }
  }

  // 搜索常规阶段混合更新：
  // 每隔 mixed_hold_steps 帧，重选一批 turn_ratio 比例的粒子赋予持续角速度 omega；
  // 其余粒子 omega=0，维持当前方向。
  // turn_ratio: 赋予 omega 的粒子比例 [0,1]
  // turn_theta_sigma_deg: 每个重选周期累计转角的标准差（度）
  // mixed_hold_steps: 每组 omega 持续的帧数
  // turning_count_out: 返回本次重选中被赋予 omega 的粒子数（可选）
  void searchParticlesMixedUpdate(Eigen::MatrixXd& particles, int N,
                                  int mixed_step_idx,
                                  int mixed_hold_steps,
                                  double turn_ratio,
                                  double turn_theta_sigma_deg,
                                  int* turning_count_out = nullptr) {
    if (N <= 0) {
      if (turning_count_out) *turning_count_out = 0;
      return;
    }

    const double dt = search_dt_;
    const int hold_steps = std::max(1, mixed_hold_steps);
    const double ratio = clamp(turn_ratio, 0.0, 1.0);
    const int k = std::max(0, std::min(N, (int)std::round(ratio * (double)N)));

    const bool size_changed = ((int)mixed_turn_omegas_.size() != N);
    if (size_changed) {
      mixed_turn_omegas_.assign(N, 0.0);
    }

    const bool reselection_needed =
        size_changed || (mixed_step_idx <= 0) || (mixed_step_idx % hold_steps == 0);
    if (reselection_needed) {
      std::fill(mixed_turn_omegas_.begin(), mixed_turn_omegas_.end(), 0.0);

      std::vector<int> indices(N);
      std::iota(indices.begin(), indices.end(), 0);
      std::shuffle(indices.begin(), indices.end(), rng_);

      const double turn_sigma = std::max(0.0, turn_theta_sigma_deg) * M_PI / 180.0;
      const double omega_sigma = turn_sigma / (std::max(1e-6, dt) * (double)hold_steps);
      std::normal_distribution<double> omega_dist(0.0, omega_sigma);

      for (int j = 0; j < k; ++j) {
        mixed_turn_omegas_[indices[j]] = omega_dist(rng_);
      }
    }

    for (int i = 0; i < N; ++i) {
      const double vx = particles(3, i);
      const double vy = particles(4, i);
      const double v_new = std::hypot(vx, vy);
      const double theta_new = std::atan2(vy, vx) + mixed_turn_omegas_[i] * dt;

      particles(3, i) = v_new * std::cos(theta_new);
      particles(4, i) = v_new * std::sin(theta_new);
      particles(0, i) += particles(3, i) * dt;
      particles(1, i) += particles(4, i) * dt;
      enforceBoundsForMatrixParticle(particles, i);
    }

    if (turning_count_out) *turning_count_out = k;
  }

  // 搜索初期分散更新：
  // 第一步为每个粒子分配一个初始外扩方向；后续几步沿既有方向继续向外推进，
  // 仅施加小角度抖动，避免“每帧重抽方向”导致云团只在原地抹开而非形成外扩环。
  void searchParticlesWideSpreadUpdate(Eigen::MatrixXd& particles, int N,
                                       int spread_step_idx, int spread_total_steps,
                                       double speed_min_ratio = 0.8,
                                       double theta_jitter_deg = 10.0) {
    if (N <= 0) return;
    const double dt = search_dt_;
    const double ratio = clamp(speed_min_ratio, 0.0, 1.0);
    const double v_min = ratio * search_vmax_;
    const double v_max = search_vmax_;
    const double theta_jitter = std::max(0.0, theta_jitter_deg) * M_PI / 180.0;

    std::uniform_real_distribution<double> unit01(0.0, 1.0);
    std::normal_distribution<double> theta_noise(0.0, theta_jitter);
    const bool initialize_directions = (spread_step_idx <= 0);
    const double global_phase = 2.0 * M_PI * unit01(rng_);

    for (int i = 0; i < N; ++i) {
      double theta = 0.0;
      double v = 0.0;
      if (initialize_directions) {
        // 仅在spread首步按索引均匀铺满[0, 2pi)，建立一圈向外发散的初始方向。
        const double frac = ((double)i + 0.5) / (double)N;
        theta = global_phase + 2.0 * M_PI * frac + theta_noise(rng_);
        v = v_min + (v_max - v_min) * unit01(rng_);
      } else {
        const double vx_prev = particles(3, i);
        const double vy_prev = particles(4, i);
        const double v_prev = std::hypot(vx_prev, vy_prev);
        if (v_prev > 1e-6) {
          theta = std::atan2(vy_prev, vx_prev) + theta_noise(rng_);
          v = std::max(v_min, std::min(v_max, v_prev));
        } else {
          // 极少数异常情况下回退到均匀分向，避免零速度粒子滞留原地。
          const double frac = ((double)i + 0.5) / (double)N;
          theta = global_phase + 2.0 * M_PI * frac + theta_noise(rng_);
          v = v_min + (v_max - v_min) * unit01(rng_);
        }
      }
      particles(3, i) = v * std::cos(theta);
      particles(4, i) = v * std::sin(theta);
      particles(0, i) += particles(3, i) * dt;
      particles(1, i) += particles(4, i) * dt;
      enforceBoundsForMatrixParticle(particles, i);
    }
  }

  // === 虚拟栅格无效区域结构体 ===
  // cells: 0=未知, 1=负观测无效, 2=障碍无效
  // cell_times: 负观测格最近一次被标记为无效的时间戳（秒）
  // dist_field: 有符号欧氏距离场（米），无效格内为负，有效格内为正
  //             需在每帧 unionWith 后调用 computeDistField() 更新
  struct InvalidGrid2D {
    double origin_x = 0.0;
    double origin_y = 0.0;
    double resolution = 1.0;
    int nx = 0;
    int ny = 0;
    std::vector<uint8_t> cells;       // 行优先: cells[iy*nx + ix]
    std::vector<double>  cell_times;  // 同尺寸，负观测格最近标记时间（秒）
    std::vector<float>   dist_field;  // 有符号距离场（米）

    bool valid() const {
      return nx > 0 && ny > 0
          && (int)cells.size() == nx * ny
          && cell_times.size() == cells.size();
    }

    void toCell(double wx, double wy, int& ix, int& iy) const {
      ix = static_cast<int>(std::floor((wx - origin_x) / resolution));
      iy = static_cast<int>(std::floor((wy - origin_y) / resolution));
    }

    bool inBounds(int ix, int iy) const {
      return ix >= 0 && ix < nx && iy >= 0 && iy < ny;
    }

    uint8_t get(int ix, int iy) const { return cells[iy * nx + ix]; }

    void set(int ix, int iy, uint8_t val, double stamp_sec = 0.0) {
      int idx = iy * nx + ix;
      cells[idx] = val;
      if (val == 1) {
        cell_times[idx] = stamp_sec;
      } else {
        cell_times[idx] = 0.0;
      }
    }

    bool isInvalid(double wx, double wy, double now_sec, double ttl_sec) const {
      if (!valid()) return false;
      int ix, iy;
      toCell(wx, wy, ix, iy);
      if (!inBounds(ix, iy)) return false;
      int idx = iy * nx + ix;
      uint8_t v = cells[idx];
      if (v == 2) return true;
      if (v == 1) {
        const double t = cell_times[idx];
        return t > 0.0 && (now_sec - t) <= ttl_sec;
      }
      return false;
    }

    int clearExpiredNegObs(double now_sec, double ttl_sec) {
      if (!valid()) return 0;
      int cleared = 0;
      const int n = nx * ny;
      for (int i = 0; i < n; ++i) {
        if (cells[i] != 1) continue;
        const double t = cell_times[i];
        if (t <= 0.0 || (now_sec - t) > ttl_sec) {
          cells[i] = 0;
          cell_times[i] = 0.0;
          ++cleared;
        }
      }
      return cleared;
    }

    // Felzenszwalb 1D 精确平方距离变换（沿一个轴）
    // f_get(q): 输入，q 处的初始平方距离（障碍格=0，其余=INF）
    // f_set(q, val): 输出，写入 q 处的最终平方距离
    template<typename FGet, typename FSet>
    static void fill1DESDF(FGet f_get, FSet f_set, int start, int end) {
      int n = end - start + 1;
      if (n <= 0) return;
      const double INF = 1e18;
      std::vector<int>    v(n);
      std::vector<double> z(n + 1);
      int k = 0;
      v[0] = start; z[0] = -INF; z[1] = INF;
      auto sep = [&](int q, int vk) -> double {
        return ((f_get(q) + (double)q * q) - (f_get(vk) + (double)vk * vk))
             / (2.0 * (q - vk));
      };
      for (int q = start + 1; q <= end; ++q) {
        double s = sep(q, v[k]);
        while (k > 0 && s <= z[k]) {
          --k;
          s = sep(q, v[k]);
        }
        ++k;
        v[k] = q;
        z[k] = s;
        z[k + 1] = INF;
      }
      k = 0;
      for (int q = start; q <= end; ++q) {
        while (z[k+1] < q) ++k;
        double val = (double)(q - v[k])*(q - v[k]) + f_get(v[k]);
        f_set(q, val);
      }
    }

    // 计算有符号欧氏距离场（Felzenszwalb 2D，Y轴→X轴两遍扫描）
    // 无效格（考虑TTL）内为负距离，有效格内为正距离，单位：米
    void computeDistField(double now_sec, double ttl_sec) {
      const int n = nx * ny;
      const double INF = 1e18;
      const double res = resolution;

      // 标记当前哪些格是无效的
      std::vector<bool> inv(n, false);
      for (int i = 0; i < n; ++i) {
        uint8_t v = cells[i];
        if (v == 2) { inv[i] = true; continue; }
        if (v == 1) {
          const double t = cell_times[i];
          inv[i] = (t > 0.0 && (now_sec - t) <= ttl_sec);
        }
      }

      // 辅助 lambda：对给定"障碍"标记做 2D 距离变换（Felzenszwalb，Y→X）
      // 返回每格到最近"障碍"格的欧氏距离（米）
      auto computeEDT = [&](const std::vector<bool>& obstacle) -> std::vector<double> {
        std::vector<double> tmp(n, INF);  // 中间结果（Y轴变换后）
        std::vector<double> out(n, INF);  // 最终结果（X轴变换后）

        // Pass 1: 沿 Y 轴（每列独立）
        for (int ix = 0; ix < nx; ++ix) {
          fill1DESDF(
            [&](int iy) { return obstacle[iy * nx + ix] ? 0.0 : INF; },
            [&](int iy, double val) { tmp[iy * nx + ix] = val; },
            0, ny - 1);
        }
        // Pass 2: 沿 X 轴（每行独立）
        for (int iy = 0; iy < ny; ++iy) {
          fill1DESDF(
            [&](int ix) { return tmp[iy * nx + ix]; },
            [&](int ix, double val) { out[iy * nx + ix] = val; },
            0, nx - 1);
        }
        // 平方距离 → 欧氏距离（米）
        for (int i = 0; i < n; ++i)
          out[i] = (out[i] < INF * 0.5) ? res * std::sqrt(out[i]) : 1e9;
        return out;
      };

      // 到最近无效格的距离（有效格用）
      std::vector<double> dist_to_inv = computeEDT(inv);

      // 到最近有效格的距离（无效格用）
      std::vector<bool> valid_mask(n);
      for (int i = 0; i < n; ++i) valid_mask[i] = !inv[i];
      std::vector<double> dist_to_valid = computeEDT(valid_mask);

      // 合并：有效格取正距离，无效格取负距离
      dist_field.resize(n);
      for (int i = 0; i < n; ++i) {
        if (inv[i])
          dist_field[i] = -static_cast<float>(dist_to_valid[i]);
        else
          dist_field[i] =  static_cast<float>(dist_to_inv[i]);
      }
    }

    // 查询有符号距离（需先调用 computeDistField）
    float signedDist(double wx, double wy) const {
      int ix, iy;
      toCell(wx, wy, ix, iy);
      if (!inBounds(ix, iy) || dist_field.empty())
        return static_cast<float>(resolution);
      return dist_field[iy * nx + ix];
    }

    void unionWith(const InvalidGrid2D& other, double neg_obs_stamp_sec = -1.0) {
      if (!other.valid()) return;
      const bool aligned = (std::abs(origin_x - other.origin_x) < 1e-6 &&
                            std::abs(origin_y - other.origin_y) < 1e-6 &&
                            std::abs(resolution - other.resolution) < 1e-6 &&
                            nx == other.nx && ny == other.ny);
      if (aligned) {
        int n = nx * ny;
        for (int i = 0; i < n; ++i) {
          uint8_t v = other.cells[i];
          if (v == 0) continue;
          if (v == 2) {
            cells[i] = 2;
            cell_times[i] = 0.0;
            continue;
          }
          if (cells[i] == 2) continue;
          const double stamp =
              (neg_obs_stamp_sec >= 0.0) ? neg_obs_stamp_sec : other.cell_times[i];
          if (cells[i] != 1 || stamp >= cell_times[i]) {
            cells[i] = 1;
            cell_times[i] = stamp;
          }
        }
      } else {
        for (int iy = 0; iy < other.ny; ++iy) {
          for (int ix = 0; ix < other.nx; ++ix) {
            uint8_t v = other.cells[iy * other.nx + ix];
            if (v == 0) continue;
            double wx = other.origin_x + (ix + 0.5) * other.resolution;
            double wy = other.origin_y + (iy + 0.5) * other.resolution;
            int my_ix, my_iy;
            toCell(wx, wy, my_ix, my_iy);
            if (!inBounds(my_ix, my_iy)) continue;
            int idx = my_iy * nx + my_ix;
            if (v == 2) {
              cells[idx] = 2;
              cell_times[idx] = 0.0;
              continue;
            }
            if (cells[idx] == 2) continue;
            const int other_idx = iy * other.nx + ix;
            const double stamp =
                (neg_obs_stamp_sec >= 0.0) ? neg_obs_stamp_sec : other.cell_times[other_idx];
            if (cells[idx] != 1 || stamp >= cell_times[idx]) {
              cells[idx] = 1;
              cell_times[idx] = stamp;
            }
          }
        }
      }
    }
  };

  // === 游程编码 / 解码 ===
  // 编码 cells（uint8）数组
  struct RLEGrid {
    double origin_x, origin_y, resolution;
    int nx, ny;
    std::vector<int32_t>  values;  // cells 的游程值
    std::vector<int32_t>  counts;  // 游程长度
  };

  static RLEGrid rleEncode(const InvalidGrid2D& grid) {
    RLEGrid rle;
    rle.origin_x = grid.origin_x;
    rle.origin_y = grid.origin_y;
    rle.resolution = grid.resolution;
    rle.nx = grid.nx;
    rle.ny = grid.ny;
    if (!grid.valid()) return rle;
    int n = grid.nx * grid.ny;
    uint8_t  cur_val = grid.cells[0];
    int32_t  cnt = 1;
    for (int i = 1; i < n; ++i) {
      if (grid.cells[i] == cur_val) {
        ++cnt;
      } else {
        rle.values.push_back(static_cast<int32_t>(cur_val));
        rle.counts.push_back(cnt);
        cur_val = grid.cells[i];
        cnt = 1;
      }
    }
    rle.values.push_back(static_cast<int32_t>(cur_val));
    rle.counts.push_back(cnt);
    return rle;
  }

  static InvalidGrid2D rleDecode(const RLEGrid& rle) {
    InvalidGrid2D grid;
    grid.origin_x   = rle.origin_x;
    grid.origin_y   = rle.origin_y;
    grid.resolution = rle.resolution;
    grid.nx = rle.nx;
    grid.ny = rle.ny;
    int total = rle.nx * rle.ny;
    grid.cells.reserve(total);
    grid.cell_times.reserve(total);
    for (size_t i = 0; i < rle.values.size(); ++i) {
      uint8_t v = static_cast<uint8_t>(rle.values[i]);
      for (int32_t k = 0; k < rle.counts[i]; ++k) {
        grid.cells.push_back(v);
        grid.cell_times.push_back(0.0);
      }
    }
    return grid;
  }

  // === 从粒子标记负观测无效格（仅负观测，障碍格由地图初始化负责）===
  void markInvalidFromParticles(InvalidGrid2D& grid,
                                const Eigen::MatrixXd& particles, int N,
                                const Eigen::Vector3d& cam_p,
                                const Eigen::Quaterniond& cam_q,
                                double now_sec = 0.0) {
    if (!grid.valid() || N <= 0) return;
    Eigen::Matrix3d R_cw = cam_q.toRotationMatrix().transpose();

    for (int i = 0; i < N; ++i) {
      Eigen::Vector3d pos = particles.col(i).head(3);
      int ix, iy;
      grid.toCell(pos.x(), pos.y(), ix, iy);
      if (!grid.inBounds(ix, iy)) continue;
      if (grid.get(ix, iy) == 2) continue; // 障碍格跳过

      // FOV检查
      Eigen::Vector3d p_c = R_cw * (pos - cam_p);
      if (p_c.z() <= 0.1 || p_c.z() >= cam_max_range_) continue;
      double u = p_c.x() * cam_fx_ / p_c.z() + cam_cx_;
      double v = p_c.y() * cam_fy_ / p_c.z() + cam_cy_;
      if (u < 0 || u > cam_width_ || v < 0 || v > cam_height_) continue;

      // 视线检查
      if (los_check_fn_ && !los_check_fn_(cam_p, pos)) continue;

      grid.set(ix, iy, 1, now_sec);
    }
  }

  // === 在给定高度平面上，用当前视场覆盖区域直接点亮负观测无效格 ===
  // 仅遍历当前视场在 plane_z 平面上的投影包围盒，不扫描全图。
  int markInvalidFromViewFootprint(InvalidGrid2D& grid,
                                   const Eigen::Vector3d& cam_p,
                                   const Eigen::Quaterniond& cam_q,
                                   double plane_z,
                                   double now_sec = 0.0,
                                   int border_samples_per_edge = 12) {
    if (!grid.valid()) return 0;
    if (!std::isfinite(plane_z)) return 0;

    const double min_depth = 0.1;
    const double max_depth = cam_max_range_;
    if (!(max_depth > min_depth)) return 0;

    const Eigen::Matrix3d R_wc = cam_q.toRotationMatrix();
    const Eigen::Matrix3d R_cw = R_wc.transpose();
    const int edge_samples = std::max(2, border_samples_per_edge);

    std::vector<Eigen::Vector2d> footprint_pts;
    footprint_pts.reserve(edge_samples * 4 + 1);

    auto appendPlaneIntersection = [&](double u, double v) {
      Eigen::Vector3d ray_c((u - cam_cx_) / cam_fx_,
                            (v - cam_cy_) / cam_fy_,
                            1.0);
      Eigen::Vector3d ray_w = R_wc * ray_c;
      if (std::abs(ray_w.z()) < 1e-6) return;

      const double depth = (plane_z - cam_p.z()) / ray_w.z();
      if (!std::isfinite(depth) || depth <= min_depth || depth >= max_depth) return;

      Eigen::Vector3d p_w = cam_p + depth * ray_w;
      if (!std::isfinite(p_w.x()) || !std::isfinite(p_w.y())) return;
      footprint_pts.emplace_back(p_w.x(), p_w.y());
    };

    for (int i = 0; i < edge_samples; ++i) {
      const double alpha = (edge_samples == 1) ? 0.0 : (double)i / (double)(edge_samples - 1);
      const double u = alpha * cam_width_;
      const double v = alpha * cam_height_;
      appendPlaneIntersection(u, 0.0);
      appendPlaneIntersection(u, cam_height_);
      appendPlaneIntersection(0.0, v);
      appendPlaneIntersection(cam_width_, v);
    }
    appendPlaneIntersection(cam_cx_, cam_cy_);

    if (footprint_pts.empty()) return 0;

    int min_ix = grid.nx;
    int max_ix = -1;
    int min_iy = grid.ny;
    int max_iy = -1;
    for (const auto& pt : footprint_pts) {
      int ix, iy;
      grid.toCell(pt.x(), pt.y(), ix, iy);
      min_ix = std::min(min_ix, ix);
      max_ix = std::max(max_ix, ix);
      min_iy = std::min(min_iy, iy);
      max_iy = std::max(max_iy, iy);
    }

    min_ix = std::max(0, min_ix);
    max_ix = std::min(grid.nx - 1, max_ix);
    min_iy = std::max(0, min_iy);
    max_iy = std::min(grid.ny - 1, max_iy);
    if (min_ix > max_ix || min_iy > max_iy) return 0;

    int marked = 0;
    for (int iy = min_iy; iy <= max_iy; ++iy) {
      for (int ix = min_ix; ix <= max_ix; ++ix) {
        if (grid.get(ix, iy) == 2) continue;

        const double wx = grid.origin_x + (ix + 0.5) * grid.resolution;
        const double wy = grid.origin_y + (iy + 0.5) * grid.resolution;
        Eigen::Vector3d pos_w(wx, wy, plane_z);
        Eigen::Vector3d p_c = R_cw * (pos_w - cam_p);

        if (p_c.z() <= min_depth || p_c.z() >= max_depth) continue;

        const double u = p_c.x() * cam_fx_ / p_c.z() + cam_cx_;
        const double v = p_c.y() * cam_fy_ / p_c.z() + cam_cy_;
        if (u < 0.0 || u > cam_width_ || v < 0.0 || v > cam_height_) continue;

        if (los_check_fn_ && !los_check_fn_(cam_p, pos_w)) continue;

        if (grid.get(ix, iy) != 1) ++marked;
        grid.set(ix, iy, 1, now_sec);
      }
    }
    return marked;
  }

  // === 用无效栅格更新粒子权重（论文2.2式，分段线性衰减）===
  // phi_min: 深无效区最小衰减因子
  // phi_mid: 无效边界处衰减因子（r=0）
  // r_occ:   无效区内截断距离（米，<0）
  // r_safe:  有效区外侧安全距离（米，>0）
  struct PruneStats {
    int total_particles = 0;
    int invalid_cell_particles = 0;
    int strong_decay_particles = 0;
    double weight_sum_before = 0.0;
    double weight_sum_after_raw = 0.0;
    double weight_sum_after_norm = 0.0;
    double mean_phi = 1.0;
    double min_phi = 1.0;
  };

  void pruneParticlesByInvalidGrid(Eigen::MatrixXd& particles,
                                   Eigen::VectorXd& weights, int N,
                                   InvalidGrid2D& grid,
                                   double now_sec, double ttl_sec,
                                   double phi_min = 0.0,
                                   double phi_mid = 0.5,
                                   double r_occ   = -0.5,
                                   double r_safe  =  0.5,
                                   PruneStats* stats_out = nullptr,
                                   double strong_phi_threshold = 0.35) {
    if (!grid.valid() || N <= 0 || weights.size() < N) return;

    phi_min = std::max(0.0, std::min(1.0, phi_min));
    phi_mid = std::max(phi_min, std::min(1.0, phi_mid));
    if (!(r_occ < 0.0)) r_occ = -std::max(1e-3, grid.resolution);
    if (!(r_safe > 0.0)) r_safe =  std::max(1e-3, grid.resolution);
    strong_phi_threshold = std::max(0.0, std::min(1.0, strong_phi_threshold));

    if (stats_out) {
      stats_out->total_particles = N;
      stats_out->invalid_cell_particles = 0;
      stats_out->strong_decay_particles = 0;
      stats_out->weight_sum_before = weights.head(N).sum();
      stats_out->weight_sum_after_raw = 0.0;
      stats_out->weight_sum_after_norm = 0.0;
      stats_out->mean_phi = 0.0;
      stats_out->min_phi = 1.0;
    }

    // 每帧基于当前TTL有效无效域重建有符号距离场，供权重衰减使用
    grid.computeDistField(now_sec, ttl_sec);

    auto decayFactor = [&](double r) -> double {
      if (r <= r_occ) {
        return phi_min;
      }
      if (r < 0.0) {
        // 公式: phi_min - (phi_mid - phi_min) * (r - r_occ) / r_occ, r_occ<0
        return phi_min - (phi_mid - phi_min) * (r - r_occ) / r_occ;
      }
      if (r < r_safe) {
        // 公式: phi_mid + (1 - phi_mid) * r / r_safe
        return phi_mid + (1.0 - phi_mid) * r / r_safe;
      }
      return 1.0;
    };

    for (int i = 0; i < N; ++i) {
      double wx = particles(0, i), wy = particles(1, i);
      int ix, iy;
      grid.toCell(wx, wy, ix, iy);
      if (!grid.inBounds(ix, iy)) continue;

      const double r = static_cast<double>(grid.signedDist(wx, wy));
      const double phi = std::max(phi_min, std::min(1.0, decayFactor(r)));
      weights(i) *= phi;

      if (stats_out) {
        stats_out->weight_sum_after_raw += weights(i);
        stats_out->mean_phi += phi;
        stats_out->min_phi = std::min(stats_out->min_phi, phi);
        if (phi <= strong_phi_threshold) ++stats_out->strong_decay_particles;
        if (phi < 0.999999) ++stats_out->invalid_cell_particles;
      }
    }

    double wsum = weights.sum();
    if (wsum > 1e-300) weights /= wsum;
    else               weights.setConstant(N, 1.0 / N);

    if (stats_out) {
      stats_out->weight_sum_after_norm = weights.head(N).sum();
      stats_out->mean_phi = (N > 0) ? (stats_out->mean_phi / static_cast<double>(N)) : 1.0;
    }
  }

  // === 6D全粒子共识 ===

  // 6D本地统计量
  struct LocalStat6D {
    int drone_id;
    int C;
    Eigen::VectorXd alpha;              // [C]
    std::vector<Eigen::VectorXd> a;     // C x [6]
    std::vector<Eigen::MatrixXd> b;     // C x [6x6]
    bool has_obs;
    ros::Time timestamp;
  };

  // 6D邻居共识状态
  struct NeighborConsensus6D {
    Eigen::VectorXd zeta_alpha;
    std::vector<Eigen::VectorXd> zeta_a;
    std::vector<Eigen::MatrixXd> zeta_b;
    bool has_obs;
    ros::Time timestamp;
  };

  struct SearchFallbackGMM6D {
    bool valid = false;
    int C = 0;
    Eigen::VectorXd pi;
    std::vector<Eigen::VectorXd> mu;
    std::vector<Eigen::MatrixXd> S;
  };

  // 初始化6D搜索GMM（进入搜索模式时调用）
  void initSearchGMM6D(const Eigen::MatrixXd& particles,
                       const Eigen::VectorXd& weights, int N, int C) {
    search_C_ = C;
    search_gmm_pi_.setConstant(C, 1.0 / C);
    search_gmm_mu_.resize(C);
    search_gmm_S_.resize(C);
    // 用加权均值初始化
    Eigen::VectorXd mean6 = Eigen::VectorXd::Zero(nx6_);
    for (int i = 0; i < N; ++i) mean6 += weights(i) * particles.col(i).head(nx6_);
    for (int c = 0; c < C; ++c) {
      search_gmm_mu_[c] = mean6;
      // 加随机扰动区分各分量
      std::normal_distribution<double> dist(0.0, 0.3);
      for (int j = 0; j < nx6_; ++j) search_gmm_mu_[c](j) += dist(rng_);
      search_gmm_S_[c] = Eigen::MatrixXd::Identity(nx6_, nx6_) * 0.5;
    }
    // 初始化共识状态
    search_zeta_alpha_ = search_gmm_pi_ * N;
    search_zeta_a_.resize(C);
    search_zeta_b_.resize(C);
    for (int c = 0; c < C; ++c) {
      search_zeta_a_[c] = search_gmm_mu_[c] * search_zeta_alpha_(c);
      search_zeta_b_[c] = search_gmm_S_[c] * search_zeta_alpha_(c);
    }
    search_gmm_initialized_ = true;
  }

  // E步：计算6D本地统计量
  LocalStat6D computeLocalStats6D(const Eigen::MatrixXd& particles,
                                   const Eigen::VectorXd& weights,
                                   int N, int drone_id, bool has_obs) {
    LocalStat6D ls;
    ls.drone_id = drone_id;
    ls.C = search_C_;
    ls.has_obs = has_obs;
    ls.timestamp = ros::Time::now();
    ls.alpha.setZero(search_C_);
    ls.a.resize(search_C_);
    ls.b.resize(search_C_);
    for (int c = 0; c < search_C_; ++c) {
      ls.a[c].setZero(nx6_);
      ls.b[c].setZero(nx6_, nx6_);
    }
    // 预计算GMM逆和行列式
    std::vector<Eigen::MatrixXd> S_inv(search_C_);
    std::vector<double> S_det(search_C_);
    for (int c = 0; c < search_C_; ++c) {
      S_inv[c] = search_gmm_S_[c].inverse();
      S_det[c] = search_gmm_S_[c].determinant();
      if (S_det[c] < 1e-300) S_det[c] = 1e-300;
    }
    for (int n = 0; n < N; ++n) {
      Eigen::VectorXd x6 = particles.col(n).head(nx6_);
      double w_n = weights(n);
      Eigen::VectorXd resp(search_C_);
      double resp_sum = 0.0;
      for (int c = 0; c < search_C_; ++c) {
        Eigen::VectorXd diff = x6 - search_gmm_mu_[c];
        double exponent = -0.5 * diff.transpose() * S_inv[c] * diff;
        double nc = 1.0 / std::sqrt(std::pow(2.0 * M_PI, nx6_) * S_det[c]);
        resp(c) = search_gmm_pi_(c) * nc * std::exp(exponent);
        resp_sum += resp(c);
      }
      if (resp_sum > 1e-300) resp /= resp_sum;
      else resp.setConstant(1.0 / search_C_);
      for (int c = 0; c < search_C_; ++c) {
        double alpha_nc = w_n * resp(c);
        ls.alpha(c) += alpha_nc;
        ls.a[c] += alpha_nc * x6;
        Eigen::VectorXd diff_c = x6 - search_gmm_mu_[c];
        ls.b[c] += alpha_nc * diff_c * diff_c.transpose();
      }
    }
    return ls;
  }

  // 共识滤波单次更新
  void consensusFilterOnce6D(const std::vector<NeighborConsensus6D>& neighbors,
                              const LocalStat6D& local_stat) {
    int d_max = neighbors.size() + 1;
    double eps = std::min(1.0 / d_max, 0.3);
    for (int c = 0; c < search_C_; ++c) {
      double alpha_diff = 0.0;
      Eigen::VectorXd a_diff = Eigen::VectorXd::Zero(nx6_);
      Eigen::MatrixXd b_diff = Eigen::MatrixXd::Zero(nx6_, nx6_);
      for (const auto& nc : neighbors) {
        if ((int)nc.zeta_alpha.size() != search_C_) continue;
        alpha_diff += nc.zeta_alpha(c) - search_zeta_alpha_(c);
        a_diff += nc.zeta_a[c] - search_zeta_a_[c];
        b_diff += nc.zeta_b[c] - search_zeta_b_[c];
      }
      double local_alpha = local_stat.alpha(c);
      Eigen::VectorXd local_a = local_stat.a[c];
      Eigen::MatrixXd local_b = local_stat.b[c];
      search_zeta_alpha_(c) += eps * (alpha_diff + (local_alpha - search_zeta_alpha_(c)));
      search_zeta_a_[c] += eps * (a_diff + (local_a - search_zeta_a_[c]));
      search_zeta_b_[c] += eps * (b_diff + (local_b - search_zeta_b_[c]));
    }
  }

  // M步：从共识状态更新6D GMM
  void globalMStep6D() {
    double alpha_sum = search_zeta_alpha_.sum();
    if (alpha_sum < 1e-300) return;
    for (int c = 0; c < search_C_; ++c) {
      if (search_zeta_alpha_(c) < 1e-300) continue;
      search_gmm_pi_(c) = search_zeta_alpha_(c) / alpha_sum;
      search_gmm_mu_[c] = search_zeta_a_[c] / search_zeta_alpha_(c);
      search_gmm_S_[c] = search_zeta_b_[c] / search_zeta_alpha_(c);
      search_gmm_S_[c] += Eigen::MatrixXd::Identity(nx6_, nx6_) * 1e-4;
    }
    double pi_sum = search_gmm_pi_.sum();
    if (pi_sum > 1e-300) search_gmm_pi_ /= pi_sum;
  }

  // 单帧EM步骤（E + 共识 + M）
  LocalStat6D emStep6D(const Eigen::MatrixXd& particles,
                        const Eigen::VectorXd& weights, int N,
                        int drone_id, bool has_obs,
                        const std::vector<NeighborConsensus6D>& neighbors) {
    if (!search_gmm_initialized_) return LocalStat6D();
    LocalStat6D ls = computeLocalStats6D(particles, weights, N, drone_id, has_obs);
    consensusFilterOnce6D(neighbors, ls);
    globalMStep6D();
    return ls;
  }

  // 获取6D共识状态（用于发布给邻居）
  int getSearchC() const { return search_C_; }
  const Eigen::VectorXd& getSearchZetaAlpha() const { return search_zeta_alpha_; }
  const std::vector<Eigen::VectorXd>& getSearchZetaA() const { return search_zeta_a_; }
  const std::vector<Eigen::MatrixXd>& getSearchZetaB() const { return search_zeta_b_; }
  bool isSearchGMMInitialized() const { return search_gmm_initialized_; }

  // === 从6D搜索共识GMM采样新粒子（替换旧粒子的前6维）===
  // 高权重GMM分量采样更多粒子，低权重区域粒子自然被淘汰
  void sampleParticlesFromSearchGMM(Eigen::MatrixXd& particles,
                                     Eigen::VectorXd& weights, int N) {
    if (!search_gmm_initialized_) return;
    std::uniform_real_distribution<double> uniform(0.0, 1.0);
    std::normal_distribution<double> normal(0.0, 1.0);

    for (int i = 0; i < N; ++i) {
      // 按混合权重选择GMM分量
      double u = uniform(rng_);
      double cum = 0.0;
      int c_sel = search_C_ - 1;
      for (int c = 0; c < search_C_; ++c) {
        cum += search_gmm_pi_(c);
        if (u <= cum) { c_sel = c; break; }
      }
      // 从选中分量采样6D状态
      Eigen::LLT<Eigen::MatrixXd> llt(search_gmm_S_[c_sel]);
      Eigen::MatrixXd L = llt.matrixL();
      Eigen::VectorXd noise(nx6_);
      for (int j = 0; j < nx6_; ++j) noise(j) = normal(rng_);
      Eigen::VectorXd x6 = search_gmm_mu_[c_sel] + L * noise;
      // 写入粒子的前6维（pos+vel），保留后3维（rpy）
      particles.col(i).head(nx6_) = x6;
    }
    weights.setConstant(N, 1.0 / N);
  }

  // 从当前粒子云拟合一个跨机接管用的6D GMM快照
  SearchFallbackGMM6D buildSearchFallbackGMM(const Eigen::MatrixXd& particles,
                                             const Eigen::VectorXd& weights, int N, int C) const {
    SearchFallbackGMM6D model;
    if (N <= 0 || particles.rows() < nx6_ || particles.cols() <= 0 || weights.size() <= 0) {
      return model;
    }

    const int n = std::min<int>(N, std::min<int>(particles.cols(), weights.size()));
    struct Candidate {
      int idx = -1;
      double w = 0.0;
      Eigen::Vector3d pos = Eigen::Vector3d::Zero();
      Eigen::Vector3d vel = Eigen::Vector3d::Zero();
    };

    std::vector<Candidate> candidates;
    candidates.reserve(n);
    for (int i = 0; i < n; ++i) {
      const double w = std::max(0.0, weights(i));
      if (w <= 1e-12) continue;
      Candidate c;
      c.idx = i;
      c.w = w;
      c.pos = particles.col(i).head(3);
      c.vel = particles.col(i).segment(3, 3);
      candidates.push_back(c);
    }
    if (candidates.empty()) return model;

    const int K = std::max(1, std::min(C, static_cast<int>(candidates.size())));
    std::vector<int> seeds;
    std::vector<bool> selected(candidates.size(), false);
    seeds.reserve(K);

    int first = -1;
    double first_score = -1.0;
    for (size_t i = 0; i < candidates.size(); ++i) {
      if (candidates[i].w > first_score) {
        first_score = candidates[i].w;
        first = static_cast<int>(i);
      }
    }
    if (first < 0) return model;
    seeds.push_back(first);
    selected[first] = true;

    while (static_cast<int>(seeds.size()) < K) {
      int best = -1;
      double best_score = -1.0;
      for (size_t i = 0; i < candidates.size(); ++i) {
        if (selected[i]) continue;
        double dmin = std::numeric_limits<double>::infinity();
        for (int sid : seeds) {
          dmin = std::min(dmin, (candidates[i].pos - candidates[sid].pos).norm());
        }
        const double score = candidates[i].w * std::max(0.1, dmin);
        if (score > best_score) {
          best_score = score;
          best = static_cast<int>(i);
        }
      }
      if (best < 0) break;
      seeds.push_back(best);
      selected[best] = true;
    }

    const double pos_sigma2 = 1.5 * 1.5;
    const double vel_sigma2 = 1.0 * 1.0;
    const double eps = 1e-12;
    model.C = static_cast<int>(seeds.size());
    model.pi = Eigen::VectorXd::Zero(model.C);
    model.mu.resize(model.C);
    model.S.resize(model.C);

    for (int k = 0; k < model.C; ++k) {
      model.mu[k].setZero(nx6_);
      model.S[k].setZero(nx6_, nx6_);
      const Candidate& seed = candidates[seeds[k]];
      Eigen::VectorXd sum_x = Eigen::VectorXd::Zero(nx6_);
      double alpha = 0.0;
      for (const auto& c : candidates) {
        const Eigen::Vector3d dp = c.pos - seed.pos;
        const Eigen::Vector3d dv = c.vel - seed.vel;
        const double dist2 = dp.squaredNorm() / pos_sigma2 + dv.squaredNorm() / vel_sigma2;
        const double kernel = std::exp(-0.5 * dist2);
        const double w = c.w * kernel;
        if (w <= eps) continue;
        sum_x += w * particles.col(c.idx).head(nx6_);
        alpha += w;
      }
      if (alpha <= eps) {
        model.mu[k] = particles.col(seed.idx).head(nx6_);
        model.S[k] = Eigen::MatrixXd::Identity(nx6_, nx6_) * 0.5;
        model.pi(k) = seed.w;
        continue;
      }
      Eigen::VectorXd mean6 = sum_x / alpha;
      Eigen::MatrixXd cov6 = Eigen::MatrixXd::Zero(nx6_, nx6_);
      for (const auto& c : candidates) {
        const Eigen::Vector3d dp = c.pos - seed.pos;
        const Eigen::Vector3d dv = c.vel - seed.vel;
        const double dist2 = dp.squaredNorm() / pos_sigma2 + dv.squaredNorm() / vel_sigma2;
        const double kernel = std::exp(-0.5 * dist2);
        const double w = c.w * kernel;
        if (w <= eps) continue;
        const Eigen::VectorXd diff = particles.col(c.idx).head(nx6_) - mean6;
        cov6 += w * diff * diff.transpose();
      }
      cov6 /= alpha;
      cov6 += Eigen::MatrixXd::Identity(nx6_, nx6_) * 1e-4;
      model.mu[k] = mean6;
      model.S[k] = cov6;
      model.pi(k) = alpha;
    }

    const double pi_sum = model.pi.sum();
    if (!(pi_sum > 1e-300)) return model;
    model.pi /= pi_sum;
    model.valid = true;
    return model;
  }

  // 依据外部拟合的6D GMM模型采样粒子
  void sampleParticlesFromSearchGMMModel(Eigen::MatrixXd& particles,
                                         Eigen::VectorXd& weights, int N,
                                         const SearchFallbackGMM6D& model) {
    if (!model.valid || model.C <= 0 || model.pi.size() != model.C) return;
    if (particles.rows() < nx6_) return;

    std::uniform_real_distribution<double> uniform(0.0, 1.0);
    std::normal_distribution<double> normal(0.0, 1.0);
    Eigen::MatrixXd new_particles(particles.rows(), N);

    const double pi_sum = model.pi.sum();
    if (!(pi_sum > 1e-300)) return;
    const Eigen::VectorXd pi = model.pi / pi_sum;

    for (int i = 0; i < N; ++i) {
      double u = uniform(rng_);
      double cum = 0.0;
      int c_sel = model.C - 1;
      for (int c = 0; c < model.C; ++c) {
        cum += pi(c);
        if (u <= cum) { c_sel = c; break; }
      }

      Eigen::LLT<Eigen::MatrixXd> llt(model.S[c_sel]);
      Eigen::MatrixXd L = llt.matrixL();
      Eigen::VectorXd noise(nx6_);
      for (int j = 0; j < nx6_; ++j) noise(j) = normal(rng_);
      Eigen::VectorXd x6 = model.mu[c_sel] + L * noise;

      new_particles.col(i) = particles.col(i);
      new_particles.col(i).head(nx6_) = x6;
      new_particles(6, i) = wrapAngle(new_particles(6, i));
      new_particles(7, i) = wrapAngle(new_particles(7, i));
      new_particles(8, i) = wrapAngle(new_particles(8, i));
    }
    particles = new_particles;
    weights.setConstant(N, 1.0 / N);
  }

  // === 从6D共识GMM中提取K个热点位置 ===
  // 按权重排序取top-K分量，返回位置和速度
  struct SearchHotspot {
    Eigen::Vector3d pos;
    Eigen::Vector3d vel;
    double weight;
  };

  std::vector<SearchHotspot> extractHotspots(
      int K, const std::vector<Eigen::Vector3d>& drone_positions = {}) const {
    if (!search_gmm_initialized_ || search_C_ == 0) return {};

    // === 加权最远点采样，排斥无人机当前位置 ===
    // min_dist初始化为到最近无人机的距离（而非无穷大）
    // 这样无人机附近的分量天然得分低，热点会远离已搜索区域
    std::vector<double> min_dist(search_C_, 1e9);
    for (int i = 0; i < search_C_; ++i) {
      Eigen::Vector3d pos_i = search_gmm_mu_[i].head(3);
      for (const auto& dp : drone_positions) {
        double d = (pos_i - dp).norm();
        min_dist[i] = std::min(min_dist[i], d);
      }
    }

    std::vector<bool> selected(search_C_, false);
    std::vector<int> seed_indices;

    // 第一个种子：score = weight * min_dist_to_drones
    {
      int best = -1;
      double best_score = -1.0;
      for (int i = 0; i < search_C_; ++i) {
        double score = search_gmm_pi_(i) * min_dist[i];
        if (score > best_score) { best_score = score; best = i; }
      }
      if (best >= 0) {
        seed_indices.push_back(best);
        selected[best] = true;
      }
    }

    while ((int)seed_indices.size() < K && (int)seed_indices.size() < search_C_) {
      // 更新min_dist（考虑最新种子）
      int last_seed = seed_indices.back();
      Eigen::Vector3d last_pos = search_gmm_mu_[last_seed].head(3);
      for (int i = 0; i < search_C_; ++i) {
        if (selected[i]) continue;
        double d = (search_gmm_mu_[i].head(3) - last_pos).norm();
        min_dist[i] = std::min(min_dist[i], d);
      }
      // 选 score = weight * min_dist 最大的
      int best = -1;
      double best_score = -1.0;
      for (int i = 0; i < search_C_; ++i) {
        if (selected[i]) continue;
        double score = search_gmm_pi_(i) * min_dist[i];
        if (score > best_score) { best_score = score; best = i; }
      }
      if (best < 0) break;
      seed_indices.push_back(best);
      selected[best] = true;
    }

    int num_seeds = (int)seed_indices.size();
    if (num_seeds == 0) return {};

    // === 第2步：把所有分量分配到最近的种子，加权合并 ===
    std::vector<double> total_w(num_seeds, 0.0);
    std::vector<Eigen::VectorXd> weighted_sum(num_seeds);
    for (int k = 0; k < num_seeds; ++k) {
      weighted_sum[k] = Eigen::VectorXd::Zero(nx6_);
    }
    for (int i = 0; i < search_C_; ++i) {
      Eigen::Vector3d pos_i = search_gmm_mu_[i].head(3);
      double best_d = 1e9;
      int best_k = 0;
      for (int k = 0; k < num_seeds; ++k) {
        double d = (pos_i - search_gmm_mu_[seed_indices[k]].head(3)).norm();
        if (d < best_d) { best_d = d; best_k = k; }
      }
      double w = search_gmm_pi_(i);
      weighted_sum[best_k] += w * search_gmm_mu_[i];
      total_w[best_k] += w;
    }

    // === 第3步：生成热点，按权重降序 ===
    std::vector<SearchHotspot> hotspots;
    for (int k = 0; k < num_seeds; ++k) {
      SearchHotspot h;
      if (total_w[k] > 1e-300) {
        Eigen::VectorXd mean6 = weighted_sum[k] / total_w[k];
        h.pos = mean6.head(3);
        h.vel = mean6.tail(3);
      } else {
        h.pos = search_gmm_mu_[seed_indices[k]].head(3);
        h.vel = search_gmm_mu_[seed_indices[k]].tail(3);
      }
      h.weight = total_w[k];
      hotspots.push_back(h);
    }
    std::sort(hotspots.begin(), hotspots.end(),
              [](const SearchHotspot& a, const SearchHotspot& b) {
                return a.weight > b.weight;
              });
    return hotspots;
  }

  // === 基于当前粒子云提取前沿热点 ===
  // 目标：优先选择远离无人机且不在无效栅格区域内的热点
  std::vector<SearchHotspot> extractFrontierHotspotsFromParticles(
      const Eigen::MatrixXd& particles,
      const Eigen::VectorXd& weights,
      int N, int K,
      const std::vector<Eigen::Vector3d>& drone_positions,
      const InvalidGrid2D& invalid_grid,
      double min_drone_dist = 2.0,
      double seed_radius = 1.5,
      double /*invalid_reject_ratio*/ = 1.0) const {
    std::vector<SearchHotspot> hotspots;
    if (N <= 0 || K <= 0 || particles.cols() <= 0 || weights.size() <= 0) {
      return hotspots;
    }

    const int n = std::min<int>(N, std::min<int>(particles.cols(), weights.size()));
    const double min_dist = std::max(0.0, min_drone_dist);
    const double radius = std::max(0.1, seed_radius);

    const bool use_invalid_filter = invalid_grid.valid();

    struct Candidate {
      int idx = -1;
      double score = 0.0;
      double min_drone_d = 0.0;
    };
    std::vector<Candidate> candidates;
    candidates.reserve(n);

    for (int i = 0; i < n; ++i) {
      const double w = std::max(0.0, weights(i));
      if (w < 1e-12) continue;

      const Eigen::Vector3d pos = particles.col(i).head(3);
      if (use_invalid_filter) {
        int ix, iy;
        invalid_grid.toCell(pos.x(), pos.y(), ix, iy);
        if (invalid_grid.inBounds(ix, iy) && invalid_grid.get(ix, iy) != 0) continue;
      }

      double nearest_drone_dist = std::numeric_limits<double>::infinity();
      if (drone_positions.empty()) {
        nearest_drone_dist = min_dist;
      } else {
        for (const auto& dp : drone_positions) {
          nearest_drone_dist = std::min(nearest_drone_dist, (pos - dp).norm());
        }
      }

      // 前沿评分：基础权重 + 远离无人机加成
      double frontier_bonus = std::max(0.0, nearest_drone_dist - min_dist);
      double score = w * (1.0 + frontier_bonus);
      candidates.push_back({i, score, nearest_drone_dist});
    }

    if (candidates.empty()) {
      return hotspots;
    }

    const int max_seed_num = std::min<int>(K, candidates.size());
    std::vector<int> seed_candidates;
    std::vector<bool> selected(candidates.size(), false);
    seed_candidates.reserve(max_seed_num);

    // Seed 1: 前沿评分最大
    int first_seed = -1;
    double first_score = -1.0;
    for (size_t i = 0; i < candidates.size(); ++i) {
      if (candidates[i].score > first_score) {
        first_score = candidates[i].score;
        first_seed = i;
      }
    }
    if (first_seed < 0) {
      return hotspots;
    }
    seed_candidates.push_back(first_seed);
    selected[first_seed] = true;

    // Seed 2..K: 加权最远点采样
    while ((int)seed_candidates.size() < max_seed_num) {
      int best_idx = -1;
      double best_score = -1.0;
      for (size_t i = 0; i < candidates.size(); ++i) {
        if (selected[i]) continue;
        const Eigen::Vector3d p = particles.col(candidates[i].idx).head(3);
        double dmin = std::numeric_limits<double>::infinity();
        for (int seed_i : seed_candidates) {
          const Eigen::Vector3d ps = particles.col(candidates[seed_i].idx).head(3);
          dmin = std::min(dmin, (p - ps).norm());
        }
        double score = candidates[i].score * std::max(0.1, dmin);
        if (score > best_score) {
          best_score = score;
          best_idx = i;
        }
      }
      if (best_idx < 0) break;
      seed_candidates.push_back(best_idx);
      selected[best_idx] = true;
    }

    // 每个seed只在局部邻域做加权均值，避免被全局分量拉回中心
    hotspots.reserve(seed_candidates.size());
    const double inv_sigma2 = 1.0 / (radius * radius);
    for (int seed_i : seed_candidates) {
      const int seed_particle_idx = candidates[seed_i].idx;
      const Eigen::Vector3d seed_pos = particles.col(seed_particle_idx).head(3);

      Eigen::Vector3d sum_pos = Eigen::Vector3d::Zero();
      Eigen::Vector3d sum_vel = Eigen::Vector3d::Zero();
      double sum_w = 0.0;

      for (const auto& c : candidates) {
        const Eigen::Vector3d p = particles.col(c.idx).head(3);
        const double d = (p - seed_pos).norm();
        if (d > radius) continue;

        const Eigen::Vector3d v = particles.col(c.idx).segment(3, 3);
        const double base_w = std::max(0.0, weights(c.idx));
        const double kernel = std::exp(-0.5 * d * d * inv_sigma2);
        const double frontier_term = 1.0 + std::max(0.0, c.min_drone_d - min_dist);
        const double local_w = base_w * kernel * frontier_term;
        if (local_w < 1e-12) continue;

        sum_pos += local_w * p;
        sum_vel += local_w * v;
        sum_w += local_w;
      }

      SearchHotspot h;
      if (sum_w > 1e-10) {
        h.pos = sum_pos / sum_w;
        h.vel = sum_vel / sum_w;
        h.weight = sum_w;
      } else {
        h.pos = seed_pos;
        h.vel = particles.col(seed_particle_idx).segment(3, 3);
        h.weight = candidates[seed_i].score;
      }
      hotspots.push_back(h);
    }

    std::sort(hotspots.begin(), hotspots.end(),
              [](const SearchHotspot& a, const SearchHotspot& b) {
                return a.weight > b.weight;
              });
    return hotspots;
  }

  double getSearchVmax() const { return search_vmax_; }
  double getSearchDt() const { return search_dt_; }
  void setSearchDt(double dt) {
    if (std::isfinite(dt) && dt > 0.0) search_dt_ = dt;
  }

  // 搜索模式粒子边界约束（通常使用无效栅格地图边界）
  void setSearchXYBounds(double min_x, double max_x,
                         double min_y, double max_y,
                         bool enable = true) {
    if (!enable) {
      search_bounds_enabled_ = false;
      return;
    }
    if (!std::isfinite(min_x) || !std::isfinite(max_x) ||
        !std::isfinite(min_y) || !std::isfinite(max_y) ||
        max_x <= min_x || max_y <= min_y) {
      search_bounds_enabled_ = false;
      return;
    }
    search_x_min_ = min_x;
    search_x_max_ = max_x;
    search_y_min_ = min_y;
    search_y_max_ = max_y;
    search_bounds_enabled_ = true;
  }

  bool searchXYBoundsEnabled() const { return search_bounds_enabled_; }

  // Setter methods
  void setCameraParams(double fx, double fy, double cx, double cy,
                       double width, double height, double max_range = 5.0) {
    cam_fx_ = fx; cam_fy_ = fy;
    cam_cx_ = cx; cam_cy_ = cy;
    cam_width_ = width; cam_height_ = height;
    cam_max_range_ = max_range;
  }

  // 设置视线检查函数
  void setLineOfSightCheckFn(std::function<bool(const Eigen::Vector3d&, const Eigen::Vector3d&)> fn) {
    los_check_fn_ = fn;
  }

  // === 负观测更新：删除在FOV内但未观测到目标的粒子，复制其他粒子补充 ===
  // cam_p: 相机位置, cam_q: 相机姿态
  // 返回被删除的粒子数
  int applyNegativeObservationDelete(const Eigen::Vector3d& cam_p,
                                     const Eigen::Quaterniond& cam_q) {
    if (!search_particles_initialized_ || search_particles_.empty()) {
      return 0;
    }

    double fx = cam_fx_;
    double fy = cam_fy_;
    double cx = cam_cx_;
    double cy = cam_cy_;
    double max_range = cam_max_range_;
    double min_range = 0.1;
    double fov_width = cam_width_;
    double fov_height = cam_height_;

    // 标记需要删除的粒子索引
    std::vector<bool> to_delete(search_particles_.size(), false);
    int delete_count = 0;

    for (size_t i = 0; i < search_particles_.size(); ++i) {
      const auto& particle = search_particles_[i];
      Eigen::Vector3d particle_pos = particle.state.head(3);

      // 将粒子位置转换到相机坐标系
      Eigen::Vector3d p_in_cam = cam_q.inverse() * (particle_pos - cam_p);

      // 检查是否在视场距离范围内
      if (p_in_cam.z() <= min_range || p_in_cam.z() >= max_range) {
        continue;  // 不在距离范围内，保留
      }

      // 投影到图像平面
      double x_img = p_in_cam.x() * fx / p_in_cam.z() + cx;
      double y_img = p_in_cam.y() * fy / p_in_cam.z() + cy;

      // 检查是否在图像范围内
      if (x_img >= 0 && x_img <= fov_width && y_img >= 0 && y_img <= fov_height) {
        // 在FOV内，检查视线是否被遮挡
        bool los_clear = !los_check_fn_ || los_check_fn_(cam_p, particle_pos);
        if (los_clear) {
          // 在FOV内且视线清晰，但没有观测到目标 -> 删除
          to_delete[i] = true;
          delete_count++;
        }
      }
    }

    if (delete_count == 0) {
      return 0;  // 没有需要删除的粒子
    }

    // 收集保留的粒子
    std::vector<SearchParticle> surviving_particles;
    surviving_particles.reserve(search_particles_.size() - delete_count);
    for (size_t i = 0; i < search_particles_.size(); ++i) {
      if (!to_delete[i]) {
        surviving_particles.push_back(search_particles_[i]);
      }
    }

    // 如果所有粒子都被删除，保留原粒子群（避免粒子耗尽）
    if (surviving_particles.empty()) {
      ROS_WARN("[sp_mgr%d] All particles in FOV, keeping original particles", drone_id_);
      return 0;
    }

    // 计算需要复制的粒子数
    int num_to_copy = delete_count;

    // 按权重复制存活粒子来补充
    // 先归一化存活粒子的权重
    double sum_w = 0.0;
    for (const auto& p : surviving_particles) {
      sum_w += p.weight;
    }
    if (sum_w > 1e-300) {
      for (auto& p : surviving_particles) {
        p.weight /= sum_w;
      }
    }

    // 按权重随机选择粒子进行复制
    std::uniform_real_distribution<double> uniform(0.0, 1.0);
    std::vector<SearchParticle> copied_particles;
    copied_particles.reserve(num_to_copy);

    for (int i = 0; i < num_to_copy; ++i) {
      double rand_val = uniform(rng_);
      double cumsum = 0.0;
      size_t selected_idx = 0;

      for (size_t j = 0; j < surviving_particles.size(); ++j) {
        cumsum += surviving_particles[j].weight;
        if (rand_val <= cumsum) {
          selected_idx = j;
          break;
        }
      }

      // 复制选中的粒子，添加小扰动
      SearchParticle new_particle = surviving_particles[selected_idx];
      new_particle.particle_id = search_particles_.size() + i;

      // 添加位置和速度扰动
      std::normal_distribution<double> pos_noise(0.0, 0.3);
      std::normal_distribution<double> vel_noise(0.0, 0.1);
      new_particle.state(0) += pos_noise(rng_);
      new_particle.state(1) += pos_noise(rng_);
      new_particle.state(3) += vel_noise(rng_);
      new_particle.state(4) += vel_noise(rng_);

      copied_particles.push_back(new_particle);
    }

    // 合并存活粒子和复制粒子
    search_particles_.clear();
    search_particles_.reserve(surviving_particles.size() + copied_particles.size());
    search_particles_.insert(search_particles_.end(), surviving_particles.begin(), surviving_particles.end());
    search_particles_.insert(search_particles_.end(), copied_particles.begin(), copied_particles.end());

    // 重新归一化权重
    double total_w = 0.0;
    for (const auto& p : search_particles_) {
      total_w += p.weight;
    }
    if (total_w > 1e-300) {
      for (auto& p : search_particles_) {
        p.weight /= total_w;
      }
    }

    ROS_INFO_THROTTLE(1.0, "[sp_mgr%d] Negative obs: deleted %d particles, copied %d, total=%zu",
                      drone_id_, delete_count, num_to_copy, search_particles_.size());

    return delete_count;
  }

  // === 初始化三个标签的粒子群（搜索模式入口） ===
  // 根据反推的平均位置和速度，为三个意图标签各创建一个粒子群
  // 每个粒子群通过动力学步进 num_frames_back 步来初始化
  void initializeLabeledParticles(const Eigen::Vector3d& mean_pos,
                                  const Eigen::Vector3d& mean_vel,
                                  int num_frames_back) {
    if (!search_particles_initialized_) {
      ROS_WARN("[sp_mgr%d] Search particles not initialized, cannot initialize labeled particles", drone_id_);
      return;
    }

    int num_particles_per_label = search_particles_.size() / 3;
    if (num_particles_per_label == 0) num_particles_per_label = 100;  // 默认每个标签100个粒子

    // 清空现有粒子
    search_particles_.clear();

    // 为三个标签各创建粒子群
    std::vector<SearchIntent> labels = {STRAIGHT, LEFT_TURN, RIGHT_TURN};

    for (SearchIntent label : labels) {
      // 为该标签创建粒子
      for (int i = 0; i < num_particles_per_label; ++i) {
        SearchParticle particle;
        particle.state.setZero(9);
        particle.state.head(3) = mean_pos;
        particle.state.segment(3, 3) = mean_vel;
        particle.state(8) = std::atan2(mean_vel.y(), mean_vel.x());  // yaw
        particle.weight = 1.0 / (3 * num_particles_per_label);
        particle.intent = label;
        particle.particle_id = search_particles_.size();
        search_particles_.push_back(particle);
      }
    }

    // 步进 num_frames_back 步（不考虑意图切换）
    for (int step = 0; step < num_frames_back; ++step) {
      for (auto& particle : search_particles_) {
        updateSearchParticleState(particle);
      }
    }
  }


/* 调试函数已弃用
// 调试：打印粒子分散趋势
void debugPrintParticleDistribution(int frame_num) const {
  if (!search_particles_initialized_) return;
  
  ROS_INFO("\n========== [Frame %d] Search Particles Distribution ==========", frame_num);
  
  // 按标签分组统计
  std::map<SearchIntent, std::vector<size_t>> label_to_indices;
  for (size_t i = 0; i < search_particles_.size(); ++i) {
    label_to_indices[search_particles_[i].intent].push_back(i);
  }
  
  const char* label_names[] = {"STRAIGHT", "LEFT_TURN", "RIGHT_TURN"};
  
  for (int label_int = STRAIGHT; label_int <= RIGHT_TURN; ++label_int) {
    SearchIntent label = static_cast<SearchIntent>(label_int);
    auto it = label_to_indices.find(label);
    if (it == label_to_indices.end() || it->second.empty()) continue;
    
    const auto& indices = it->second;
    int count = indices.size();
    
    // 统计位置和速度范围
    Eigen::Vector3d pos_min = Eigen::Vector3d::Constant(1e9);
    Eigen::Vector3d pos_max = Eigen::Vector3d::Constant(-1e9);
    Eigen::Vector3d vel_min = Eigen::Vector3d::Constant(1e9);
    Eigen::Vector3d vel_max = Eigen::Vector3d::Constant(-1e9);
    Eigen::Vector3d pos_mean = Eigen::Vector3d::Zero();
    Eigen::Vector3d vel_mean = Eigen::Vector3d::Zero();
    double weight_sum = 0.0;
    double weight_max = 0.0;
    
    for (size_t idx : indices) {
      const auto& p = search_particles_[idx];
      Eigen::Vector3d pos = p.state.head(3);
      Eigen::Vector3d vel = p.state.segment(3, 3);
      
      pos_min = pos_min.cwiseMin(pos);
      pos_max = pos_max.cwiseMax(pos);
      vel_min = vel_min.cwiseMin(vel);
      vel_max = vel_max.cwiseMax(vel);
      
      pos_mean += p.weight * pos;
      vel_mean += p.weight * vel;
      weight_sum += p.weight;
      weight_max = std::max(weight_max, p.weight);
    }
    
    if (weight_sum > 1e-300) {
      pos_mean /= weight_sum;
      vel_mean /= weight_sum;
    }
    
    // 计算位置和速度的标准差
    Eigen::Vector3d pos_std = Eigen::Vector3d::Zero();
    Eigen::Vector3d vel_std = Eigen::Vector3d::Zero();
    for (size_t idx : indices) {
      const auto& p = search_particles_[idx];
      Eigen::Vector3d pos = p.state.head(3);
      Eigen::Vector3d vel = p.state.segment(3, 3);
      pos_std += p.weight * (pos - pos_mean).cwiseAbs2();
      vel_std += p.weight * (vel - vel_mean).cwiseAbs2();
    }
    pos_std = (pos_std / weight_sum).cwiseSqrt();
    vel_std = (vel_std / weight_sum).cwiseSqrt();
    
    // 打印标签信息
    ROS_INFO("\n--- Label: %s (Count: %d) ---", label_names[label_int], count);
    ROS_INFO("  Position Mean: (%.3f, %.3f, %.3f)", pos_mean.x(), pos_mean.y(), pos_mean.z());
    ROS_INFO("  Position Std:  (%.3f, %.3f, %.3f)", pos_std.x(), pos_std.y(), pos_std.z());
    ROS_INFO("  Position Range: X[%.2f, %.2f] Y[%.2f, %.2f] Z[%.2f, %.2f]",
             pos_min.x(), pos_max.x(), pos_min.y(), pos_max.y(), pos_min.z(), pos_max.z());
    
    ROS_INFO("  Velocity Mean: (%.3f, %.3f, %.3f)", vel_mean.x(), vel_mean.y(), vel_mean.z());
    ROS_INFO("  Velocity Std:  (%.3f, %.3f, %.3f)", vel_std.x(), vel_std.y(), vel_std.z());
    ROS_INFO("  Velocity Range: X[%.2f, %.2f] Y[%.2f, %.2f] Z[%.2f, %.2f]",
             vel_min.x(), vel_max.x(), vel_min.y(), vel_max.y(), vel_min.z(), vel_max.z());
    
    ROS_INFO("  Weight: Max=%.4f, Sum=%.4f, Avg=%.4f", weight_max, weight_sum, weight_sum / count);
    
    // 打印GMM参数
    ROS_INFO("  GMM Components:");
    for (int c = 0; c < num_components_; ++c) {
      ROS_INFO("    [%d] pi=%.4f, mu=(%.2f,%.2f,%.2f), det(S)=%.2e",
               c, gmm_pi_.at(label)(c),
               gmm_mu_.at(label)[c](0), gmm_mu_.at(label)[c](1), gmm_mu_.at(label)[c](2),
               gmm_S_.at(label)[c].determinant());
    }
  }
  
  ROS_INFO("===========================================================\n");
}
调试函数结束 */

private:
  // 成员变量
  int drone_id_;
  int num_components_; // 【修复2】添加缺失的成员变量
  std::vector<SearchParticle> search_particles_;
  bool search_particles_initialized_;

  // 搜索粒子动力学参数
  double search_dt_;
  double search_vmax_;
  double search_vmin_;
  double search_amax_;
  double intent_keep_prob_;
  double search_pos_noise_;
  double search_vel_noise_;
  double last_yaw_;

  // 相机内参（用于负观测权重更新）
  double cam_fx_, cam_fy_, cam_cx_, cam_cy_;
  double cam_width_, cam_height_, cam_max_range_;

  // 搜索粒子XY边界
  bool search_bounds_enabled_ = false;
  double search_x_min_ = 0.0;
  double search_x_max_ = 0.0;
  double search_y_min_ = 0.0;
  double search_y_max_ = 0.0;

  // 随机数生成器
  std::mt19937 rng_;
  std::vector<double> mixed_turn_omegas_;  // mixed更新中每个粒子的持续角速度(rad/s)
  
  // 按标签存储的共识状态（用于分布式共识）
  std::map<SearchIntent, Eigen::VectorXd> zeta_alpha_;      // 全局alpha共识状态
  std::map<SearchIntent, std::vector<Eigen::VectorXd>> zeta_a_; // 全局a共识状态
  std::map<SearchIntent, std::vector<Eigen::MatrixXd>> zeta_b_; // 全局b共识状态

  // 按标签存储的GMM参数（对齐原DPF）
  std::map<SearchIntent, Eigen::VectorXd> gmm_pi_;
  std::map<SearchIntent, std::vector<Eigen::VectorXd>> gmm_mu_;
  std::map<SearchIntent, std::vector<Eigen::MatrixXd>> gmm_S_;

  // 视线检查回调函数
  std::function<bool(const Eigen::Vector3d&, const Eigen::Vector3d&)> los_check_fn_;

  // === 6D全粒子共识GMM状态 (pos+vel) ===
  static constexpr int nx6_ = 6;
  int search_C_ = 6;  // 搜索模式GMM分量数，初始化时设为2*num_drones
  Eigen::VectorXd search_gmm_pi_;
  std::vector<Eigen::VectorXd> search_gmm_mu_;
  std::vector<Eigen::MatrixXd> search_gmm_S_;
  // 共识状态 zeta
  Eigen::VectorXd search_zeta_alpha_;
  std::vector<Eigen::VectorXd> search_zeta_a_;
  std::vector<Eigen::MatrixXd> search_zeta_b_;
  bool search_gmm_initialized_ = false;
};
