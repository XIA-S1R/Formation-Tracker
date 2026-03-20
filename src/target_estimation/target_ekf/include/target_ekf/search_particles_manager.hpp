#pragma once
#include <Eigen/Dense>
#include <vector>
#include <random>
#include <functional>
#include <memory>
#include <algorithm>
#include <cmath>
#include <limits>
#include <ros/time.h>
#include <ros/node_handle.h>
#include <map>

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

// 标签化统计量结构体（用于分标签共识）
struct LabeledLocalStat {
  int drone_id;
  SearchIntent label;     // 标签类型
  int C;        // GMM 分量数
  int nx;       // 状态维度 = 9 (GMM拟合此空间)
  Eigen::VectorXd alpha;              // [C] 对应 kα_{m,c}^t
  std::vector<Eigen::VectorXd> a;     // C 个 [nx] 向量 对应 a_{m,c}^t
  std::vector<Eigen::MatrixXd> b;     // C 个 [nx x nx] 矩阵 对应 b_{m,c}^t
  bool has_obs;
  ros::Time timestamp;  // 时间戳，用于时间同步
};

// 标签化邻居共识状态结构体
struct LabeledNeighborConsensus {
  SearchIntent label;   // 标签类型
  Eigen::VectorXd zeta_alpha;
  std::vector<Eigen::VectorXd> zeta_a;
  std::vector<Eigen::MatrixXd> zeta_b;
  bool has_obs;
  ros::Time timestamp;
};

// 搜索粒子管理类
class SearchParticlesManager {
public:
  // 构造函数
  SearchParticlesManager(int drone_id = 0, int num_components = 4, ros::NodeHandle* nh = nullptr)
    : drone_id_(drone_id),
      num_components_(num_components),
      search_particles_initialized_(false),
      my_label_(static_cast<SearchIntent>(drone_id % 3)),  // 根据ID分配标签
      is_label_master_(drone_id < 3),                       // 前3架是掌管者
      frames_since_init_(0),
      search_dt_(0.2),  // 搜索模式更新周期，5Hz
      search_vmax_(2),        // 最大速度 m/s (默认值)
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
  T clamp(const T& val, const T& min_val, const T& max_val) {
    return std::max(min_val, std::min(val, max_val));
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

  // 获取本无人机负责的标签
  SearchIntent getMyLabel() const {
    return my_label_;
  }

  // 是否是该标签的掌管者
  bool isLabelMaster() const {
    return is_label_master_;
  }

  // 获取标签名称（用于日志）
  static const char* getLabelName(SearchIntent label) {
    static const char* names[] = {"STRAIGHT", "LEFT_TURN", "RIGHT_TURN"};
    return names[static_cast<int>(label)];
  }

  // 设置标签分配结果（由匈牙利算法计算后调用）
  void setLabelAssignment(SearchIntent label, bool is_master) {
    my_label_ = label;
    is_label_master_ = is_master;
    ROS_INFO("[sp_mgr%d] Label assignment updated: %s, is_master=%d",
             drone_id_, getLabelName(label), is_master);
  }

  // 新搜索方案：粒子动力学更新
  // 水平速度方向加扰动，速度大小从[0.5*vmax, vmax]均匀采样，步进dt
  void searchParticlesDynamicsUpdate(Eigen::MatrixXd& particles, int N) {
    const double sigma_theta = 30.0 * M_PI / 180.0;
    const double dt = search_dt_;
    std::normal_distribution<double> theta_dist(0.0, sigma_theta);
    std::uniform_real_distribution<double> speed_dist(0.5 * search_vmax_, search_vmax_);

    for (int i = 0; i < N; ++i) {
      double vx = particles(3, i);
      double vy = particles(4, i);
      double theta_old = std::atan2(vy, vx);
      double theta_new = theta_old + theta_dist(rng_);
      double v_new = speed_dist(rng_);
      particles(3, i) = v_new * std::cos(theta_new);
      particles(4, i) = v_new * std::sin(theta_new);
      particles(0, i) += particles(3, i) * dt;
      particles(1, i) += particles(4, i) * dt;
    }
  }

  // === 3D无效区域GMM结构体 ===
  struct InvalidGMM3D {
    int C = 0;
    Eigen::VectorXd weights;
    std::vector<Eigen::Vector3d> means;
    std::vector<Eigen::Matrix3d> covs;
  };

  // === 粒子分类：找出负观测无效粒子和障碍无效粒子的索引 ===
  // neg_obs_indices: 在FOV内且视线无遮挡的粒子（能看到但没目标）
  // obstacle_indices: 在局部地图障碍内的粒子
  struct ParticleClassification {
    std::vector<int> neg_obs_indices;
    std::vector<int> obstacle_indices;
  };

  // is_occupied: 外部传入的占据检查函数
  ParticleClassification classifyInvalidParticles(
      const Eigen::MatrixXd& particles, int N,
      const Eigen::Vector3d& cam_p, const Eigen::Quaterniond& cam_q,
      std::function<bool(const Eigen::Vector3d&)> is_occupied) {
    ParticleClassification result;
    Eigen::Matrix3d R_cam_inv = cam_q.toRotationMatrix().transpose();

    for (int i = 0; i < N; ++i) {
      Eigen::Vector3d p = particles.col(i).head(3);

      // 障碍检查
      if (is_occupied(p)) {
        result.obstacle_indices.push_back(i);
        continue; // 障碍内的粒子不再做FOV检查
      }

      // FOV + 视线检查
      Eigen::Vector3d p_in_cam = R_cam_inv * (p - cam_p);
      if (p_in_cam.z() > 0.1 && p_in_cam.z() < cam_max_range_) {
        double u = p_in_cam.x() * cam_fx_ / p_in_cam.z() + cam_cx_;
        double v = p_in_cam.y() * cam_fy_ / p_in_cam.z() + cam_cy_;
        if (u >= 0 && u <= cam_width_ && v >= 0 && v <= cam_height_) {
          // 在FOV内，检查视线
          if (los_check_fn_ && los_check_fn_(cam_p, p)) {
            result.neg_obs_indices.push_back(i);
          }
        }
      }
    }
    return result;
  }

  // === 对选定粒子的3D位置拟合GMM（简单EM）===
  InvalidGMM3D fitGMM3D(const Eigen::MatrixXd& particles,
                        const std::vector<int>& indices,
                        int C, int max_iters = 15) {
    InvalidGMM3D gmm;
    int N = indices.size();
    if (N < C || N < 3) {
      gmm.C = 0;
      return gmm;
    }
    gmm.C = C;
    gmm.weights.setConstant(C, 1.0 / C);
    gmm.means.resize(C);
    gmm.covs.resize(C);

    // 初始化：均匀间隔选取粒子作为初始均值
    for (int c = 0; c < C; ++c) {
      int idx = indices[c * N / C];
      gmm.means[c] = particles.col(idx).head(3);
      gmm.covs[c] = Eigen::Matrix3d::Identity() * 0.5;
    }

    // EM迭代
    Eigen::MatrixXd resp(N, C); // 责任度矩阵
    for (int iter = 0; iter < max_iters; ++iter) {
      // E步
      for (int n = 0; n < N; ++n) {
        Eigen::Vector3d p = particles.col(indices[n]).head(3);
        double total = 0.0;
        for (int c = 0; c < C; ++c) {
          Eigen::Vector3d diff = p - gmm.means[c];
          double det = gmm.covs[c].determinant();
          if (det < 1e-30) det = 1e-30;
          Eigen::Matrix3d inv = gmm.covs[c].inverse();
          double exponent = -0.5 * diff.transpose() * inv * diff;
          double nc = 1.0 / std::sqrt(std::pow(2.0 * M_PI, 3) * det);
          resp(n, c) = gmm.weights(c) * nc * std::exp(exponent);
          total += resp(n, c);
        }
        if (total > 1e-300) resp.row(n) /= total;
        else resp.row(n).setConstant(1.0 / C);
      }
      // M步
      for (int c = 0; c < C; ++c) {
        double Nc = resp.col(c).sum();
        if (Nc < 1e-10) continue;
        gmm.weights(c) = Nc / N;
        gmm.means[c].setZero();
        for (int n = 0; n < N; ++n) {
          gmm.means[c] += resp(n, c) * particles.col(indices[n]).head(3);
        }
        gmm.means[c] /= Nc;
        gmm.covs[c].setZero();
        for (int n = 0; n < N; ++n) {
          Eigen::Vector3d diff = particles.col(indices[n]).head(3) - gmm.means[c];
          gmm.covs[c] += resp(n, c) * diff * diff.transpose();
        }
        gmm.covs[c] /= Nc;
        gmm.covs[c] += Eigen::Matrix3d::Identity() * 1e-4; // 正定保证
      }
    }
    // 归一化权重
    double wsum = gmm.weights.sum();
    if (wsum > 1e-300) gmm.weights /= wsum;
    return gmm;
  }

  // === 用无效区域GMM裁剪粒子权重 ===
  // 对每个粒子，计算其3D位置在无效GMM下的概率密度，按比例衰减权重
  // p_threshold: 密度超过此值时权重完全归零
  void pruneParticlesByInvalidGMM(Eigen::MatrixXd& particles,
                                   Eigen::VectorXd& weights, int N,
                                   const InvalidGMM3D& neg_obs_gmm,
                                   const InvalidGMM3D& obstacle_gmm,
                                   double p_threshold = -1.0) {
    // 合并两个无效GMM的所有分量
    std::vector<Eigen::Vector3d> all_mu;
    std::vector<Eigen::Matrix3d> all_cov_inv;
    std::vector<double> all_norm;
    std::vector<double> all_w;

    auto addComponents = [&](const InvalidGMM3D& gmm) {
      for (int c = 0; c < gmm.C; ++c) {
        double det = gmm.covs[c].determinant();
        if (det < 1e-30) continue;
        all_mu.push_back(gmm.means[c]);
        all_cov_inv.push_back(gmm.covs[c].inverse());
        all_norm.push_back(1.0 / std::sqrt(std::pow(2.0 * M_PI, 3) * det));
        all_w.push_back(gmm.weights(c));
      }
    };
    addComponents(neg_obs_gmm);
    addComponents(obstacle_gmm);

    if (all_mu.empty()) return;

    // 自动计算阈值：取所有分量峰值密度的中位数作为参考
    if (p_threshold <= 0) {
      std::vector<double> peaks;
      for (size_t c = 0; c < all_mu.size(); ++c) {
        peaks.push_back(all_w[c] * all_norm[c]); // 分量中心处的密度
      }
      std::sort(peaks.begin(), peaks.end());
      p_threshold = peaks[peaks.size() / 2] * 0.5; // 峰值中位数的一半
      if (p_threshold < 1e-10) p_threshold = 1e-10;
    }

    // 对每个粒子计算无效密度并衰减权重
    for (int i = 0; i < N; ++i) {
      Eigen::Vector3d pos = particles.col(i).head(3);
      double p_invalid = 0.0;
      for (size_t c = 0; c < all_mu.size(); ++c) {
        Eigen::Vector3d diff = pos - all_mu[c];
        double exponent = -0.5 * diff.transpose() * all_cov_inv[c] * diff;
        p_invalid += all_w[c] * all_norm[c] * std::exp(exponent);
      }
      double ratio = p_invalid / p_threshold;
      double factor = std::max(0.0, 1.0 - ratio);
      weights(i) *= factor;
    }

    // 归一化权重
    double wsum = weights.sum();
    if (wsum > 1e-300) {
      weights /= wsum;
    } else {
      weights.setConstant(N, 1.0 / N);
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
  // 目标：优先选择远离无人机且不在高无效密度区域内的热点
  std::vector<SearchHotspot> extractFrontierHotspotsFromParticles(
      const Eigen::MatrixXd& particles,
      const Eigen::VectorXd& weights,
      int N, int K,
      const std::vector<Eigen::Vector3d>& drone_positions,
      const InvalidGMM3D& neg_obs_gmm,
      const InvalidGMM3D& obstacle_gmm,
      double min_drone_dist = 2.0,
      double seed_radius = 1.5,
      double invalid_reject_ratio = 1.0) const {
    std::vector<SearchHotspot> hotspots;
    if (N <= 0 || K <= 0 || particles.cols() <= 0 || weights.size() <= 0) {
      return hotspots;
    }

    const int n = std::min<int>(N, std::min<int>(particles.cols(), weights.size()));
    const double min_dist = std::max(0.0, min_drone_dist);
    const double radius = std::max(0.1, seed_radius);
    const double reject_ratio = std::max(1e-6, invalid_reject_ratio);

    // 组装无效GMM分量（与 pruneParticlesByInvalidGMM 一致的密度定义）
    std::vector<Eigen::Vector3d> all_mu;
    std::vector<Eigen::Matrix3d> all_cov_inv;
    std::vector<double> all_norm;
    std::vector<double> all_w;
    auto add_components = [&](const InvalidGMM3D& gmm) {
      for (int c = 0; c < gmm.C; ++c) {
        double det = gmm.covs[c].determinant();
        if (det < 1e-30) continue;
        all_mu.push_back(gmm.means[c]);
        all_cov_inv.push_back(gmm.covs[c].inverse());
        all_norm.push_back(1.0 / std::sqrt(std::pow(2.0 * M_PI, 3) * det));
        all_w.push_back(gmm.weights(c));
      }
    };
    add_components(neg_obs_gmm);
    add_components(obstacle_gmm);

    double p_threshold = 1e-10;
    const bool use_invalid_filter = !all_mu.empty();
    if (use_invalid_filter) {
      std::vector<double> peaks;
      peaks.reserve(all_mu.size());
      for (size_t c = 0; c < all_mu.size(); ++c) {
        peaks.push_back(all_w[c] * all_norm[c]);
      }
      std::sort(peaks.begin(), peaks.end());
      p_threshold = peaks[peaks.size() / 2] * 0.5;
      if (p_threshold < 1e-10) p_threshold = 1e-10;
    }

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
        double p_invalid = 0.0;
        for (size_t c = 0; c < all_mu.size(); ++c) {
          Eigen::Vector3d diff = pos - all_mu[c];
          double exponent = -0.5 * diff.transpose() * all_cov_inv[c] * diff;
          p_invalid += all_w[c] * all_norm[c] * std::exp(exponent);
        }
        double ratio = p_invalid / p_threshold;
        if (ratio >= reject_ratio) {
          continue;
        }
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

  /*// Setter methods
  void setDroneId(int drone_id) { drone_id_ = drone_id; }
  void setSearchDt(double dt) { search_dt_ = dt; }
  double getSearchVmax() const { return search_vmax_; }
  void setSearchVMax(double vmax) { search_vmax_ = vmax; }
  void setSearchVMin(double vmin) { search_vmin_ = vmin; }
  void setIntentKeepProb(double prob) { intent_keep_prob_ = prob; }
  void setCameraParams(double fx, double fy, double cx, double cy,
                       double width, double height, double max_range = 5.0) {
    cam_fx_ = fx; cam_fy_ = fy;
    cam_cx_ = cx; cam_cy_ = cy;
    cam_width_ = width; cam_height_ = height;
    cam_max_range_ = max_range;
  }*/
  // 设置视线检查函数
  void setLineOfSightCheckFn(std::function<bool(const Eigen::Vector3d&, const Eigen::Vector3d&)> fn) {
    los_check_fn_ = fn;
  }

  // === 初始化单标签的粒子群（新方法：无共识） ===
  // 只有掌管者才初始化粒子群，根据反推的平均位置和速度，按意图动力学步进
  void initializeSingleLabelParticles(const Eigen::Vector3d& mean_pos,
                                      const Eigen::Vector3d& mean_vel,
                                      int num_frames_back,
                                      int num_particles = 300) {
    if (!is_label_master_) {
      ROS_INFO("[sp_mgr%d] Not a label master for %s, skip initialization",
               drone_id_, getLabelName(my_label_));
      return;
    }

    // 清空现有粒子，重置帧计数
    search_particles_.clear();
    search_particles_.reserve(num_particles);
    frames_since_init_ = 0;

    // 只为本无人机负责的标签创建粒子群
    for (int i = 0; i < num_particles; ++i) {
      SearchParticle particle;
      particle.state.setZero(9);
      particle.state.head(3) = mean_pos;
      particle.state.segment(3, 3) = mean_vel;
      particle.state(8) = std::atan2(mean_vel.y(), mean_vel.x());  // yaw
      particle.weight = 1.0 / num_particles;
      particle.intent = my_label_;  // 初始都是本标签
      particle.particle_id = i;
      search_particles_.push_back(particle);
    }

    double v_horiz = mean_vel.head(2).norm();
    double yaw_init = std::atan2(mean_vel.y(), mean_vel.x());
    ROS_INFO("[sp_mgr%d] Initialized %d particles for label %s, mean_vel=(%.3f,%.3f,%.3f), v_horiz=%.3f, yaw=%.3f deg",
             drone_id_, num_particles, getLabelName(my_label_),
             mean_vel.x(), mean_vel.y(), mean_vel.z(), v_horiz, yaw_init * 180.0 / M_PI);

    // 记录初始位置
    Eigen::Vector3d init_pos = mean_pos;

    // 步进 num_frames_back 步（初始化阶段不切换意图）
    for (int step = 0; step < num_frames_back; ++step) {
      for (auto& particle : search_particles_) {
        updateSearchParticleState(particle);
      }
    }

    search_particles_initialized_ = true;

    // 计算步进后的均值位置
    Eigen::Vector3d final_mean_pos = getSingleLabelMeanPosition();
    Eigen::Vector3d displacement = final_mean_pos - init_pos;
    double disp_angle = std::atan2(displacement.y(), displacement.x()) * 180.0 / M_PI;
    double vel_angle = yaw_init * 180.0 / M_PI;

    ROS_INFO("[sp_mgr%d] Label %s: %d particles stepped %d frames, displacement=(%.3f,%.3f,%.3f), disp_angle=%.1f deg, vel_angle=%.1f deg, diff=%.1f deg",
             drone_id_, getLabelName(my_label_), num_particles, num_frames_back,
             displacement.x(), displacement.y(), displacement.z(),
             disp_angle, vel_angle, disp_angle - vel_angle);
  }

  // === 计算当前保持意图的概率（随帧数衰减：90% -> 60%） ===
  double computeKeepIntentProb() const {
    // 初始90%，衰减到60%，衰减时间常数约100帧
    const double init_prob = 0.90;
    const double min_prob = 0.60;
    const double decay_rate = 0.02;  // 每帧衰减率

    double prob = init_prob - decay_rate * frames_since_init_;
    return std::max(min_prob, prob);
  }

  // === 意图切换（带衰减概率） ===
  void switchIntentsWithDecay() {
    if (!search_particles_initialized_ || search_particles_.empty()) {
      return;
    }

    double keep_prob = computeKeepIntentProb();
    double switch_prob = (1.0 - keep_prob) / 2.0;  // 平均分配给另外两个意图

    std::uniform_real_distribution<double> uniform(0.0, 1.0);

    for (auto& particle : search_particles_) {
      double rand_val = uniform(rng_);
      SearchIntent old_intent = particle.intent;

      if (rand_val < keep_prob) {
        // 保持原意图
        continue;
      } else if (rand_val < keep_prob + switch_prob) {
        // 切换到第一个其他意图
        if (old_intent == STRAIGHT) particle.intent = LEFT_TURN;
        else if (old_intent == LEFT_TURN) particle.intent = STRAIGHT;
        else particle.intent = STRAIGHT;
      } else {
        // 切换到第二个其他意图
        if (old_intent == STRAIGHT) particle.intent = RIGHT_TURN;
        else if (old_intent == LEFT_TURN) particle.intent = RIGHT_TURN;
        else particle.intent = LEFT_TURN;
      }
    }
  }

  // === 更新单标签粒子群（新方法：带意图切换） ===
  void updateSingleLabelParticles() {
    if (!is_label_master_ || !search_particles_initialized_) {
      return;
    }

    // 步骤1：意图切换（带衰减概率）
    switchIntentsWithDecay();

    // 步骤2：更新粒子状态
    for (auto& particle : search_particles_) {
      updateSearchParticleState(particle);
    }

    // 增加帧计数
    frames_since_init_++;

    ROS_DEBUG("[sp_mgr%d] Updated %zu particles, keep_prob=%.2f, frame=%d",
              drone_id_, search_particles_.size(), computeKeepIntentProb(), frames_since_init_);
  }

  // === 获取单标签粒子群的加权均值位置（用于规划目标点） ===
  Eigen::Vector3d getSingleLabelMeanPosition() const {
    if (search_particles_.empty()) {
      return Eigen::Vector3d::Zero();
    }

    Eigen::Vector3d mean_pos = Eigen::Vector3d::Zero();
    double total_weight = 0.0;

    for (const auto& p : search_particles_) {
      mean_pos += p.weight * p.state.head(3);
      total_weight += p.weight;
    }

    if (total_weight > 1e-300) {
      mean_pos /= total_weight;
    }

    return mean_pos;
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

  // === 旧版初始化函数（保留兼容，后续弃用） ===
  struct SearchTargets {
    Eigen::Vector3d forward;      // 沿速度方向前进
    Eigen::Vector3d left_45;      // 左前45度
    Eigen::Vector3d right_45;     // 右前45度
    Eigen::Vector3d mean_pos;     // 均值位置（反推后）
    Eigen::Vector3d mean_vel;     // 均值速度
  };

  SearchTargets computeSearchTargets(const Eigen::MatrixXd& dpf_particles,
                                     const Eigen::VectorXd& dpf_weights,
                                     int num_frames_back,
                                     double search_distance = 5.0) {
    SearchTargets targets;
    int N = dpf_particles.cols();
    double dt = search_dt_;

    // 步骤1：计算当前粒子的加权均值位置和速度
    Eigen::Vector3d pos_mean = Eigen::Vector3d::Zero();
    Eigen::Vector3d vel_mean = Eigen::Vector3d::Zero();
    double total_weight = 0.0;

    for (int i = 0; i < N; ++i) {
      pos_mean += dpf_weights(i) * dpf_particles.col(i).head(3);
      vel_mean += dpf_weights(i) * dpf_particles.col(i).segment(3, 3);
      total_weight += dpf_weights(i);
    }

    if (total_weight > 1e-300) {
      pos_mean /= total_weight;
      vel_mean /= total_weight;
    }

    // 步骤2：反推num_frames_back帧前的位置（使用匀速模型）
    Eigen::Vector3d pos_back = pos_mean - vel_mean * dt * num_frames_back;

    targets.mean_pos = pos_back;
    targets.mean_vel = vel_mean;

    // 步骤3：计算三个搜索目标点
    Eigen::Vector3d vel_horiz = vel_mean;
    vel_horiz.z() = 0.0;
    double v_horiz = vel_horiz.norm();

    Eigen::Vector3d forward_dir = Eigen::Vector3d::Zero();
    if (v_horiz > 0.1) {
      forward_dir = vel_horiz.normalized();
    } else {
      forward_dir = Eigen::Vector3d(1.0, 0.0, 0.0);
    }

    // 左前45度方向：旋转-45度
    double angle_45 = M_PI / 4.0;
    Eigen::Vector3d left_45_dir;
    left_45_dir.x() = forward_dir.x() * std::cos(angle_45) - forward_dir.y() * std::sin(angle_45);
    left_45_dir.y() = forward_dir.x() * std::sin(angle_45) + forward_dir.y() * std::cos(angle_45);
    left_45_dir.z() = 0.0;

    // 右前45度方向：旋转+45度
    Eigen::Vector3d right_45_dir;
    right_45_dir.x() = forward_dir.x() * std::cos(-angle_45) - forward_dir.y() * std::sin(-angle_45);
    right_45_dir.y() = forward_dir.x() * std::sin(-angle_45) + forward_dir.y() * std::cos(-angle_45);
    right_45_dir.z() = 0.0;

    // 三个搜索点
    targets.forward = pos_back + search_distance * forward_dir;
    targets.left_45 = pos_back + search_distance * left_45_dir;
    targets.right_45 = pos_back + search_distance * right_45_dir;

    // 保持z高度
    targets.forward.z() = pos_back.z();
    targets.left_45.z() = pos_back.z();
    targets.right_45.z() = pos_back.z();

    return targets;
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

  // 单标签模式：每个无人机只掌管一个标签
  SearchIntent my_label_;        // 本无人机负责的标签
  bool is_label_master_;         // 是否是该标签的掌管者（drone_id < 3）
  int frames_since_init_;        // 初始化后经过的帧数（用于意图切换概率衰减）
  
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

  // 随机数生成器
  std::mt19937 rng_;
  
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

// ========== 以下为旧的标签化共识滤波实现（已弃用，保留供参考）==========
#if 0  // 旧代码开始

// ================== 分标签GMM拟合与分布式共识实现 ==================
// 对每个标签l，单独筛选该标签下的粒子，计算本地统计量
inline std::vector<LabeledLocalStat> SearchParticlesManager::computeLabeledLocalStats() {
  std::vector<LabeledLocalStat> labeled_stats;
  int nx = 9;
  int C = num_components_;

  // 按标签分组粒子
  std::map<SearchIntent, std::vector<size_t>> label_to_indices;
  for (size_t i = 0; i < search_particles_.size(); ++i) {
    label_to_indices[search_particles_[i].intent].push_back(i);
  }

  // 对每个标签，严格对齐原DPF的EM-E步计算统计量
  for (int label_int = STRAIGHT; label_int <= RIGHT_TURN; ++label_int) {
    SearchIntent label = static_cast<SearchIntent>(label_int);
    auto it = label_to_indices.find(label);
    if (it == label_to_indices.end() || it->second.empty()) continue;

    const std::vector<size_t>& indices = it->second;
    LabeledLocalStat stat;
    stat.drone_id = drone_id_;
    stat.label = label;
    stat.C = C;
    stat.nx = nx;
    stat.has_obs = false;
    stat.timestamp = ros::Time::now();
    stat.alpha.setZero(C);
    stat.a.resize(C);
    stat.b.resize(C);
    for (int c = 0; c < C; ++c) {
      stat.a[c].setZero(nx);
      stat.b[c].setZero(nx, nx);
    }

    // 预计算该标签GMM的逆和行列式（对齐原DPF）
    std::vector<Eigen::MatrixXd> S_inv(C);
    std::vector<double> S_det(C);
    for (int c = 0; c < C; ++c) {
      S_inv[c] = gmm_S_[label][c].inverse();
      S_det[c] = gmm_S_[label][c].determinant();
      if (S_det[c] < 1e-300) S_det[c] = 1e-300;
    }

    // 严格按原论文公式(17)计算责任度，公式(21)累加统计量
    for (size_t idx : indices) {
      const SearchParticle& particle = search_particles_[idx];
      Eigen::VectorXd x_n = particle.state;
      double w_n = particle.weight;

      // 计算每个高斯分量的责任度
      Eigen::VectorXd resp(C);
      double resp_sum = 0.0;
      for (int c = 0; c < C; ++c) {
        // 9维状态残差计算（对齐原DPF的stateDiff）
        Eigen::VectorXd diff(nx);
        diff.head(3) = x_n.head(3) - gmm_mu_[label][c].head(3);
        diff.segment(3, 3) = x_n.segment(3, 3) - gmm_mu_[label][c].segment(3, 3);
        diff(6) = angleDiff(x_n(6), gmm_mu_[label][c](6));
        diff(7) = angleDiff(x_n(7), gmm_mu_[label][c](7));
        diff(8) = angleDiff(x_n(8), gmm_mu_[label][c](8));

        double exponent = -0.5 * diff.transpose() * S_inv[c] * diff;
        double nc = 1.0 / std::sqrt(std::pow(2.0 * M_PI, nx) * S_det[c]);
        resp(c) = gmm_pi_[label](c) * nc * std::exp(exponent);
        resp_sum += resp(c);
      }

      // 归一化责任度
      if (resp_sum > 1e-300) resp /= resp_sum;
      else resp.setConstant(1.0 / C);

      // 累加本地统计量（对齐原论文公式21）
      for (int c = 0; c < C; ++c) {
        double alpha_nc = w_n * resp(c);
        stat.alpha(c) += alpha_nc;
        stat.a[c] += alpha_nc * x_n;
        Eigen::VectorXd diff_c(nx);
        diff_c.head(3) = x_n.head(3) - gmm_mu_[label][c].head(3);
        diff_c.segment(3, 3) = x_n.segment(3, 3) - gmm_mu_[label][c].segment(3, 3);
        diff_c(6) = angleDiff(x_n(6), gmm_mu_[label][c](6));
        diff_c(7) = angleDiff(x_n(7), gmm_mu_[label][c](7));
        diff_c(8) = angleDiff(x_n(8), gmm_mu_[label][c](8));
        stat.b[c] += alpha_nc * diff_c * diff_c.transpose();
      }
    }

    labeled_stats.push_back(stat);
  }

  return labeled_stats;
}

// 对每个标签的本地统计量，独立运行平均共识滤波
inline void SearchParticlesManager::labeledConsensusFilter(const std::vector<LabeledNeighborConsensus>& neighbor_consensus,
  const std::vector<LabeledLocalStat>& local_stats) {
// 遍历每个标签，对每个标签独立运行共识滤波
for (int label_int = STRAIGHT; label_int <= RIGHT_TURN; ++label_int) {
SearchIntent label = static_cast<SearchIntent>(label_int);

// 找到该标签对应的本地统计量
const LabeledLocalStat* local_stat = nullptr;
for (const auto& stat : local_stats) {
if (stat.label == label) {
local_stat = &stat;
break;
}
}

if (local_stat) {
consensusFilterForLabel(label, neighbor_consensus, *local_stat);
}
}
}

// 对每个标签的本地统计量，独立运行平均共识滤波
inline void SearchParticlesManager::consensusFilterForLabel(SearchIntent label,
  const std::vector<LabeledNeighborConsensus>& neighbor_consensus,
  const LabeledLocalStat& local_stat) {
  int C = num_components_;
  int nx = 9;

  int d_max = neighbor_consensus.size() + 1;
  double adaptive_epsilon = 1.0 / d_max;
  adaptive_epsilon = std::min(adaptive_epsilon, 0.3);

  for (int iter = 0; iter < 10; ++iter) {
  for (int c = 0; c < C; ++c) {
  // 邻居ζ的差值：Σ_j(ζ_j^t - ζ^t)
  double alpha_diff = 0.0;
  Eigen::VectorXd a_diff = Eigen::VectorXd::Zero(nx);
  Eigen::MatrixXd b_diff = Eigen::MatrixXd::Zero(nx, nx);

  for (const auto& nc : neighbor_consensus) {
  alpha_diff += nc.zeta_alpha(c) - zeta_alpha_[label](c);
  a_diff += nc.zeta_a[c] - zeta_a_[label][c];
  b_diff += nc.zeta_b[c] - zeta_b_[label][c];
  }

  // ✅ 直接用本地u，不做has_obs判断
  // 这样就是标准论文公式26：ζ += ε * (邻居差值 + (u - ζ))
  double local_alpha = local_stat.alpha(c);
  Eigen::VectorXd local_a = local_stat.a[c];
  Eigen::MatrixXd local_b = local_stat.b[c];

  zeta_alpha_[label](c) += adaptive_epsilon * (alpha_diff + (local_alpha - zeta_alpha_[label](c)));
  zeta_a_[label][c] += adaptive_epsilon * (a_diff + (local_a - zeta_a_[label][c]));
  zeta_b_[label][c] += adaptive_epsilon * (b_diff + (local_b - zeta_b_[label][c]));
  }
  }

  globalMStepForLabel(label);
  }


// 单标签全局M步
inline void SearchParticlesManager::globalMStepForLabel(SearchIntent label) {
  int C = num_components_;
  int nx = 9;
  
  double alpha_sum = zeta_alpha_[label].sum();
  if (alpha_sum < 1e-300) return;
  
  // 对齐原论文公式(30)，用共识后的全局统计量更新GMM参数
  for (int c = 0; c < C; ++c) {
    if (zeta_alpha_[label](c) < 1e-300) continue;
    
    // 混合权重
    gmm_pi_[label](c) = zeta_alpha_[label](c) / alpha_sum;
    // 均值
    gmm_mu_[label][c] = zeta_a_[label][c] / zeta_alpha_[label](c);
    // 角度归一化
    gmm_mu_[label][c](6) = wrapAngle(gmm_mu_[label][c](6));
    gmm_mu_[label][c](7) = wrapAngle(gmm_mu_[label][c](7));
    gmm_mu_[label][c](8) = wrapAngle(gmm_mu_[label][c](8));
    // 协方差
    gmm_S_[label][c] = zeta_b_[label][c] / zeta_alpha_[label](c);
    // 保证协方差正定
    gmm_S_[label][c] += Eigen::MatrixXd::Identity(nx, nx) * 1e-4;
  }
  
  // 归一化混合权重
  double pi_sum = gmm_pi_[label].sum();
  if (pi_sum > 1e-300) gmm_pi_[label] /= pi_sum;
}

// 获取每个标签的GMM参数（用于轨迹规划）
inline std::map<SearchIntent, std::vector<Eigen::VectorXd>> SearchParticlesManager::getLabeledGMMMeans() const {
  std::map<SearchIntent, std::vector<Eigen::VectorXd>> labeled_means;
  for (int label_int = STRAIGHT; label_int <= RIGHT_TURN; ++label_int) {
    SearchIntent label = static_cast<SearchIntent>(label_int);
    labeled_means[label] = gmm_mu_.at(label);
  }
  return labeled_means;
}

inline std::map<SearchIntent, std::vector<Eigen::MatrixXd>> SearchParticlesManager::getLabeledGMMCovs() const {
  std::map<SearchIntent, std::vector<Eigen::MatrixXd>> labeled_covs;
  for (int label_int = STRAIGHT; label_int <= RIGHT_TURN; ++label_int) {
    SearchIntent label = static_cast<SearchIntent>(label_int);
    labeled_covs[label] = gmm_S_.at(label);
  }
  return labeled_covs;
}

inline std::map<SearchIntent, Eigen::VectorXd> SearchParticlesManager::getLabeledGMMPis() const {
  std::map<SearchIntent, Eigen::VectorXd> labeled_pis;
  for (int label_int = STRAIGHT; label_int <= RIGHT_TURN; ++label_int) {
    SearchIntent label = static_cast<SearchIntent>(label_int);
    labeled_pis[label] = gmm_pi_.at(label);
  }
  return labeled_pis;
}

// 计算有效粒子数（用于判断是否需要重采样）
inline double SearchParticlesManager::computeEffectiveSampleSize() const {
  if (search_particles_.empty()) return 0.0;
  
  double sum_weights = 0.0;
  double sum_weight_squares = 0.0;
  
  for (const auto& particle : search_particles_) {
    sum_weights += particle.weight;
    sum_weight_squares += particle.weight * particle.weight;
  }
  
  if (sum_weights > 1e-300 && sum_weight_squares > 1e-300) {
    return sum_weights * sum_weights / sum_weight_squares;
  }
  
  return 0.0;
}

// 从GMM采样新粒子（基于标准DPF实现）
inline void SearchParticlesManager::sampleParticlesFromGMM() {
  if (!search_particles_initialized_ || search_particles_.empty()) return;
  
  int nx = 9; // 9维状态空间
  
  // 按标签分组粒子
  std::map<SearchIntent, std::vector<size_t>> label_to_indices;
  for (size_t i = 0; i < search_particles_.size(); ++i) {
    label_to_indices[search_particles_[i].intent].push_back(i);
  }
  
  // 对每个标签执行GMM采样（类似于标准DPF的实现）
  for (const auto& pair : label_to_indices) {
    SearchIntent label = pair.first;
    const std::vector<size_t>& indices = pair.second;
    
    if (indices.empty()) continue;
    
    int label_particle_count = indices.size();
    int C = num_components_;
    
    // 从该标签的GMM采样新粒子
    std::uniform_real_distribution<double> uniform(0.0, 1.0);
    std::normal_distribution<double> normal(0.0, 1.0);
    
    for (int i = 0; i < label_particle_count; ++i) {
      size_t idx = indices[i];
      SearchParticle& particle = search_particles_[idx];
      
      // 按混合权重选择GMM分量
      double u = uniform(rng_);
      double cum = 0.0;
      int c_sel = C - 1;
      for (int c = 0; c < C; ++c) {
        cum += gmm_pi_[label](c);
        if (u <= cum) { c_sel = c; break; }
      }
      
      // 从GMM采样9维完整状态
      Eigen::LLT<Eigen::MatrixXd> llt(gmm_S_[label][c_sel]);
      Eigen::MatrixXd L = llt.matrixL();
      Eigen::VectorXd noise_x(nx);
      for (int j = 0; j < nx; ++j) noise_x(j) = normal(rng_);
      Eigen::VectorXd x_sample = gmm_mu_[label][c_sel] + L * noise_x;
      
      // 直接赋值9维完整状态
      particle.state = x_sample;
      // 归一化角度
      particle.state(6) = wrapAngle(particle.state(6));
      particle.state(7) = wrapAngle(particle.state(7));
      particle.state(8) = wrapAngle(particle.state(8));
      // ✅ 重采样后重置权重为均匀分布
      particle.weight = 1.0 / label_particle_count;
    }
  }
  
  ROS_DEBUG("[sp_mgr%d] Sampled new particles from GMM", drone_id_);
}
inline std::map<SearchIntent, Eigen::VectorXd> SearchParticlesManager::getLabeledZetaAlpha() const {
  std::map<SearchIntent, Eigen::VectorXd> result;
  for (int label_int = STRAIGHT; label_int <= RIGHT_TURN; ++label_int) {
    SearchIntent label = static_cast<SearchIntent>(label_int);
     result.insert({label, zeta_alpha_.at(label)});
  }
  return result;
}

inline std::map<SearchIntent, std::vector<Eigen::VectorXd>> SearchParticlesManager::getLabeledZetaA() const {
  std::map<SearchIntent, std::vector<Eigen::VectorXd>> result;
  for (int label_int = STRAIGHT; label_int <= RIGHT_TURN; ++label_int) {
    SearchIntent label = static_cast<SearchIntent>(label_int);
    result.insert({label, zeta_a_.at(label)});
  }
  return result;
}

inline std::map<SearchIntent, std::vector<Eigen::MatrixXd>> SearchParticlesManager::getLabeledZetaB() const {
  std::map<SearchIntent, std::vector<Eigen::MatrixXd>> result;
  for (int label_int = STRAIGHT; label_int <= RIGHT_TURN; ++label_int) {
    SearchIntent label = static_cast<SearchIntent>(label_int);
    result.insert({label, zeta_b_.at(label)});
  }
  return result;
}

#endif  // 旧代码结束
// ========== 旧的标签化共识滤波实现结束 ==========
