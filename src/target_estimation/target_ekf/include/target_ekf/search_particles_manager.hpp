#pragma once
#include <Eigen/Dense>
#include <vector>
#include <random>
#include <functional>
#include <memory>
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
      num_components_(num_components), // 【修复2】添加缺失的成员变量初始化
      search_particles_initialized_(false),
      search_dt_(1.0 / 20.0),  // 搜索模式更新周期，默认20Hz
      search_vmax_(1.5),        // 最大速度 m/s (默认值)
      search_vmin_(0.5),        // 最小速度 m/s
      search_amax_(2.5),        // 最大加速度 m/s² (默认值)
      intent_keep_prob_(0.8),   // 意图保持概率 80%
      search_pos_noise_(0.1),   // 位置噪声 m
      search_vel_noise_(0.2),   // 速度噪声 m/s
      last_yaw_(0.0),
      cam_fx_(320.0), cam_fy_(320.0), cam_cx_(320.0), cam_cy_(240.0),
      cam_width_(640.0), cam_height_(480.0), cam_max_range_(5.0) {
    // 从参数服务器读取动力学参数
    if (nh) {
      double param_vmax = 1.5;
      double param_amax = 2.5;
      nh->param("/target/planning/vmax", param_vmax, param_vmax);
      nh->param("/target/planning/amax", param_amax, param_amax);
      search_vmax_ = param_vmax;
      search_amax_ = param_amax;
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
        std::uniform_real_distribution<double> uniform_acc(-search_amax_/2, search_amax_);  
        double anom = uniform_acc(rng_);
        double amag = anom + acc_noise(rng_);
        // 硬约束最大加速度
        amag = clamp(amag, -search_amax_, search_amax_);
        // 速度上限约束
        if (v + amag * search_dt_ > search_vmax_) amag = (search_vmax_ - v) / search_dt_;
        acc.head(2) = amag * ev.head(2);
        break;
      }
      case LEFT_TURN: {
        // 左转加速度：垂直速度方向向左
        Eigen::Vector3d eleft = Eigen::Vector3d::Zero();
        eleft.x() = -ev.y();
        eleft.y() = ev.x();
        // 生成一个在 0 到 search_amax 之间，越接近 search_amax 概率越高的分布
        // 使用幂次分布：aturn = search_amax * (u^k)，其中 k < 1
        std::uniform_real_distribution<double> uniform(0.0, 1.0);
        double u = uniform(rng_);
        double k = 0.5; // k 越小，越偏向于 search_amax
        double aturn = search_amax_ * std::pow(u, k);
        double amag = clamp(aturn + acc_noise(rng_), 0.0, search_amax_);
        // akeep 设计为在 -search_amax_/2 和 search_amax_/2 之间正态分布的随机值
        std::normal_distribution<double> akeep_dist(0.0, search_amax_/4); // 均值0，标准差为amax/4
        double akeep = akeep_dist(rng_);
        akeep = clamp(akeep, -search_amax_/2, search_amax_/2);
        acc.head(2) = amag * eleft.head(2) + akeep * ev.head(2);
        // 硬约束总加速度不超过amax
        if (acc.norm() > search_amax_) acc = acc.normalized() * search_amax_;
        break;
      }
      case RIGHT_TURN: {
        // 右转加速度：垂直速度方向向右
        Eigen::Vector3d eright = Eigen::Vector3d::Zero();
        eright.x() = ev.y();
        eright.y() = -ev.x();
        // 生成一个在 0 到 search_amax 之间，越接近 search_amax 概率越高的分布
        // 使用幂次分布：aturn = search_amax * (u^k)，其中 k < 1
        std::uniform_real_distribution<double> uniform(0.0, 1.0);
        double u = uniform(rng_);
        double k = 0.5; // k 越小，越偏向于 search_amax
        double aturn = search_amax_ * std::pow(u, k);
        double amag = clamp(aturn + acc_noise(rng_), 0.0, search_amax_);
        // akeep 设计为在 -search_amax_/2 和 search_amax_/2 之间正态分布的随机值
        std::normal_distribution<double> akeep_dist(0.0, search_amax_/4); // 均值0，标准差为amax/4
        double akeep = akeep_dist(rng_);
        akeep = clamp(akeep, -search_amax_/2, search_amax_/2);
        acc.head(2) = amag * eright.head(2) + akeep* ev.head(2);
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

  // 分标签GMM拟合与分布式共识（核心功能）
  // 对每个标签l，单独筛选该标签下的粒子，计算本地统计量
  std::vector<LabeledLocalStat> computeLabeledLocalStats();
  
  // 对每个标签的本地统计量，独立运行平均共识滤波
  void labeledConsensusFilter(const std::vector<LabeledNeighborConsensus>& neighbor_consensus,
                             const std::vector<LabeledLocalStat>& local_stats);
  
  // 获取每个标签的GMM参数（用于轨迹规划）
  std::map<SearchIntent, std::vector<Eigen::VectorXd>> getLabeledGMMMeans() const;
  std::map<SearchIntent, std::vector<Eigen::MatrixXd>> getLabeledGMMCovs() const;
  std::map<SearchIntent, Eigen::VectorXd> getLabeledGMMPis() const;
  
  // 从GMM采样新粒子（基于标准DPF实现）
  void sampleParticlesFromGMM();
  
  // 计算有效粒子数（用于判断是否需要重采样）
  double computeEffectiveSampleSize() const;
  
  // 单标签共识滤波（内部使用）
  void consensusFilterForLabel(SearchIntent label,
                              const std::vector<LabeledNeighborConsensus>& neighbor_consensus,
                              const LabeledLocalStat& local_stat);
  
  // 单标签全局M步（内部使用）
  void globalMStepForLabel(SearchIntent label);

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

  // Setter methods
  void setDroneId(int drone_id) { drone_id_ = drone_id; }
  void setSearchDt(double dt) { search_dt_ = dt; }
  void setSearchVMax(double vmax) { search_vmax_ = vmax; }
  void setSearchVMin(double vmin) { search_vmin_ = vmin; }
  void setIntentKeepProb(double prob) { intent_keep_prob_ = prob; }
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
};

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

// 单标签共识滤波
inline void SearchParticlesManager::consensusFilterForLabel(SearchIntent label,
                                                           const std::vector<LabeledNeighborConsensus>& neighbor_consensus,
                                                           const LabeledLocalStat& local_stat) {
  int C = num_components_;
  int nx = 9;
  
  // 动态调整共识步长
  int d_max = neighbor_consensus.size() + 1;
  double adaptive_epsilon = 1.0 / d_max;
  adaptive_epsilon = std::min(adaptive_epsilon, 0.3); // 上限0.3保证稳定
  
  // 对齐原DPF的共识迭代次数
  for (int iter = 0; iter < 10; ++iter) {
    for (int c = 0; c < C; ++c) {
      // 计算邻居差值
      double alpha_diff = 0.0;
      Eigen::VectorXd a_diff = Eigen::VectorXd::Zero(nx);
      Eigen::MatrixXd b_diff = Eigen::MatrixXd::Zero(nx, nx);
      
      int valid_neighbor = 0;
      for (const auto& nc : neighbor_consensus) {
        alpha_diff += nc.zeta_alpha(c) - zeta_alpha_[label](c);
        a_diff += nc.zeta_a[c] - zeta_a_[label][c];
        b_diff += nc.zeta_b[c] - zeta_b_[label][c];
        valid_neighbor++;
      }
      
      // 本地统计量处理
      double local_alpha = 0.0;
      Eigen::VectorXd local_a = Eigen::VectorXd::Zero(nx);
      Eigen::MatrixXd local_b = Eigen::MatrixXd::Zero(nx, nx);
      
      if (local_stat.has_obs) {
        local_alpha = local_stat.alpha(c);
        local_a = local_stat.a[c];
        local_b = local_stat.b[c];
      } else {
        local_alpha = zeta_alpha_[label](c);
        local_a = zeta_a_[label][c];
        local_b = zeta_b_[label][c];
      }
      
      // 更新共识状态
      zeta_alpha_[label](c) += adaptive_epsilon * (alpha_diff + (local_alpha - zeta_alpha_[label](c)));
      zeta_a_[label][c] += adaptive_epsilon * (a_diff + (local_a - zeta_a_[label][c]));
      zeta_b_[label][c] += adaptive_epsilon * (b_diff + (local_b - zeta_b_[label][c]));
    }
  }
  
  // 全局M步更新该标签的GMM
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
    }
  }
  
  ROS_DEBUG("[sp_mgr%d] Sampled new particles from GMM", drone_id_);
}
std::map<SearchIntent, Eigen::VectorXd> getLabeledZetaAlpha() const {
  std::map<SearchIntent, Eigen::VectorXd> result;
  for (int label_int = STRAIGHT; label_int <= RIGHT_TURN; ++label_int) {
    SearchIntent label = static_cast<SearchIntent>(label_int);
    result[label] = zeta_alpha_[label];
  }
  return result;
}

std::map<SearchIntent, std::vector<Eigen::VectorXd>> getLabeledZetaA() const {
  std::map<SearchIntent, std::vector<Eigen::VectorXd>> result;
  for (int label_int = STRAIGHT; label_int <= RIGHT_TURN; ++label_int) {
    SearchIntent label = static_cast<SearchIntent>(label_int);
    result[label] = zeta_a_[label];
  }
  return result;
}

std::map<SearchIntent, std::vector<Eigen::MatrixXd>> getLabeledZetaB() const {
  std::map<SearchIntent, std::vector<Eigen::MatrixXd>> result;
  for (int label_int = STRAIGHT; label_int <= RIGHT_TURN; ++label_int) {
    SearchIntent label = static_cast<SearchIntent>(label_int);
    result[label] = zeta_b_[label];
  }
  return result;
}
