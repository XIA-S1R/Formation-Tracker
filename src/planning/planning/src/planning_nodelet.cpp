#include <geometry_msgs/PoseStamped.h>
#include <mapping/mapping.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseArray.h>
#include <nodelet/nodelet.h>
#include <quadrotor_msgs/OccMap3d.h>
#include <quadrotor_msgs/PolyTraj.h>
#include <quadrotor_msgs/ReplanState.h>
#include <ros/package.h>
#include <ros/ros.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Bool.h>
#include <traj_opt/traj_opt.h>
#include <target_ekf/SearchLabelInfo.h>

#include <Eigen/Core>
#include <atomic>
#include <algorithm>
#include <cmath>
#include <mutex>
#include <string>
#include <env/env.hpp>
#include <prediction/prediction.hpp>
#include <thread>
#include <visualization/visualization.hpp>
#include <wr_msg/wr_msg.hpp>

namespace planning {

Eigen::IOFormat CommaInitFmt(Eigen::StreamPrecision, Eigen::DontAlignCols, ", ", ", ", "", "", " << ", ";");

// SwarmTrajData is defined in traj_opt/traj_opt.h — no local redefinition needed.

class Nodelet : public nodelet::Nodelet {
 private:
  std::thread initThread_;
  ros::Subscriber gridmap_sub_, odom_sub_, target_sub_, triger_sub_, land_triger_sub_;
  ros::Timer plan_timer_;

  ros::Publisher traj_pub_, heartbeat_pub_, replanState_pub_;
  ros::Publisher broadcast_traj_pub_;
  ros::Subscriber search_state_sub_;
  ros::Subscriber search_targets_sub_;  // 搜索目标点订阅器
  std::vector<ros::Subscriber> search_label_info_subs_;  // 各掌管者的标签信息订阅器

  std::shared_ptr<mapping::OccGridMap> gridmapPtr_;
  std::shared_ptr<env::Env> envPtr_;
  std::shared_ptr<visualization::Visualization> visPtr_;
  std::shared_ptr<traj_opt::TrajOpt> trajOptPtr_;
  std::shared_ptr<prediction::Predict> prePtr_;

  // 搜索模式相关
  bool search_mode_active_ = false;
  std::atomic_flag search_state_lock_ = ATOMIC_FLAG_INIT;
  std_msgs::Bool latest_search_state_;
  geometry_msgs::PoseArray search_targets_;  // 三个搜索目标点
  std::atomic_flag search_targets_lock_ = ATOMIC_FLAG_INIT;

  // 搜索标签信息（从各掌管者接收）
  std::map<int, target_ekf::SearchLabelInfo> search_label_infos_;  // label -> info
  std::atomic_flag search_label_info_lock_ = ATOMIC_FLAG_INIT;
  Eigen::Vector3d search_target_with_offset_;  // 本机的搜索目标点（含偏置）
  double search_desired_yaw_ = 0.0;            // 搜索模式期望偏航角
  double search_yaw_scan_range_deg_ = 45.0;    // 搜索模式偏航扫描半幅（度）
  double search_yaw_scan_freq_hz_ = 0.25;       // 搜索模式偏航扫描频率（Hz）
  double post_reacquire_boost_sec_ = 3.0;      // 退出搜索后的增强跟踪窗口
  bool post_reacquire_boost_active_ = false;   // 增强跟踪是否激活
  ros::Time post_reacquire_boost_end_time_ = ros::Time(0);

  // NOTE planning or fake target
  bool fake_ = false;
  Eigen::Vector3d goal_;
  Eigen::Vector3d land_p_;
  Eigen::Quaterniond land_q_;

  // NOTE just for debug
  bool debug_ = false;
  quadrotor_msgs::ReplanState replanStateMsg_;
  ros::Publisher gridmap_pub_, inflate_gridmap_pub_;
  quadrotor_msgs::OccMap3d occmap_msg_;

  double tracking_dur_, tracking_dist_, tolerance_d_;
  double formation_heading_speed_thresh_ = 0.2;
  double hard_replan_hz_ = 5.0;
  double hard_replan_interval_ = 0.2;
  bool obs_clip_enable_ = true;
  double obs_clip_range_ = 10.0;
  double obs_clip_margin_ = 0.5;
  ros::Time last_hard_replan_stamp_ = ros::Time(0);
  double last_formation_heading_ = 0.0;
  bool last_formation_heading_valid_ = false;
  double vmax_, amax_;

  Trajectory traj_poly_;
  ros::Time replan_stamp_;
  int traj_id_ = 0;
  bool wait_hover_ = true;
  bool force_hover_ = true;
  bool has_published_motion_traj_ = false;
  int execute_last_traj_streak_ = 0;
  ros::Time execute_last_traj_first_stamp_ = ros::Time(0);
  std::string last_validcheck_fail_reason_ = "none";
  double last_validcheck_fail_t_ = -1.0;
  Eigen::Vector3d last_validcheck_fail_p_ = Eigen::Vector3d::Zero();

  nav_msgs::Odometry odom_msg_, target_msg_;
  Eigen::Vector3d last_target_p_ = Eigen::Vector3d::Zero();
  bool last_target_p_valid_ = false;
  quadrotor_msgs::OccMap3d map_msg_;
  std::atomic_flag odom_lock_ = ATOMIC_FLAG_INIT;
  std::atomic_flag target_lock_ = ATOMIC_FLAG_INIT;
  std::atomic_flag gridmap_lock_ = ATOMIC_FLAG_INIT;
  std::atomic_bool odom_received_ = ATOMIC_VAR_INIT(false);
  std::atomic_bool map_received_ = ATOMIC_VAR_INIT(false);
  std::atomic_bool triger_received_ = ATOMIC_VAR_INIT(false);
  std::atomic_bool target_received_ = ATOMIC_VAR_INIT(false);
  std::atomic_bool land_triger_received_ = ATOMIC_VAR_INIT(false);

  std::vector<SwarmTrajData> swarm_trajs_;
  std::mutex swarm_trajs_mutex_;
  ros::Subscriber broadcast_traj_sub_;

  bool clipPathToObservationRange(std::vector<Eigen::Vector3d>& path,
                                  const Eigen::Vector3d& sensor_p) const {
    if (!obs_clip_enable_ || path.size() <= 1) {
      return false;
    }

    const double clip_r = std::max(0.5, obs_clip_range_ - obs_clip_margin_);
    const double clip_r2 = clip_r * clip_r;
    auto is_inside = [&](const Eigen::Vector3d& p) {
      return (p - sensor_p).squaredNorm() <= clip_r2;
    };

    std::vector<Eigen::Vector3d> clipped;
    clipped.reserve(path.size());
    clipped.push_back(path.front());

    bool clipped_any = false;
    for (size_t i = 1; i < path.size(); ++i) {
      const Eigen::Vector3d& p0 = path[i - 1];
      const Eigen::Vector3d& p1 = path[i];
      const bool p0_in = is_inside(p0);
      const bool p1_in = is_inside(p1);

      if (p1_in) {
        clipped.push_back(p1);
        continue;
      }

      clipped_any = true;
      if (p0_in) {
        Eigen::Vector3d d = p1 - p0;
        double a = d.dot(d);
        if (a > 1e-9) {
          Eigen::Vector3d f = p0 - sensor_p;
          double b = 2.0 * f.dot(d);
          double c = f.dot(f) - clip_r2;
          double disc = b * b - 4.0 * a * c;
          if (disc >= 0.0) {
            double sqrt_disc = std::sqrt(disc);
            double t1 = (-b - sqrt_disc) / (2.0 * a);
            double t2 = (-b + sqrt_disc) / (2.0 * a);
            double t = 1.0;
            bool has_t = false;
            if (t1 >= 0.0 && t1 <= 1.0) {
              t = t1;
              has_t = true;
            }
            if (t2 >= 0.0 && t2 <= 1.0 && (!has_t || t2 > t)) {
              t = t2;
              has_t = true;
            }
            if (has_t) {
              Eigen::Vector3d hit = p0 + t * d;
              if ((hit - clipped.back()).norm() > 1e-3) {
                clipped.push_back(hit);
              }
            }
          }
        }
      }
      break;
    }

    if (!clipped_any) {
      return false;
    }

    if (clipped.size() < 2) {
      Eigen::Vector3d dir = path.back() - clipped.front();
      if (dir.norm() < 1e-6) {
        dir = Eigen::Vector3d::UnitX();
      }
      clipped.push_back(sensor_p + clip_r * dir.normalized());
    }

    path.swap(clipped);
    return true;
  }

  void pub_hover_p(const Eigen::Vector3d& hover_p, const ros::Time& stamp) {
    quadrotor_msgs::PolyTraj traj_msg;
    traj_msg.hover = true;
    traj_msg.use_yaw_scan = false;
    traj_msg.hover_p.resize(3);
    for (int i = 0; i < 3; ++i) {
      traj_msg.hover_p[i] = hover_p[i];
    }
    traj_msg.start_time = stamp;
    traj_msg.traj_id = traj_id_++;
    traj_pub_.publish(traj_msg);
  }
  // 紧急刹车：生成5阶多项式刹车轨迹，显式规划减速到零
  void emergency_brake(const Eigen::MatrixXd& iniState, const ros::Time& stamp) {
    Eigen::Vector3d p0 = iniState.col(0);
    Eigen::Vector3d v0 = iniState.col(1);
    Eigen::Vector3d a0 = iniState.col(2);
    double v_norm = v0.norm();
    if (v_norm < 0.3) {
      pub_hover_p(p0, stamp);
      return;
    }
    // 刹车时间: v/a_max, 至少0.5s
    double a_max = 5.0;
    double T = std::max(v_norm / a_max, 0.5);
    // 刹车终点(匀减速近似)，沿速度方向检查障碍物安全性
    Eigen::Vector3d pf_ideal = p0 + v0 * T * 0.5;
    Eigen::Vector3d dir = v0.normalized();
    double max_d = (pf_ideal - p0).norm();
    Eigen::Vector3d pf = p0;
    for (double d = 0.1; d <= max_d; d += 0.1) {
      Eigen::Vector3d c = p0 + d * dir;
      if (gridmapPtr_->isOccupied(c)) break;
      pf = c;
    }
    // 解5阶多项式系数: p(t)=c5+c4*t+c3*t^2+c2*t^3+c1*t^4+c0*t^5
    // 边界: p(0)=p0,v(0)=v0,a(0)=a0, p(T)=pf,v(T)=0,a(T)=0
    double T2=T*T, T3=T2*T, T4=T3*T, T5=T4*T;
    Eigen::Matrix3d A;
    A <<   T3,    T4,     T5,
         3*T2,  4*T3,   5*T4,
         6*T,  12*T2,  20*T3;
    Eigen::Matrix3d Ainv = A.inverse();
    CoefficientMat cMat;
    cMat.setZero();
    cMat.col(5) = p0;
    cMat.col(4) = v0;
    cMat.col(3) = a0 / 2.0;
    for (int d = 0; d < 3; d++) {
      Eigen::Vector3d rhs(pf(d) - p0(d) - v0(d)*T - a0(d)/2.0*T2,
                          -v0(d) - a0(d)*T,
                          -a0(d));
      Eigen::Vector3d sol = Ainv * rhs;
      cMat(d, 2) = sol(0);
      cMat(d, 1) = sol(1);
      cMat(d, 0) = sol(2);
    }
    std::vector<double> durs = {T};
    std::vector<CoefficientMat> cMats = {cMat};
    Trajectory traj(durs, cMats);
    double yaw = atan2(v0.y(), v0.x());
    pub_traj(traj, yaw, stamp);
    traj_poly_ = traj;
    replan_stamp_ = stamp;
  }
  void pub_traj(const Trajectory& traj, const double& yaw, const ros::Time& stamp,
                bool use_yaw_scan = false, double yaw_scan_base = 0.0) {
    quadrotor_msgs::PolyTraj traj_msg;
    traj_msg.hover = false;
    traj_msg.order = 5;
    Eigen::VectorXd durs = traj.getDurations();
    int piece_num = traj.getPieceNum();
    traj_msg.duration.resize(piece_num);
    traj_msg.coef_x.resize(6 * piece_num);
    traj_msg.coef_y.resize(6 * piece_num);
    traj_msg.coef_z.resize(6 * piece_num);
    for (int i = 0; i < piece_num; ++i) {
      traj_msg.duration[i] = durs(i);
      CoefficientMat cMat = traj[i].getCoeffMat();
      int i6 = i * 6;
      for (int j = 0; j < 6; j++) {
        traj_msg.coef_x[i6 + j] = cMat(0, j);
        traj_msg.coef_y[i6 + j] = cMat(1, j);
        traj_msg.coef_z[i6 + j] = cMat(2, j);
      }
    }
    traj_msg.start_time = stamp;
    traj_msg.traj_id = traj_id_++;
    // NOTE yaw
    traj_msg.yaw = yaw;
    traj_msg.use_yaw_scan = use_yaw_scan;
    if (use_yaw_scan) {
      const double amp = std::max(0.0, search_yaw_scan_range_deg_) * M_PI / 180.0;
      const double freq = std::max(0.0, search_yaw_scan_freq_hz_);
      const double phase_unwrapped = 2.0 * M_PI * freq * stamp.toSec();
      traj_msg.yaw_scan_base = std::atan2(std::sin(yaw_scan_base), std::cos(yaw_scan_base));
      traj_msg.yaw_scan_amp = amp;
      traj_msg.yaw_scan_freq = freq;
      traj_msg.yaw_scan_phase = std::atan2(std::sin(phase_unwrapped), std::cos(phase_unwrapped));
    } else {
      traj_msg.yaw_scan_base = 0.0;
      traj_msg.yaw_scan_amp = 0.0;
      traj_msg.yaw_scan_freq = 0.0;
      traj_msg.yaw_scan_phase = 0.0;
    }
    traj_msg.drone_id = trajOptPtr_->drone_id_;
    has_published_motion_traj_ = true;
    traj_pub_.publish(traj_msg);
    // Only swarm trackers (drone_id >= 0) participate in broadcast trajectory exchange.
    if (trajOptPtr_->drone_id_ >= 0) {
      broadcast_traj_pub_.publish(traj_msg);
    }
  }

  void RecvBroadcastPolyTrajCallback(const quadrotor_msgs::PolyTrajConstPtr& msg) {
    if (msg->drone_id < 0) {
      ROS_WARN_THROTTLE(1.0, "[drone %d] drop traj: invalid drone_id=%d",
                        trajOptPtr_->drone_id_, msg->drone_id);
      return;
    }
    if (msg->order != 5) {
      ROS_WARN_THROTTLE(1.0, "[drone %d] drop traj from drone %d: order=%d (!=5)",
                        trajOptPtr_->drone_id_, msg->drone_id, msg->order);
      return;
    }
    if (msg->duration.size() * (msg->order + 1) != msg->coef_x.size()) {
      ROS_WARN_THROTTLE(1.0, "[drone %d] drop traj from drone %d: malformed coeff size",
                        trajOptPtr_->drone_id_, msg->drone_id);
      return;
    }
    // 仿真中规划耗时 + 消息延迟可达数百ms，放宽到 2.0s
    double stamp_diff = (ros::Time::now() - msg->start_time).toSec();
    if (stamp_diff > 2.0 || stamp_diff < -2.0) {
      ROS_WARN_THROTTLE(1.0, "[drone %d] drop traj from drone %d: stamp_diff=%.3fs",
                        trajOptPtr_->drone_id_, msg->drone_id, stamp_diff);
      return;
    }

    const size_t recv_id = (size_t)msg->drone_id;
    if ((int)recv_id == trajOptPtr_->drone_id_) return;  // Assume TrajOpt has drone_id_

    SwarmTrajData incoming;
    incoming.drone_id = recv_id;
    incoming.traj_id = msg->traj_id;
    incoming.start_time = msg->start_time.toSec();

    int piece_nums = msg->duration.size();
    std::vector<double> dura(piece_nums);
    std::vector<CoefficientMat> cMats(piece_nums);
    for (int i = 0; i < piece_nums; ++i) {
      int i6 = i * 6;
      cMats[i].row(0) << msg->coef_x[i6 + 0], msg->coef_x[i6 + 1], msg->coef_x[i6 + 2],
          msg->coef_x[i6 + 3], msg->coef_x[i6 + 4], msg->coef_x[i6 + 5];
      cMats[i].row(1) << msg->coef_y[i6 + 0], msg->coef_y[i6 + 1], msg->coef_y[i6 + 2],
          msg->coef_y[i6 + 3], msg->coef_y[i6 + 4], msg->coef_y[i6 + 5];
      cMats[i].row(2) << msg->coef_z[i6 + 0], msg->coef_z[i6 + 1], msg->coef_z[i6 + 2],
          msg->coef_z[i6 + 3], msg->coef_z[i6 + 4], msg->coef_z[i6 + 5];

      dura[i] = msg->duration[i];
    }

    Trajectory trajectory(dura, cMats);
    incoming.traj = trajectory;
    incoming.duration = trajectory.getTotalDuration();
    incoming.start_pos = trajectory.getPos(0.0);

    /* Fill up and store data */
    std::lock_guard<std::mutex> lk(swarm_trajs_mutex_);
    if (swarm_trajs_.size() <= recv_id) {
      for (size_t i = swarm_trajs_.size(); i <= recv_id; i++) {
        SwarmTrajData blank;
        blank.drone_id = -1;
        swarm_trajs_.push_back(blank);
      }
    }
    swarm_trajs_[recv_id] = incoming;
    ROS_INFO_THROTTLE(1.0, "[drone %d] recv traj from drone %zu, traj_id=%d, dur=%.2f",
                      trajOptPtr_->drone_id_, recv_id, incoming.traj_id, incoming.duration);

    /* Check Collision */
    // Add collision check if needed
  }

  void triger_callback(const geometry_msgs::PoseStampedConstPtr& msgPtr) {
  // 将triger话题中的x,y位置作为goal位置
    goal_ << msgPtr->pose.position.x, msgPtr->pose.position.y, 3 ;
    triger_received_ = true;
  }

  void land_triger_callback(const geometry_msgs::PoseStampedConstPtr& msgPtr) {
    land_p_.x() = msgPtr->pose.position.x;
    land_p_.y() = msgPtr->pose.position.y;
    land_p_.z() = msgPtr->pose.position.z;
    land_q_.w() = msgPtr->pose.orientation.w;
    land_q_.x() = msgPtr->pose.orientation.x;
    land_q_.y() = msgPtr->pose.orientation.y;
    land_q_.z() = msgPtr->pose.orientation.z;
    land_triger_received_ = true;
  }

  void odom_callback(const nav_msgs::Odometry::ConstPtr& msgPtr) {
    while (odom_lock_.test_and_set())
      ;
    odom_msg_ = *msgPtr;
    odom_received_ = true;
    odom_lock_.clear();
  }

  void target_callback(const nav_msgs::Odometry::ConstPtr& msgPtr) {
    while (target_lock_.test_and_set())
      ;
    target_msg_ = *msgPtr;
    target_received_ = true;
    target_lock_.clear();
  }

  void gridmap_callback(const quadrotor_msgs::OccMap3dConstPtr& msgPtr) {
    while (gridmap_lock_.test_and_set())
      ;
    map_msg_ = *msgPtr;
    map_received_ = true;
    gridmap_lock_.clear();
  }

  void search_state_callback(const std_msgs::Bool::ConstPtr& msgPtr) {
    while (search_state_lock_.test_and_set())
      ;
    const bool prev_search_mode = search_mode_active_;
    latest_search_state_ = *msgPtr;
    search_mode_active_ = msgPtr->data;
    if (search_mode_active_) {
      post_reacquire_boost_active_ = false;
      post_reacquire_boost_end_time_ = ros::Time(0);
    } else if (prev_search_mode && post_reacquire_boost_sec_ > 0.0) {
      post_reacquire_boost_active_ = true;
      post_reacquire_boost_end_time_ = ros::Time::now() + ros::Duration(post_reacquire_boost_sec_);
      ROS_WARN("[drone %d planner] EXIT SEARCH -> BOOSTED TRACKING %.2fs: suppress formation cost",
               trajOptPtr_->drone_id_, post_reacquire_boost_sec_);
    }
    search_state_lock_.clear();
  }

  void search_targets_callback(const geometry_msgs::PoseArray::ConstPtr& msgPtr) {
    while (search_targets_lock_.test_and_set())
      ;
    search_targets_ = *msgPtr;
    search_targets_lock_.clear();
  }

  // 搜索标签信息回调：接收掌管者发布的粒子群均值和分配信息
  void search_label_info_callback(const target_ekf::SearchLabelInfo::ConstPtr& msgPtr) {
    while (search_label_info_lock_.test_and_set())
      ;
    search_label_infos_[msgPtr->label] = *msgPtr;
    search_label_info_lock_.clear();

    // 检查本机是否在该标签的分配列表中
    int my_drone_id = trajOptPtr_->drone_id_;
    const auto& assigned_ids = msgPtr->assigned_drone_ids;
    auto it = std::find(assigned_ids.begin(), assigned_ids.end(), my_drone_id);
    if (it == assigned_ids.end()) {
      return;  // 本机不在该标签内
    }

    // 计算本机在标签内的索引和偏置
    int my_index = std::distance(assigned_ids.begin(), it);
    int num_drones_in_label = assigned_ids.size();

    Eigen::Vector3d mean_pos(msgPtr->mean_pos.x, msgPtr->mean_pos.y, msgPtr->mean_pos.z);
    Eigen::Vector3d mean_vel(msgPtr->mean_vel.x, msgPtr->mean_vel.y, msgPtr->mean_vel.z);

    // 计算速度方向（用于偏置和偏航角）
    Eigen::Vector2d vel_dir(mean_vel.x(), mean_vel.y());
    double vel_norm = vel_dir.norm();
    if (vel_norm < 0.1) {
      vel_dir = Eigen::Vector2d(1.0, 0.0);  // 默认朝x正方向
    } else {
      vel_dir.normalize();
    }

    // 垂直于速度方向的单位向量（用于横向偏置）
    Eigen::Vector2d perp_dir(-vel_dir.y(), vel_dir.x());

    // 计算偏置：多无人机时沿垂直方向展开，增加搜索覆盖面
    double offset_spacing = 3.0;  // 无人机间距（米）
    double lateral_offset = 0.0;
    if (num_drones_in_label > 1) {
      // 居中分布：-1, 0, 1 或 -1.5, -0.5, 0.5, 1.5 等
      lateral_offset = (my_index - (num_drones_in_label - 1) / 2.0) * offset_spacing;
    }

    // 应用偏置
    search_target_with_offset_ = mean_pos;
    search_target_with_offset_.x() += lateral_offset * perp_dir.x();
    search_target_with_offset_.y() += lateral_offset * perp_dir.y();

    // 计算期望偏航角：朝向粒子群均值方向，但各无人机略微分散
    double base_yaw = std::atan2(vel_dir.y(), vel_dir.x());
    double yaw_spread = M_PI / 6.0;  // 30度扇形展开
    double yaw_offset = 0.0;
    if (num_drones_in_label > 1) {
      yaw_offset = (my_index - (num_drones_in_label - 1) / 2.0) * yaw_spread / (num_drones_in_label - 1);
    }
    search_desired_yaw_ = base_yaw + yaw_offset;

    ROS_DEBUG_THROTTLE(1.0, "[planner drone%d] Search label %d: offset=(%.2f,%.2f), yaw=%.2f",
                       my_drone_id, msgPtr->label, lateral_offset * perp_dir.x(),
                       lateral_offset * perp_dir.y(), search_desired_yaw_ * 180.0 / M_PI);
  }

  // NOTE main callback
  void plan_timer_callback(const ros::TimerEvent& event) {
    heartbeat_pub_.publish(std_msgs::Empty());
    ROS_INFO_THROTTLE(5.0, "[drone %d] plan_timer alive", trajOptPtr_->drone_id_);  // 确认回调在执行
    if (!odom_received_ || !map_received_) {
      return;
    }
    // obtain state of odom
    ROS_DEBUG("[drone %d] acquiring odom_lock", trajOptPtr_->drone_id_);
    while (odom_lock_.test_and_set());
    auto odom_msg = odom_msg_;
    odom_lock_.clear();//odom_lock_用于保证读取odom_msg_时的线程安全
    ROS_DEBUG("[drone %d] odom_lock released", trajOptPtr_->drone_id_);
    Eigen::Vector3d odom_p(odom_msg.pose.pose.position.x,
                           odom_msg.pose.pose.position.y,
                           odom_msg.pose.pose.position.z);
    Eigen::Vector3d odom_v(odom_msg.twist.twist.linear.x,
                           odom_msg.twist.twist.linear.y,
                           odom_msg.twist.twist.linear.z);
    Eigen::Quaterniond odom_q(odom_msg.pose.pose.orientation.w,
                              odom_msg.pose.pose.orientation.x,
                              odom_msg.pose.pose.orientation.y,
                              odom_msg.pose.pose.orientation.z);
    if (!triger_received_) {//追踪者接收到sh文件发布的triger话题，启动规划，但triger话题中的位置不决定目标位置，目标位置由target话题中的里程计消息决定
      return;
    }
    if (!target_received_) {
      return;
    }
    if (post_reacquire_boost_active_ && ros::Time::now() >= post_reacquire_boost_end_time_) {
      post_reacquire_boost_active_ = false;
      ROS_WARN("[drone %d planner] BOOSTED TRACKING ended: restore formation cost", trajOptPtr_->drone_id_);
    }
    trajOptPtr_->suppress_formation_cost_ = post_reacquire_boost_active_;
    if (post_reacquire_boost_active_) {
      const double remain = std::max(0.0, (post_reacquire_boost_end_time_ - ros::Time::now()).toSec());
      ROS_INFO_THROTTLE(1.0,
                        "[drone %d planner] BOOSTED TRACKING active: suppress formation cost, remaining=%.2fs",
                        trajOptPtr_->drone_id_, remain);
    }

    // NOTE sequential start：drone_id=0 直接规划；drone_id>=1 等收到前一架无人机的广播轨迹后才开始规划
    // 与 Swarm-Formation 的 SEQUENTIAL_START 状态逻辑一致，保证第一次规划时 swarm_trajs_ 非空
    if (trajOptPtr_->use_formation_ && trajOptPtr_->drone_id_ >= 1) {
      int prev_id = trajOptPtr_->drone_id_ - 1;
      bool have_prev = false;
      int slot_drone_id = -999;
      int slot_traj_id = -999;
      size_t buf_size = 0;
      {
        std::lock_guard<std::mutex> lk(swarm_trajs_mutex_);
        buf_size = swarm_trajs_.size();
        if ((int)swarm_trajs_.size() > prev_id) {
          slot_drone_id = swarm_trajs_[prev_id].drone_id;
          slot_traj_id = swarm_trajs_[prev_id].traj_id;
        }
        have_prev = ((int)swarm_trajs_.size() > prev_id &&
                     swarm_trajs_[prev_id].drone_id == prev_id);
      }
      if (!have_prev) {
        ROS_WARN_THROTTLE(1.0,
                          "[drone %d] waiting for drone %d trajectory... (buf=%zu, slot_drone_id=%d, slot_traj_id=%d)",
                          trajOptPtr_->drone_id_, prev_id, buf_size, slot_drone_id, slot_traj_id);
        return;
      }
    }

    // NOTE obtain state of target
    while (target_lock_.test_and_set())//追踪者节点需要订阅对目标状态的推断！然后填入target信息用于规划
      ;
    replanStateMsg_.target = target_msg_;
    target_lock_.clear();
    Eigen::Vector3d target_p(replanStateMsg_.target.pose.pose.position.x,
                             replanStateMsg_.target.pose.pose.position.y,
                             replanStateMsg_.target.pose.pose.position.z);
    Eigen::Vector3d target_v(replanStateMsg_.target.twist.twist.linear.x,
                             replanStateMsg_.target.twist.twist.linear.y,
                             replanStateMsg_.target.twist.twist.linear.z);
    Eigen::Quaterniond target_q;
    target_q.w() = replanStateMsg_.target.pose.pose.orientation.w;
    target_q.x() = replanStateMsg_.target.pose.pose.orientation.x;
    target_q.y() = replanStateMsg_.target.pose.pose.orientation.y;
    target_q.z() = replanStateMsg_.target.pose.pose.orientation.z;
    Eigen::Vector3d raw_target_p = target_p;  // 保存原始目标位置（编队偏移前）

    // NOTE detect ekf reset
    if (last_target_p_valid_) {
      if ((target_p - last_target_p_).norm() > 2.0) {
        ROS_WARN("EKF reset detected, skipping this planning cycle.");
        last_target_p_ = target_p;
        return;
      }
    }
    last_target_p_ = target_p;
    last_target_p_valid_ = true;

    // NOTE force-hover: waiting for the speed of drone small enough
    if (force_hover_ && odom_v.norm() > 0.1) {
      ROS_WARN_THROTTLE(0.5,
                        "[drone %d] force_hover gate: waiting for speed to drop (|v|=%.2f), skip planning this cycle",
                        trajOptPtr_->drone_id_, odom_v.norm());
      return;
    }

    // NOTE just for landing on the car!
    double search_scan_base_yaw = std::atan2(std::sin(search_desired_yaw_), std::cos(search_desired_yaw_));
    bool use_search_yaw_scan = false;
    if (land_triger_received_) {//车上着陆逻辑
      if (std::fabs((target_p - odom_p).norm() < 0.1 && odom_v.norm() < 0.1 && target_v.norm() < 0.2)) {
        if (!wait_hover_) {
          pub_hover_p(odom_p, ros::Time::now());
          wait_hover_ = true;
        }
        ROS_WARN("[drone %d] HOVERING...", trajOptPtr_->drone_id_);
        return;
      }
      // TODO get the orientation fo target and calculate the pose of landing point
      target_p = target_p + target_q * land_p_;
      wait_hover_ = false;
    } else {//追踪逻辑
      if (search_mode_active_) {
        // 搜索模式：target_p 已经是 dpf 节点发布的热点位置，偏航在朝向热点基础上连续摇头扫描
        Eigen::Vector3d dir = target_p - odom_p;
        dir.z() = 0.0;
        double base_yaw = search_desired_yaw_;
        if (dir.head<2>().norm() > 1e-2) {
          base_yaw = std::atan2(dir.y(), dir.x());
        }
        search_scan_base_yaw = std::atan2(std::sin(base_yaw), std::cos(base_yaw));
        use_search_yaw_scan = true;

        // 连续正弦扫角，尽可能扩大搜索期视场覆盖，同时避免离散跳变导致抖动
        const double scan_angle_range = std::max(0.0, search_yaw_scan_range_deg_) * M_PI / 180.0;
        const double scan_freq_hz = std::max(0.0, search_yaw_scan_freq_hz_);
        const double t = ros::Time::now().toSec();
        const double yaw_offset = scan_angle_range * std::sin(2.0 * M_PI * scan_freq_hz * t);
        search_desired_yaw_ = search_scan_base_yaw + yaw_offset;
        search_desired_yaw_ = std::atan2(std::sin(search_desired_yaw_), std::cos(search_desired_yaw_));
        target_p.z() = std::max(2.0, odom_p.z());  // 保持安全高度

        ROS_INFO_THROTTLE(1.0, "[planner drone%d] SEARCH MODE: target=(%.2f,%.2f,%.2f)",
                          trajOptPtr_->drone_id_, target_p.x(), target_p.y(), target_p.z());
      } else {
        target_p.z() += 0.3;// 追踪目标定在目标上方1m处

        // 仿照 Swarm-Formation：每架无人机终点 = 目标位置 + 编队偏移
        // 先将模板偏移放缩到 tracking_dist_，再按目标速度方向旋转到世界系
        if (trajOptPtr_->use_formation_) {
          Eigen::Vector3d offset_local = trajOptPtr_->formation_offset_;
          double r = offset_local.head<2>().norm();
          if (r > 1e-3) {
            offset_local *= (tracking_dist_ / r);

            // 编队朝向必须由目标状态统一决定（不能依赖本机 odom），
            // 否则不同无人机会得到不同朝向导致队形扭曲/重叠。
            Eigen::Vector2d target_v_xy = target_v.head<2>();
            double formation_heading = last_formation_heading_;
            bool have_target_yaw = false;
            double target_yaw = 0.0;
            if (target_q.norm() > 1e-6) {
              Eigen::Quaterniond qn = target_q.normalized();
              target_yaw = std::atan2(2.0 * (qn.w() * qn.z() + qn.x() * qn.y()),
                                      1.0 - 2.0 * (qn.y() * qn.y() + qn.z() * qn.z()));
              have_target_yaw = std::isfinite(target_yaw);
            }

            if (target_v_xy.norm() >= formation_heading_speed_thresh_) {
              formation_heading = std::atan2(target_v_xy.y(), target_v_xy.x());
            } else if (have_target_yaw) {
              formation_heading = target_yaw;
            } else if (!last_formation_heading_valid_) {
              formation_heading = 0.0;
            }
            formation_heading = std::atan2(std::sin(formation_heading), std::cos(formation_heading));
            last_formation_heading_ = formation_heading;
            last_formation_heading_valid_ = true;

            const double c = std::cos(formation_heading);
            const double s = std::sin(formation_heading);
            Eigen::Vector3d offset_world;
            offset_world.x() = c * offset_local.x() - s * offset_local.y();
            offset_world.y() = s * offset_local.x() + c * offset_local.y();
            offset_world.z() = offset_local.z();
            target_p += offset_world;
          }
        }
      }

      // NOTE determin whether to replan
      
      // std::cout << "dist : " << dp.norm() << std::endl;
      double desired_yaw;
      if (search_mode_active_) {
        // 搜索模式：使用分配的期望偏航角（增加搜索覆盖面）
        desired_yaw = search_desired_yaw_;
      } else {
        // 追踪模式：朝向原始目标方向
        Eigen::Vector3d dp_yaw = raw_target_p - odom_p;
        desired_yaw = std::atan2(dp_yaw.y(), dp_yaw.x());
      }
      Eigen::Vector3d project_yaw = odom_q.toRotationMatrix().col(0);  // NOTE ZYX
      double now_yaw = std::atan2(project_yaw.y(), project_yaw.x());
      double yaw_err = std::atan2(std::sin(desired_yaw - now_yaw), std::cos(desired_yaw - now_yaw));
      const double dist_to_target = (target_p - odom_p).norm();
      const bool reach_tracking_goal = trajOptPtr_->use_formation_
                                           ? (dist_to_target < tolerance_d_)
                                           : (std::fabs(dist_to_target - tracking_dist_) < tolerance_d_);
      if (has_published_motion_traj_ &&
          reach_tracking_goal &&
          odom_v.norm() < 0.1 && target_v.norm() < 0.2 &&
          std::fabs(yaw_err) < 0.5) {// 如果接近目标、速度够小、朝向正确，就保持悬停
        if (!wait_hover_) {
          pub_hover_p(odom_p, ros::Time::now());
          wait_hover_ = true;
        }
        ROS_WARN("[drone %d] HOVERING...", trajOptPtr_->drone_id_);
        replanStateMsg_.state = -1;
        replanState_pub_.publish(replanStateMsg_);
        return;
      } else {
        wait_hover_ = false;
      }
    }

    // NOTE obtain map
    ROS_DEBUG("[drone %d] acquiring gridmap_lock", trajOptPtr_->drone_id_);
    while (gridmap_lock_.test_and_set())
      ;
    gridmapPtr_->from_msg(map_msg_);
    if (trajOptPtr_->use_soft_constraint_) {
      gridmapPtr_->updateESDF();  // 软约束模式需要 ESDF
    }
    replanStateMsg_.occmap = map_msg_;
    gridmap_lock_.clear();
    ROS_DEBUG("[drone %d] gridmap_lock released", trajOptPtr_->drone_id_);


    // Check map freshness - allow up to 200ms delay for lidar mapping
    double map_age = (ros::Time::now() - map_msg_.header.stamp).toSec();
    if (map_age > 0.2) {
      ROS_WARN("[drone %d planner] Map is stale: %.3f seconds, set force_hover (|odom_v|=%.2f)",
               trajOptPtr_->drone_id_, map_age, odom_v.norm());
      force_hover_ = true;
      replanStateMsg_.state = 4;
      replanState_pub_.publish(replanStateMsg_);
      return;
    }

    prePtr_->setMap(*gridmapPtr_);

    // Set swarm trajectories for collision avoidance
    {
      std::lock_guard<std::mutex> lk(swarm_trajs_mutex_);
      trajOptPtr_->setSwarmTrajs(swarm_trajs_);
    }

    // visualize the ray from drone to target
    if (envPtr_->checkRayValid(odom_p, target_p)) {
      visPtr_->visualize_arrow(odom_p, target_p, "ray", visualization::yellow);// 无遮挡标记黄色
    } else {
      visPtr_->visualize_arrow(odom_p, target_p, "ray", visualization::red);// 有遮挡标记红色
    }

    // 20Hz 主循环中，默认只做合法性检查；
    // 仅在“当前轨迹失效”或“到达硬重规划节拍”时才执行真正重规划。
    const ros::Time now = ros::Time::now();
    const bool has_traj_for_check = has_published_motion_traj_ && !force_hover_;
    bool need_replan_due_invalid = false;
    if (has_traj_for_check && !validcheck(traj_poly_, replan_stamp_)) {
      need_replan_due_invalid = true;
      ROS_WARN_THROTTLE(0.2,
                        "[drone %d] current traj invalid, trigger immediate replan (reason=%s, t=%.2f, p=[%.2f %.2f %.2f])",
                        trajOptPtr_->drone_id_, last_validcheck_fail_reason_.c_str(),
                        last_validcheck_fail_t_, last_validcheck_fail_p_.x(),
                        last_validcheck_fail_p_.y(), last_validcheck_fail_p_.z());
    }

    const bool need_periodic_hard_replan =
        (!has_published_motion_traj_) ||
        ((now - last_hard_replan_stamp_).toSec() >= hard_replan_interval_);

    if (!need_replan_due_invalid && !need_periodic_hard_replan) {
      ROS_DEBUG_THROTTLE(1.0, "[drone %d] check-only cycle (skip heavy replan)",
                         trajOptPtr_->drone_id_);
      return;
    }

    if (!need_replan_due_invalid) {
      // 周期性硬重规划按固定频率节拍触发，避免每个20Hz周期都做重规划。
      last_hard_replan_stamp_ = now;
    }

    // NOTE prediction 追踪者需要对目标未来状态进行预测
    std::vector<Eigen::Vector3d> target_predcit;
    // ros::Time t_start = ros::Time::now();
    bool generate_new_traj_success = prePtr_->predict(target_p, target_v, target_predcit);// 预测采用简单的匀速模型A*拓展（可以考虑在其中添加丢失观测只靠预测时的不确定度）
    // ros::Time t_stop = ros::Time::now();
    // std::cout << "predict costs: " << (t_stop - t_start).toSec() * 1e3 << "ms" << std::endl;
    if (generate_new_traj_success && target_predcit.empty()) {
      ROS_WARN("[drone %d] prediction returned empty path, skip this cycle", trajOptPtr_->drone_id_);
      generate_new_traj_success = false;
    }
    if (generate_new_traj_success) {
      Eigen::Vector3d observable_p = target_predcit.back();
      visPtr_->visualize_path(target_predcit, "car_predict");
      std::vector<Eigen::Vector3d> observable_margin;
      for (double theta = 0; theta <= 2 * M_PI; theta += 0.01) {
        observable_margin.emplace_back(observable_p + tracking_dist_ * Eigen::Vector3d(cos(theta), sin(theta), 0));
      }
      visPtr_->visualize_path(observable_margin, "observable_margin");//observable_margin是以预测轨迹的最后一个点为圆心，追踪距离为半径的圆，用于可视化追踪者的期望位置范围
    }

    // NOTE replan state
    Eigen::MatrixXd iniState;
    iniState.setZero(3, 3);
    ros::Time replan_stamp = ros::Time::now() + ros::Duration(0.03);
    double replan_t = (replan_stamp - replan_stamp_).toSec();//上次规划到现在的时间
    if (force_hover_ || replan_t > traj_poly_.getTotalDuration()) {//上次规划的轨迹已经执行完了，就从当前状态重新规划
      // should replan from the hover state
      iniState.col(0) = odom_p;
      iniState.col(1) = odom_v;
    } else {//上次规划的轨迹还没执行完就重规划，就从上次规划的轨迹状态重新规划，保证新轨迹和上次规划的轨迹衔接平滑
      // should replan from the last trajectory
      Eigen::Vector3d traj_pos = traj_poly_.getPos(replan_t);
      if ((traj_pos - odom_p).norm() > 0.5) {
        ROS_WARN("[drone %d] traj-odom drift %.2fm, replan from odom", trajOptPtr_->drone_id_, (traj_pos - odom_p).norm());
        iniState.col(0) = odom_p;
        iniState.col(1) = odom_v;
      } else {
        iniState.col(0) = traj_pos;
        iniState.col(1) = traj_poly_.getVel(replan_t);
        iniState.col(2) = traj_poly_.getAcc(replan_t);
      }
    }
    replanStateMsg_.header.stamp = ros::Time::now();
    replanStateMsg_.iniState.resize(9);
    Eigen::Map<Eigen::MatrixXd>(replanStateMsg_.iniState.data(), 3, 3) = iniState;

    // NOTE path searching
    Eigen::Vector3d p_start = iniState.col(0);
    std::vector<Eigen::Vector3d> path, way_pts;

    // NOTE calculate time of path searching, corridor generation and optimization

    // static double t_path_ = 0;
    // static double t_corridor_ = 0;
    // static double t_optimization_ = 0;
    // static int times_path_ = 0;
    // static int times_corridor_ = 0;
    // static int times_optimization_ = 0;
    // double t_path = 0;

    if (generate_new_traj_success) {
      // 前端：short_astar 到当前偏置目标点，再与预测路径拼接。
      ROS_DEBUG("[drone %d] starting path search", trajOptPtr_->drone_id_);
      Eigen::Vector3d actual_target = target_p;
      generate_new_traj_success = envPtr_->short_astar(p_start, target_p, path, &actual_target);;
      ROS_DEBUG("[drone %d] path search done: %d", trajOptPtr_->drone_id_, generate_new_traj_success);

      if (generate_new_traj_success && path.empty()) {
        // 防御式兜底：short_astar 成功但返回空路径时，至少保留终点，避免后续 back() 崩溃
        path.push_back(actual_target);
        ROS_WARN("[drone %d] short_astar returned empty path, fallback to single-point path",
                 trajOptPtr_->drone_id_);
      }

      // 如果 short_astar 终点被调整，预测轨迹同步平移，保证拼接连续。
      if (generate_new_traj_success && (actual_target - target_p).norm() > 0.01) {
        Eigen::Vector3d offset = actual_target - target_p;
        for (auto& p : target_predcit) {
          p += offset;
        }
        ROS_DEBUG("[drone %d] target adjusted, offset prediction by %.2fm", trajOptPtr_->drone_id_, offset.norm());
      }
      // 原可见路径逻辑（已注释）:
      // if (land_triger_received_) {
      //   generate_new_traj_success = envPtr_->short_astar(p_start, target_p, path);
      // } else {
      //   generate_new_traj_success = envPtr_->findVisiblePath(p_start, target_predcit, way_pts, path);
      // }
      // ros::Time t_end0 = ros::Time::now();
      // t_path += (t_end0 - t_front0).toSec() * 1e3;
    }

    std::vector<Eigen::Vector3d> visible_ps;
    std::vector<double> thetas;
    Trajectory traj;
    if (generate_new_traj_success) {
      visPtr_->visualize_path(path, "astar");
      // 拼接预测路径：astar_path + predict_path（去掉重复连接点）。
      std::vector<Eigen::Vector3d> astar_path = path;
      std::vector<Eigen::Vector3d> predict_waypts;
      predict_waypts.push_back(astar_path.back());
      for (const auto& p : target_predcit) {
        predict_waypts.push_back(p);
      }
      std::vector<Eigen::Vector3d> predict_path;
      envPtr_->pts2path(predict_waypts, predict_path);
      path = astar_path;
      if (!predict_path.empty()) {
        for (size_t i = 1; i < predict_path.size(); ++i) {
          path.push_back(predict_path[i]);
        }
      }
      const size_t raw_path_size = path.size();
      if (clipPathToObservationRange(path, odom_p)) {
        ROS_INFO_THROTTLE(1.0,
                          "[drone %d] clip path to obs range: %zu -> %zu (range=%.2f, margin=%.2f)",
                          trajOptPtr_->drone_id_, raw_path_size, path.size(), obs_clip_range_, obs_clip_margin_);
      }
      // 原可见区域逻辑（已注释）:
      // if (land_triger_received_) {
      //   for (const auto& p : target_predcit) {
      //     path.push_back(p);
      //   }
      // } else {
      //   target_predcit.pop_back();
      //   way_pts.pop_back();
      //   envPtr_->generate_visible_regions(target_predcit, way_pts, visible_ps, thetas);
      //   visPtr_->visualize_pointcloud(visible_ps, "visible_ps");
      //   visPtr_->visualize_fan_shape_meshes(target_predcit, visible_ps, thetas, "visible_region");
      //   std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> rays;
      //   for (int i = 0; i < (int)way_pts.size(); ++i) {
      //     rays.emplace_back(target_predcit[i], way_pts[i]);
      //   }
      //   visPtr_->visualize_pointcloud(way_pts, "way_pts");
      //   way_pts.insert(way_pts.begin(), p_start);
      //   envPtr_->pts2path(way_pts, path);
      // }
      // NOTE corridor generating (only needed for hard constraint mode)
      std::vector<Eigen::MatrixXd> hPolys;
      std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> keyPts;

      if (!trajOptPtr_->use_soft_constraint_) {
        ROS_DEBUG("[drone %d] starting generateSFC", trajOptPtr_->drone_id_);
        envPtr_->generateSFC(path, 2.0, hPolys, keyPts);
        ROS_DEBUG("[drone %d] generateSFC done", trajOptPtr_->drone_id_);
        envPtr_->visCorridor(hPolys);
        visPtr_->visualize_pairline(keyPts, "keyPts");
      }

      // NOTE trajectory optimization
      Eigen::MatrixXd finState;
      finState.setZero(3, 3);
      finState.col(0) = path.back();
      finState.col(1) = target_v;
      ROS_DEBUG("[drone %d] starting traj optimization", trajOptPtr_->drone_id_);
      if (trajOptPtr_->use_soft_constraint_) {
        generate_new_traj_success = trajOptPtr_->generate_traj(iniState, finState, target_predcit, hPolys, path, traj);
      } else {
        generate_new_traj_success = trajOptPtr_->generate_traj_hard(iniState, finState, target_predcit, hPolys, traj);
      }
      ROS_DEBUG("[drone %d] traj optimization done: %d", trajOptPtr_->drone_id_, generate_new_traj_success);

      visPtr_->visualize_traj(traj, "traj");
    }

    // NOTE collision check
    bool valid = false;
    // 从消息更新网格地图
    while (gridmap_lock_.test_and_set());
    gridmapPtr_->from_msg(map_msg_);
    gridmap_lock_.clear();
    if (generate_new_traj_success) {
      valid = validcheck(traj, replan_stamp);
    } else {
      replanStateMsg_.state = -2;
      replanState_pub_.publish(replanStateMsg_);
    }
    if (valid) {
      force_hover_ = false;
      trajOptPtr_->emergency_recovery_ = false;
      execute_last_traj_streak_ = 0;
      execute_last_traj_first_stamp_ = ros::Time(0);
      ROS_WARN("[drone %d planner] REPLAN SUCCESS", trajOptPtr_->drone_id_);
      replanStateMsg_.state = 0;
      replanState_pub_.publish(replanStateMsg_);
      Eigen::Vector3d dp = raw_target_p + target_v * 0.03 - iniState.col(0);
      double yaw = 0.0;
      if (land_triger_received_) {
        yaw = 2 * std::atan2(target_q.z(), target_q.w());
      } else if (search_mode_active_) {
        yaw = search_desired_yaw_;
      } else {
        // NOTE : if the drone is going to unknown areas, watch that direction
        // Eigen::Vector3d un_known_p = traj.getPos(1.0);
        // if (gridmapPtr_->isUnKnown(un_known_p)) {
        //   dp = un_known_p - odom_p;
        // }
        yaw = std::atan2(dp.y(), dp.x());
      }
      pub_traj(traj, yaw, replan_stamp, use_search_yaw_scan, search_scan_base_yaw);
      traj_poly_ = traj;
      replan_stamp_ = replan_stamp;
      last_hard_replan_stamp_ = ros::Time::now();
    } else if (force_hover_) {
      ROS_ERROR("[drone %d planner] REPLAN FAILED, HOVERING...", trajOptPtr_->drone_id_);
      replanStateMsg_.state = 1;
      replanState_pub_.publish(replanStateMsg_);
      return;
    } else if (!validcheck(traj_poly_, replan_stamp_)) {
      force_hover_ = true;
      ROS_FATAL("[drone %d planner] EMERGENCY STOP!!! old traj invalid (reason=%s, t=%.2f, p=[%.2f %.2f %.2f], execute_last_streak=%d)",
                trajOptPtr_->drone_id_, last_validcheck_fail_reason_.c_str(),
                last_validcheck_fail_t_, last_validcheck_fail_p_.x(),
                last_validcheck_fail_p_.y(), last_validcheck_fail_p_.z(),
                execute_last_traj_streak_);
      replanStateMsg_.state = 2;
      replanState_pub_.publish(replanStateMsg_);
      trajOptPtr_->emergency_recovery_ = true;
      emergency_brake(iniState, replan_stamp);
      return;
    } else {
      // Update map with latest data before executing old trajectory

      // Verify old trajectory with latest map
      if (!validcheck(traj_poly_, replan_stamp_)) {
        force_hover_ = true;
        ROS_FATAL("[drone %d planner] EMERGENCY STOP - OLD TRAJ INVALID WITH LATEST MAP!!! (reason=%s, t=%.2f, p=[%.2f %.2f %.2f], execute_last_streak=%d)",
                  trajOptPtr_->drone_id_, last_validcheck_fail_reason_.c_str(),
                  last_validcheck_fail_t_, last_validcheck_fail_p_.x(),
                  last_validcheck_fail_p_.y(), last_validcheck_fail_p_.z(),
                  execute_last_traj_streak_);
        replanStateMsg_.state = 2;
        replanState_pub_.publish(replanStateMsg_);
        trajOptPtr_->emergency_recovery_ = true;
        emergency_brake(iniState, replan_stamp);
        return;
      }

      if (execute_last_traj_streak_ == 0) {
        execute_last_traj_first_stamp_ = ros::Time::now();
      }
      execute_last_traj_streak_++;
      double streak_dur = (ros::Time::now() - execute_last_traj_first_stamp_).toSec();
      double old_traj_t = (ros::Time::now() - replan_stamp_).toSec();
      old_traj_t = old_traj_t > 0.0 ? old_traj_t : 0.0;
      double old_traj_rest = std::max(0.0, traj_poly_.getTotalDuration() - old_traj_t);

      ROS_ERROR("[drone %d planner] REPLAN FAILED, EXECUTE LAST TRAJ...", trajOptPtr_->drone_id_);
      ROS_ERROR_THROTTLE(0.2,
                         "[drone %d planner] execute-last stats: streak=%d, streak_dur=%.2fs, old_traj_rest=%.2fs, map_age=%.3fs",
                         trajOptPtr_->drone_id_, execute_last_traj_streak_, streak_dur,
                         old_traj_rest, map_age);
      replanStateMsg_.state = 3;
      replanState_pub_.publish(replanStateMsg_);
      trajOptPtr_->emergency_recovery_ = true;
      return;  // current generated traj invalid but last is valid
    }
    visPtr_->visualize_traj(traj, "traj");
  }

  void fake_timer_callback(const ros::TimerEvent& event) {
    heartbeat_pub_.publish(std_msgs::Empty());
    if (!odom_received_ || !map_received_) {
      return;
    }
    // obtain state of odom
    while (odom_lock_.test_and_set())
      ;
    auto odom_msg = odom_msg_;
    odom_lock_.clear();//odom_lock_用于保证读取odom_msg_时的线程安全
    Eigen::Vector3d odom_p(odom_msg.pose.pose.position.x,
                           odom_msg.pose.pose.position.y,
                           odom_msg.pose.pose.position.z);
    Eigen::Vector3d odom_v(odom_msg.twist.twist.linear.x,
                           odom_msg.twist.twist.linear.y,
                           odom_msg.twist.twist.linear.z);
    if (!triger_received_) {//目标需要收到RVIZ NavigateTo话题才会启动规划
      return;
    }
    // NOTE force-hover: waiting for the speed of drone small enough
    if (force_hover_ && odom_v.norm() > 0.1) {
      return;
    }

    // NOTE local goal
    Eigen::Vector3d local_goal;
    Eigen::Vector3d delta = goal_ - odom_p;
    if (delta.norm() < 15) {//规划半径15m
      local_goal = goal_;
    } else {
      local_goal = delta.normalized() * 15 + odom_p;
    }

    // NOTE obtain map
    while (gridmap_lock_.test_and_set())
      ;
    gridmapPtr_->from_msg(map_msg_);
    replanStateMsg_.occmap = map_msg_;
    gridmap_lock_.clear();//gridmap_lock_用于保证读取map_msg_时的线程安全

    // NOTE determin whether to replan
    bool no_need_replan = false;//一般情况下局部目标点不断更新，每有所更新就触发重规划
    if (!force_hover_ && !wait_hover_) {
      double last_traj_t_rest = traj_poly_.getTotalDuration() - (ros::Time::now() - replan_stamp_).toSec();//上次规划的轨迹剩余时间
      bool new_goal = (local_goal - traj_poly_.getPos(traj_poly_.getTotalDuration())).norm() > tracking_dist_;//因为goal_是一直在更新的，local_goal也在一直更新。目前的轨迹是上一次基于上一次replan时的local_goal的，如果当前轨迹的终点和local_goal距离大于tracking_dist_，则认为有更新的目标
      if (!new_goal) {
        if (last_traj_t_rest < 1.0) {
          ROS_WARN("[planner] NEAR GOAL...");
          no_need_replan = true;//没有更新的目标，当前轨迹的终点就是最终目标点。并且当前轨迹快要结束了，不需要重新规划
        } else if (validcheck(traj_poly_, replan_stamp_, last_traj_t_rest)) {
          ROS_WARN("[planner] NO NEED REPLAN...");
          double t_delta = traj_poly_.getTotalDuration() < 1.0 ? traj_poly_.getTotalDuration() : 1.0;
          double t_yaw = (ros::Time::now() - replan_stamp_).toSec() + t_delta;
          Eigen::Vector3d un_known_p = traj_poly_.getPos(t_yaw);
          Eigen::Vector3d dp = un_known_p - odom_p;
          double yaw = search_mode_active_ ? search_desired_yaw_ : std::atan2(dp.y(), dp.x());
          pub_traj(traj_poly_, yaw, replan_stamp_);
          no_need_replan = true;//没有更新的目标，当前轨迹的终点就是最终目标点。并且当前轨迹在剩余时间内都是安全的，那么不重新规划，并且微调轨迹让无人机偏航角正对行进方向
        }
      }
    }
    // NOTE determin whether to pub hover
    if (has_published_motion_traj_ &&
        (goal_ - odom_p).norm() < tracking_dist_ + tolerance_d_ && odom_v.norm() < 0.1) {//已经接近目标点了，并且速度很小，认为到达目标点，可以悬停了
      if (!wait_hover_) {
        pub_hover_p(odom_p, ros::Time::now());
        wait_hover_ = true;
      }
      ROS_WARN("[planner] HOVERING...");
      replanStateMsg_.state = -1;
      replanState_pub_.publish(replanStateMsg_);
      return;
    } else {
      wait_hover_ = false;
    }
    if (no_need_replan) {
      return;
    }

    // NOTE replan state
    Eigen::MatrixXd iniState;
    iniState.setZero(3, 3);
    ros::Time replan_stamp = ros::Time::now() + ros::Duration(0.03);
    double replan_t = (replan_stamp - replan_stamp_).toSec();//上次规划到现在的时间
    if (force_hover_ || replan_t > traj_poly_.getTotalDuration()) {//上次规划的轨迹已经执行完了，就从当前状态重新规划
      // should replan from the hover state
      iniState.col(0) = odom_p;
      iniState.col(1) = odom_v;
    } else {//上次规划的轨迹还没执行完就重规划，就从上次规划的轨迹状态重新规划，保证新轨迹和上次规划的轨迹衔接平滑
      // should replan from the last trajectory
      Eigen::Vector3d traj_pos = traj_poly_.getPos(replan_t);
      if ((traj_pos - odom_p).norm() > 1.0) {
        ROS_WARN("[drone %d] traj-odom drift %.2fm, replan from odom", trajOptPtr_->drone_id_, (traj_pos - odom_p).norm());
        iniState.col(0) = odom_p;
        iniState.col(1) = odom_v;
      } else {
        iniState.col(0) = traj_pos;
        iniState.col(1) = traj_poly_.getVel(replan_t);
        iniState.col(2) = traj_poly_.getAcc(replan_t);
      }
    }
    replanStateMsg_.header.stamp = ros::Time::now();
    replanStateMsg_.iniState.resize(9);
    Eigen::Map<Eigen::MatrixXd>(replanStateMsg_.iniState.data(), 3, 3) = iniState;

    // NOTE generate an extra corridor
    Eigen::Vector3d p_start = iniState.col(0);
    bool need_extra_corridor = iniState.col(1).norm() > 1.0;//如果重规划时无人机速度很大就需要额外安全走廊
    Eigen::MatrixXd hPoly;
    std::pair<Eigen::Vector3d, Eigen::Vector3d> line;
    if (need_extra_corridor) {//额外安全走廊的规划逻辑就是当前位置沿着当前速度方向前进，直到碰撞或者前进1m，形成一个线段，然后以这个线段为中心生成一个走廊
      Eigen::Vector3d v_norm = iniState.col(1).normalized();
      line.first = p_start;
      double step = 0.1;
      for (double dx = step; dx < 1.0; dx += step) {
        p_start += step * v_norm;
        if (gridmapPtr_->isOccupied(p_start)) {
          p_start -= step * v_norm;
          break;
        }
      }
      line.second = p_start;
      envPtr_->generateOneCorridor(line, 2.0, hPoly);
    }
    // NOTE path searching
    std::vector<Eigen::Vector3d> path;
    bool generate_new_traj_success = envPtr_->astar_search(p_start, local_goal, path);//env类中定义的A*搜索，已经考虑了避障
    Trajectory traj;
    if (generate_new_traj_success) {
      visPtr_->visualize_path(path, "astar");
      // NOTE corridor generating
      std::vector<Eigen::MatrixXd> hPolys;
      std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> keyPts;
      envPtr_->generateSFC(path, 2.0, hPolys, keyPts);
      if (need_extra_corridor) {
        hPolys.insert(hPolys.begin(), hPoly);
        keyPts.insert(keyPts.begin(), line);
      }
      envPtr_->visCorridor(hPolys);
      visPtr_->visualize_pairline(keyPts, "keyPts");//生成了安全走廊hPolys（每个元素表示每段走廊的多边形数据），每段走廊的代表线段keyPts

      // NOTE trajectory optimization
      Eigen::MatrixXd finState;
      finState.setZero(3, 3);
      finState.col(0) = path.back();
      // return;
      generate_new_traj_success = trajOptPtr_->generate_traj(iniState, finState, hPolys, traj);//轨迹优化！！
      visPtr_->visualize_traj(traj, "traj");
    }

    // NOTE collision check
    bool valid = false;
    if (generate_new_traj_success) {
      valid = validcheck(traj, replan_stamp);
    } else {
      replanStateMsg_.state = -2;
      replanState_pub_.publish(replanStateMsg_);
    }
    if (valid) {
      force_hover_ = false;
      trajOptPtr_->emergency_recovery_ = false;
      ROS_WARN("[planner] REPLAN SUCCESS");
      replanStateMsg_.state = 0;
      replanState_pub_.publish(replanStateMsg_);
      // NOTE : if the trajectory is known, watch that direction
      Eigen::Vector3d un_known_p = traj.getPos(traj.getTotalDuration() < 1.0 ? traj.getTotalDuration() : 1.0);
      Eigen::Vector3d dp = un_known_p - odom_p;
      double yaw = std::atan2(dp.y(), dp.x());
      pub_traj(traj, yaw, replan_stamp);
      traj_poly_ = traj;
      replan_stamp_ = replan_stamp;
    } else if (force_hover_) {
      ROS_ERROR("[drone %d planner] REPLAN FAILED, HOVERING...", trajOptPtr_->drone_id_);
      replanStateMsg_.state = 1;
      replanState_pub_.publish(replanStateMsg_);
      return;
    } else if (!validcheck(traj_poly_, replan_stamp_)) {
      force_hover_ = true;
      ROS_FATAL("[drone %d planner] EMERGENCY STOP!!!", trajOptPtr_->drone_id_);
      replanStateMsg_.state = 2;
      replanState_pub_.publish(replanStateMsg_);
      trajOptPtr_->emergency_recovery_ = true;
      emergency_brake(iniState, replan_stamp);
      return;
    } else {
      // Update map with latest data before executing old trajectory
      while (gridmap_lock_.test_and_set());
      gridmapPtr_->from_msg(map_msg_);
      gridmap_lock_.clear();

      // Verify old trajectory with latest map
      if (!validcheck(traj_poly_, replan_stamp_)) {
        force_hover_ = true;
        ROS_FATAL("[drone %d planner] EMERGENCY STOP - OLD TRAJ INVALID WITH LATEST MAP!!!", trajOptPtr_->drone_id_);
        replanStateMsg_.state = 2;
        replanState_pub_.publish(replanStateMsg_);
        trajOptPtr_->emergency_recovery_ = true;
      emergency_brake(iniState, replan_stamp);
        return;
      }

      ROS_ERROR("[drone %d planner] REPLAN FAILED, EXECUTE LAST TRAJ...", trajOptPtr_->drone_id_);
      replanStateMsg_.state = 3;
      replanState_pub_.publish(replanStateMsg_);
      trajOptPtr_->emergency_recovery_ = true;
      return;  // current generated traj invalid but last is valid
    }
    visPtr_->visualize_traj(traj, "traj");
  }

  void debug_timer_callback(const ros::TimerEvent& event) {
    inflate_gridmap_pub_.publish(replanStateMsg_.occmap);
    Eigen::MatrixXd iniState;
    iniState.setZero(3, 3);
    ros::Time replan_stamp = ros::Time::now() + ros::Duration(0.03);

    iniState = Eigen::Map<Eigen::MatrixXd>(replanStateMsg_.iniState.data(), 3, 3);
    Eigen::Vector3d target_p(replanStateMsg_.target.pose.pose.position.x,
                             replanStateMsg_.target.pose.pose.position.y,
                             replanStateMsg_.target.pose.pose.position.z);
    Eigen::Vector3d target_v(replanStateMsg_.target.twist.twist.linear.x,
                             replanStateMsg_.target.twist.twist.linear.y,
                             replanStateMsg_.target.twist.twist.linear.z);
    // std::cout << "target_p: " << target_p.transpose() << std::endl;
    // std::cout << "target_v: " << target_v.transpose() << std::endl;

    // visualize the target and the drone velocity
    visPtr_->visualize_arrow(iniState.col(0), iniState.col(0) + iniState.col(1), "drone_vel");
    visPtr_->visualize_arrow(target_p, target_p + target_v, "target_vel");

    // visualize the ray from drone to target
    if (envPtr_->checkRayValid(iniState.col(0), target_p)) {
      visPtr_->visualize_arrow(iniState.col(0), target_p, "ray", visualization::yellow);
    } else {
      visPtr_->visualize_arrow(iniState.col(0), target_p, "ray", visualization::red);
    }

    // NOTE prediction
    std::vector<Eigen::Vector3d> target_predcit;
    if (gridmapPtr_->isOccupied(target_p)) {
      std::cout << "target is invalid!" << std::endl;
      assert(false);
    }
    bool generate_new_traj_success = prePtr_->predict(target_p, target_v, target_predcit);

    if (generate_new_traj_success) {
      Eigen::Vector3d observable_p = target_predcit.back();
      visPtr_->visualize_path(target_predcit, "car_predict");
      std::vector<Eigen::Vector3d> observable_margin;
      for (double theta = 0; theta <= 2 * M_PI; theta += 0.01) {
        observable_margin.emplace_back(observable_p + tracking_dist_ * Eigen::Vector3d(cos(theta), sin(theta), 0));
      }
      visPtr_->visualize_path(observable_margin, "observable_margin");
    }

    // NOTE path searching
    Eigen::Vector3d p_start = iniState.col(0);
    std::vector<Eigen::Vector3d> path, way_pts;
    if (generate_new_traj_success) {
      generate_new_traj_success = envPtr_->findVisiblePath(p_start, target_predcit, way_pts, path);
    }

    std::vector<Eigen::Vector3d> visible_ps;
    std::vector<double> thetas;
    Trajectory traj;
    if (generate_new_traj_success) {
      visPtr_->visualize_path(path, "astar");
      // NOTE generate visible regions
      target_predcit.pop_back();
      way_pts.pop_back();
      envPtr_->generate_visible_regions(target_predcit, way_pts,
                                        visible_ps, thetas);
      visPtr_->visualize_pointcloud(visible_ps, "visible_ps");
      visPtr_->visualize_fan_shape_meshes(target_predcit, visible_ps, thetas, "visible_region");
      // NOTE corridor generating
      std::vector<Eigen::MatrixXd> hPolys;
      std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> keyPts;
      // TODO change the final state
      std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> rays;
      for (int i = 0; i < (int)way_pts.size(); ++i) {
        rays.emplace_back(target_predcit[i], way_pts[i]);
      }
      visPtr_->visualize_pointcloud(way_pts, "way_pts");
      way_pts.insert(way_pts.begin(), p_start);
      envPtr_->pts2path(way_pts, path);
      visPtr_->visualize_path(path, "corridor_path");
      envPtr_->generateSFC(path, 2.0, hPolys, keyPts);
      envPtr_->visCorridor(hPolys);
      visPtr_->visualize_pairline(keyPts, "keyPts");

      // NOTE trajectory optimization
      Eigen::MatrixXd finState;
      finState.setZero(3, 3);
      finState.col(0) = path.back();
      finState.col(1) = target_v;

      generate_new_traj_success = trajOptPtr_->generate_traj(iniState, finState,
                                                             target_predcit, hPolys, path, traj);
      visPtr_->visualize_traj(traj, "traj");
    }
    if (!generate_new_traj_success) {
      return;
      // assert(false);
    }
    // check
    bool valid = true;
    std::vector<Eigen::Vector3d> check_pts, invalid_pts;
    double t0 = (ros::Time::now() - replan_stamp).toSec();
    t0 = t0 > 0.0 ? t0 : 0.0;
    double check_dur = 1.0;
    double delta_t = check_dur < traj.getTotalDuration() ? check_dur : traj.getTotalDuration();
    for (double t = t0; t < t0 + delta_t; t += 0.1) {
      Eigen::Vector3d p = traj.getPos(t);
      check_pts.push_back(p);
      if (gridmapPtr_->isOccupied(p)) {
        invalid_pts.push_back(p);
      }
    }
    visPtr_->visualize_path(invalid_pts, "invalid_pts");
    visPtr_->visualize_path(check_pts, "check_pts");
    valid = validcheck(traj, replan_stamp);
    if (!valid) {
      std::cout << "invalid!" << std::endl;
    }
  }

  bool validcheck(const Trajectory& traj, const ros::Time& t_start, const double& check_dur = 2.0) {
    double t0 = (ros::Time::now() - t_start).toSec();
    t0 = t0 > 0.0 ? t0 : 0.0;
    double total_dur = traj.getTotalDuration();

    // 如果轨迹已经过期，认为无效
    if (t0 >= total_dur) {
      last_validcheck_fail_reason_ = "expired";
      last_validcheck_fail_t_ = t0;
      last_validcheck_fail_p_ = traj.getPos(total_dur);
      ROS_WARN_THROTTLE(1.0, "[validcheck] trajectory expired: t0=%.2f >= total_dur=%.2f", t0, total_dur);
      return false;
    }

    // 检查从当前时刻到轨迹结束或 check_dur 内的碰撞
    double t_end = std::min(t0 + check_dur, total_dur);
    for (double t = t0; t < t_end; t += 0.01) {
      Eigen::Vector3d p = traj.getPos(t);
      if (gridmapPtr_->isOccupied(p)) {
        last_validcheck_fail_reason_ = "occupied";
        last_validcheck_fail_t_ = t;
        last_validcheck_fail_p_ = p;
        ROS_WARN_THROTTLE(0.2,
                          "[validcheck] occupied at t=%.2f (window=[%.2f, %.2f]), p=[%.2f %.2f %.2f]",
                          t, t0, t_end, p.x(), p.y(), p.z());
        return false;
      }
    }
    last_validcheck_fail_reason_ = "none";
    last_validcheck_fail_t_ = -1.0;
    last_validcheck_fail_p_.setZero();
    return true;
  }

  void init(ros::NodeHandle& nh) {
    // set parameters of planning
    int plan_hz = 10;
    nh.getParam("plan_hz", plan_hz);
    nh.getParam("tracking_dur", tracking_dur_);
    nh.getParam("tracking_dist", tracking_dist_);
    nh.getParam("tolerance_d", tolerance_d_);
    nh.param("hard_replan_hz", hard_replan_hz_, 5.0);
    hard_replan_hz_ = std::max(0.1, hard_replan_hz_);
    hard_replan_interval_ = 1.0 / hard_replan_hz_;
    nh.param("obs_clip_enable", obs_clip_enable_, true);
    nh.param("obs_clip_range", obs_clip_range_, 10.0);
    nh.param("obs_clip_margin", obs_clip_margin_, 0.5);
    obs_clip_range_ = std::max(0.5, obs_clip_range_);
    obs_clip_margin_ = std::max(0.0, std::min(obs_clip_margin_, obs_clip_range_ - 0.1));
    nh.param("formation_heading_speed_thresh", formation_heading_speed_thresh_, 0.2);
    nh.param("search_yaw_scan_range_deg", search_yaw_scan_range_deg_, 45.0);
    nh.param("search_yaw_scan_freq_hz", search_yaw_scan_freq_hz_, 0.25);
    nh.param("post_reacquire_boost_sec", post_reacquire_boost_sec_, 3.0);
    post_reacquire_boost_sec_ = std::max(0.0, post_reacquire_boost_sec_);
    search_yaw_scan_range_deg_ = std::max(0.0, search_yaw_scan_range_deg_);
    search_yaw_scan_freq_hz_ = std::max(0.0, search_yaw_scan_freq_hz_);
    nh.getParam("debug", debug_);
    nh.getParam("fake", fake_);
    nh.getParam("vmax", vmax_);
    nh.getParam("amax", amax_);

    gridmapPtr_ = std::make_shared<mapping::OccGridMap>();
    envPtr_ = std::make_shared<env::Env>(nh, gridmapPtr_);
    visPtr_ = std::make_shared<visualization::Visualization>(nh);
    trajOptPtr_ = std::make_shared<traj_opt::TrajOpt>(nh);
    trajOptPtr_->setMap(gridmapPtr_.get());
    prePtr_ = std::make_shared<prediction::Predict>(nh);

    heartbeat_pub_ = nh.advertise<std_msgs::Empty>("heartbeat", 10);
    traj_pub_ = nh.advertise<quadrotor_msgs::PolyTraj>("trajectory", 1);
    replanState_pub_ = nh.advertise<quadrotor_msgs::ReplanState>("replanState", 1);
    broadcast_traj_pub_ = nh.advertise<quadrotor_msgs::PolyTraj>("/planning/broadcast_traj_send", 10);

    if (debug_) {
      plan_timer_ = nh.createTimer(ros::Duration(1.0 / plan_hz), &Nodelet::debug_timer_callback, this);
      // TODO read debug data from files
      wr_msg::readMsg(replanStateMsg_, ros::package::getPath("planning") + "/../../../debug/replan_state.bin");
      inflate_gridmap_pub_ = nh.advertise<quadrotor_msgs::OccMap3d>("gridmap_inflate", 10);
      gridmapPtr_->from_msg(replanStateMsg_.occmap);
      prePtr_->setMap(*gridmapPtr_);
      std::cout << "plan state: " << replanStateMsg_.state << std::endl;
    } else if (fake_) {
      plan_timer_ = nh.createTimer(ros::Duration(1.0 / plan_hz), &Nodelet::fake_timer_callback, this);
    } else {
      plan_timer_ = nh.createTimer(ros::Duration(1.0 / plan_hz), &Nodelet::plan_timer_callback, this);
    }
    gridmap_sub_ = nh.subscribe<quadrotor_msgs::OccMap3d>("gridmap_inflate", 1, &Nodelet::gridmap_callback, this, ros::TransportHints().tcpNoDelay());
    odom_sub_ = nh.subscribe<nav_msgs::Odometry>("odom", 10, &Nodelet::odom_callback, this, ros::TransportHints().tcpNoDelay());
    target_sub_ = nh.subscribe<nav_msgs::Odometry>("target", 10, &Nodelet::target_callback, this, ros::TransportHints().tcpNoDelay());
    search_state_sub_ = nh.subscribe<std_msgs::Bool>("search_state", 10, &Nodelet::search_state_callback, this, ros::TransportHints().tcpNoDelay());
    search_targets_sub_ = nh.subscribe<geometry_msgs::PoseArray>("search_targets", 10, &Nodelet::search_targets_callback, this, ros::TransportHints().tcpNoDelay());

    // 订阅各掌管者的搜索标签信息（drone0-2 可能是掌管者）
    for (int i = 0; i < 3; ++i) {
      std::string topic = "/drone" + std::to_string(i) + "/drone" + std::to_string(i) + "_target_dpf/search_label_info";
      search_label_info_subs_.push_back(
          nh.subscribe<target_ekf::SearchLabelInfo>(topic, 10, &Nodelet::search_label_info_callback, this, ros::TransportHints().tcpNoDelay()));
    }

    triger_sub_ = nh.subscribe<geometry_msgs::PoseStamped>("triger", 10, &Nodelet::triger_callback, this, ros::TransportHints().tcpNoDelay());
    land_triger_sub_ = nh.subscribe<geometry_msgs::PoseStamped>("land_triger", 10, &Nodelet::land_triger_callback, this, ros::TransportHints().tcpNoDelay());
    broadcast_traj_sub_ = nh.subscribe<quadrotor_msgs::PolyTraj>("/planning/broadcast_traj_recv", 100,
                                                                 &Nodelet::RecvBroadcastPolyTrajCallback,
                                                                 this,
                                                                 ros::TransportHints().tcpNoDelay());

    ROS_WARN("Planning node initialized!");
  }

 public:
  void onInit(void) {
    ros::NodeHandle nh(getMTPrivateNodeHandle());
    initThread_ = std::thread(std::bind(&Nodelet::init, this, nh));
  }
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

}  // namespace planning

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(planning::Nodelet, nodelet::Nodelet);
