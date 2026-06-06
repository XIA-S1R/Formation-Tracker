#pragma once
#include <ros/ros.h>

#include "minco.hpp"
#include <swarm_graph/swarm_graph.hpp>
#include <mapping/mapping.h>

struct SwarmTrajData {
  int drone_id = -1;
  int traj_id = -1;
  double start_time = 0.0;
  Trajectory traj;
  double duration = 0.0;
  Eigen::Vector3d start_pos;
};

namespace traj_opt {

class TrajOpt {
 public:
  ros::NodeHandle nh_;
  // # pieces and # key points
  int N_, K_, dim_t_, dim_p_;
  // weight for time regularization term
  double rhoT_;
  // collision avoiding and dynamics paramters
  double vmax_, amax_;
  double rhoP_, rhoV_, rhoA_;
  double rhoTracking_, rhosVisibility_;
  double clearance_d_, tolerance_d_, theta_clearance_;
  double rhoSwarm_;
  // gridmap pointer for obstacle distance penalty
  mapping::OccGridMap* mapPtr_ = nullptr;
  // corridor
  std::vector<Eigen::MatrixXd> cfgVs_;
  std::vector<Eigen::MatrixXd> cfgHs_;
  // Minimum Jerk Optimizer
  minco::MinJerkOpt jerkOpt_;
  // weight for each vertex
  Eigen::VectorXd p_;
  // duration of each piece of the trajectory
  Eigen::VectorXd t_;
  std::vector<double> x_buffer_;
  double* x_ = nullptr;
  size_t x_capacity_ = 0;
  double sum_T_;

  std::vector<Eigen::Vector3d> tracking_ps_;
  std::vector<Eigen::Vector3d> tracking_visible_ps_;
  std::vector<double> tracking_thetas_;
  double tracking_dur_;
  double tracking_dist_;
  double tracking_dt_;

  // Formation-related parameters
  SwarmGraph::Ptr swarm_graph_;
  double wei_formation_ = 0.0;
  double formation_grad_clip_ = 5.0;  // <=0: disable clip, >0: clip each drone formation grad norm
  int formation_size_ = 0, drone_id_ = -1;
  std::vector<SwarmTrajData> swarm_trajs_;
  bool use_formation_ = false;
  int formation_type_ = 0;
  Eigen::Vector3d formation_offset_;  // 本机在期望编队中的偏移（相对编队质心），仅供外部参考，不用于锁定 finState
  bool use_soft_constraint_ = false;  // true: ESDF软约束, false: 走廊硬约束
  bool use_tracking_cost_ = false;    // false: disable tracking cost in trajectory optimization
  bool emergency_recovery_ = false;   // 紧急刹车后首次规划：跳过编队代价和tracking代价
  bool suppress_formation_cost_ = false; // 临时关闭编队代价（不影响编队前端偏置和tracking代价）

  // Absolute time at the start of each optimization call (fixed during one L-BFGS run)
  double t_now_;

  // Debug: per-optimization-call cost breakdown (accumulated in addTimeIntPenalty / addTimeCost)
  double debug_cost_corridor_  = 0;
  double debug_cost_vel_       = 0;
  double debug_cost_acc_       = 0;
  double debug_cost_collision_ = 0;
  double debug_cost_formation_ = 0;
  double debug_cost_tracking_  = 0;
  double debug_cost_vis_       = 0;
  bool   debug_print_once_     = false;  // set true before each optimize() call, cleared after first print

  // polyH utils
  bool extractVs(const std::vector<Eigen::MatrixXd>& hPs,
                 std::vector<Eigen::MatrixXd>& vPs) const;
  void ensureWorkspace(size_t required_size);

 public:
  TrajOpt(ros::NodeHandle& nh);
  ~TrajOpt() {}

  // --- Soft constraint (ESDF) ---
  void setBoundConds(const Eigen::MatrixXd& iniState, const Eigen::MatrixXd& finState,
                     const std::vector<Eigen::Vector3d>& path = {});
  int optimize(const double& delta = 1e-4);
  bool generate_traj(const Eigen::MatrixXd& iniState,
                     const Eigen::MatrixXd& finState,
                     const std::vector<Eigen::Vector3d>& target_predcit,
                     const std::vector<Eigen::Vector3d>& visible_ps,
                     const std::vector<double>& thetas,
                     const std::vector<Eigen::MatrixXd>& hPolys,
                     Trajectory& traj);
  bool generate_traj(const Eigen::MatrixXd& iniState,
                     const Eigen::MatrixXd& finState,
                     const std::vector<Eigen::Vector3d>& target_predcit,
                     const std::vector<Eigen::MatrixXd>& hPolys,
                     const std::vector<Eigen::Vector3d>& path,
                     Trajectory& traj);
  bool generate_traj(const Eigen::MatrixXd& iniState,
                     const Eigen::MatrixXd& finState,
                     const std::vector<Eigen::MatrixXd>& hPolys,
                     Trajectory& traj);

  // --- Hard constraint (corridor) ---
  void setBoundCondsHard(const Eigen::MatrixXd& iniState, const Eigen::MatrixXd& finState);
  int optimizeHard(const double& delta = 1e-4);
  bool generate_traj_hard(const Eigen::MatrixXd& iniState,
                          const Eigen::MatrixXd& finState,
                          const std::vector<Eigen::Vector3d>& target_predcit,
                          const std::vector<Eigen::MatrixXd>& hPolys,
                          Trajectory& traj);
  bool generate_traj_hard(const Eigen::MatrixXd& iniState,
                          const Eigen::MatrixXd& finState,
                          const std::vector<Eigen::MatrixXd>& hPolys,
                          Trajectory& traj);

  void addTimeIntPenalty(double& cost);
  void addTimeCost(double& cost);
  void setDesiredFormation(int type);
  void setSwarmTrajs(const std::vector<SwarmTrajData>& swarm_trajs) { swarm_trajs_ = swarm_trajs; }
  void setMap(mapping::OccGridMap* map) { mapPtr_ = map; }

  // Formation cost
  bool grad_cost_swarm_formation(const int piece,
                                 const double t,
                                 const Eigen::Vector3d& p,
                                 const Eigen::Vector3d& v,
                                 Eigen::Vector3d& gradp,
                                 double& gradt,
                                 double& grad_prev_t,
                                 double& costp);

  // Swarm collision avoidance cost (ellipsoidal)
  bool grad_cost_swarm_collision(const int piece,
                                 const double t,
                                 const Eigen::Vector3d& p,
                                 const Eigen::Vector3d& v,
                                 Eigen::Vector3d& gradp,
                                 double& gradt,
                                 double& grad_prev_t,
                                 double& costp);

  bool grad_cost_p_corridor(const Eigen::Vector3d& p,
                            const Eigen::MatrixXd& hPoly,
                            Eigen::Vector3d& gradp,
                            double& costp);
  // Gridmap-based smooth obstacle penalty (replaces corridor penalty in soft constraint mode)
  bool grad_cost_obstacle(const Eigen::Vector3d& p,
                          Eigen::Vector3d& gradp,
                          double& costp);
  bool grad_cost_p_tracking(const Eigen::Vector3d& p,
                            const Eigen::Vector3d& target_p,
                            Eigen::Vector3d& gradp,
                            double& costp);
  bool grad_cost_p_landing(const Eigen::Vector3d& p,
                           const Eigen::Vector3d& target_p,
                           Eigen::Vector3d& gradp,
                           double& costp);
  bool grad_cost_visibility(const Eigen::Vector3d& p,
                            const Eigen::Vector3d& center,
                            const Eigen::Vector3d& vis_p,
                            const double& theta,
                            Eigen::Vector3d& gradp,
                            double& costp);
  bool grad_cost_v(const Eigen::Vector3d& v,
                   Eigen::Vector3d& gradv,
                   double& costv);
  bool grad_cost_a(const Eigen::Vector3d& a,
                   Eigen::Vector3d& grada,
                   double& costa);
};

}  // namespace traj_opt
