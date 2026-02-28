#pragma once
#include <ros/ros.h>

#include "minco.hpp"
#include <swarm_graph/swarm_graph.hpp>

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
  // corridor
  std::vector<Eigen::MatrixXd> cfgVs_;
  std::vector<Eigen::MatrixXd> cfgHs_;
  // Minimum Jerk Optimizer
  minco::MinJerkOpt jerkOpt_;
  // weight for each vertex
  Eigen::VectorXd p_;
  // duration of each piece of the trajectory
  Eigen::VectorXd t_;
  double* x_;
  double sum_T_;

  std::vector<Eigen::Vector3d> tracking_ps_;
  std::vector<Eigen::Vector3d> tracking_visible_ps_;
  std::vector<double> tracking_thetas_;
  double tracking_dur_;
  double tracking_dist_;
  double tracking_dt_;

  // Formation-related parameters
  SwarmGraph::Ptr swarm_graph_;
  double wei_formation_;
  int formation_size_, drone_id_;
  std::vector<SwarmTrajData> swarm_trajs_;
  bool use_formation_;
  int formation_type_;

  // Absolute time at the start of each optimization call (fixed during one L-BFGS run)
  double t_now_;

  // polyH utils
  bool extractVs(const std::vector<Eigen::MatrixXd>& hPs,
                 std::vector<Eigen::MatrixXd>& vPs) const;

 public:
  TrajOpt(ros::NodeHandle& nh);
  ~TrajOpt() {}

  void setBoundConds(const Eigen::MatrixXd& iniState, const Eigen::MatrixXd& finState);
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
                     Trajectory& traj);
  bool generate_traj(const Eigen::MatrixXd& iniState,
                     const Eigen::MatrixXd& finState,
                     const std::vector<Eigen::MatrixXd>& hPolys,
                     Trajectory& traj);

  void addTimeIntPenalty(double& cost);
  void addTimeCost(double& cost);
  void setDesiredFormation(int type);
  void setSwarmTrajs(const std::vector<SwarmTrajData>& swarm_trajs) { swarm_trajs_ = swarm_trajs; }

  // Formation cost: returns true if penalty was applied.
  // t: accumulated trajectory time at the current sample point (= sum of durations of previous pieces + s1)
  // piece: index of the current trajectory piece (needed for grad_prev_t propagation)
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