#include <traj_opt/traj_opt.h>

#include <random>
#include <traj_opt/geoutils.hpp>
#include <traj_opt/lbfgs_raw.hpp>

namespace traj_opt {

static bool landing_ = false;

// SECTION  variables transformation and gradient transmission
static double expC2(double t) {
  return t > 0.0 ? ((0.5 * t + 1.0) * t + 1.0)
                 : 1.0 / ((0.5 * t - 1.0) * t + 1.0);
}
static double logC2(double T) {
  return T > 1.0 ? (sqrt(2.0 * T - 1.0) - 1.0) : (1.0 - sqrt(2.0 / T - 1.0));
}
static void forwardT(const Eigen::Ref<const Eigen::VectorXd>& t, const double& sT, Eigen::Ref<Eigen::VectorXd> vecT) {
  int M = t.size();
  for (int i = 0; i < M; ++i) {
    vecT(i) = expC2(t(i));
  }
  vecT(M) = 0.0;
  vecT /= 1.0 + vecT.sum();
  vecT(M) = 1.0 - vecT.sum();
  vecT *= sT;
  return;
}
static void backwardT(const Eigen::Ref<const Eigen::VectorXd>& vecT, Eigen::Ref<Eigen::VectorXd> t) {
  int M = t.size();
  t = vecT.head(M) / vecT(M);
  for (int i = 0; i < M; ++i) {
    t(i) = logC2(vecT(i));
  }
  return;
}
static void addLayerTGrad(const Eigen::Ref<const Eigen::VectorXd>& t,
                          const double& sT,
                          const Eigen::Ref<const Eigen::VectorXd>& gradT,
                          Eigen::Ref<Eigen::VectorXd> gradt) {
  int Ms1 = t.size();
  Eigen::VectorXd gFree = sT * gradT.head(Ms1);
  double gTail = sT * gradT(Ms1);
  Eigen::VectorXd dExpTau(Ms1);
  double expTauSum = 0.0, gFreeDotExpTau = 0.0;
  double denSqrt, expTau;
  for (int i = 0; i < Ms1; i++) {
    if (t(i) > 0) {
      expTau = (0.5 * t(i) + 1.0) * t(i) + 1.0;
      dExpTau(i) = t(i) + 1.0;
      expTauSum += expTau;
      gFreeDotExpTau += expTau * gFree(i);
    } else {
      denSqrt = (0.5 * t(i) - 1.0) * t(i) + 1.0;
      expTau = 1.0 / denSqrt;
      dExpTau(i) = (1.0 - t(i)) / (denSqrt * denSqrt);
      expTauSum += expTau;
      gFreeDotExpTau += expTau * gFree(i);
    }
  }
  denSqrt = expTauSum + 1.0;
  gradt = (gFree.array() - gTail) * dExpTau.array() / denSqrt -
          (gFreeDotExpTau - gTail * expTauSum) * dExpTau.array() / (denSqrt * denSqrt);
}

static void forwardP(const Eigen::Ref<const Eigen::VectorXd>& p,
                     const std::vector<Eigen::MatrixXd>& cfgPolyVs,
                     Eigen::MatrixXd& inP) {
  int M = cfgPolyVs.size();
  Eigen::VectorXd q;
  int j = 0, k;
  for (int i = 0; i < M; ++i) {
    k = cfgPolyVs[i].cols() - 1;
    q = 2.0 / (1.0 + p.segment(j, k).squaredNorm()) * p.segment(j, k);
    inP.col(i) = cfgPolyVs[i].rightCols(k) * q.cwiseProduct(q) +
                 cfgPolyVs[i].col(0);
    j += k;
  }
  return;
}
static double objectiveNLS(void* ptrPOBs,
                           const double* x,
                           double* grad,
                           const int n) {
  const Eigen::MatrixXd& pobs = *(Eigen::MatrixXd*)ptrPOBs;
  Eigen::Map<const Eigen::VectorXd> p(x, n);
  Eigen::Map<Eigen::VectorXd> gradp(grad, n);

  double qnsqr = p.squaredNorm();
  double qnsqrp1 = qnsqr + 1.0;
  double qnsqrp1sqr = qnsqrp1 * qnsqrp1;
  Eigen::VectorXd r = 2.0 / qnsqrp1 * p;

  Eigen::Vector3d delta = pobs.rightCols(n) * r.cwiseProduct(r) +
                          pobs.col(1) - pobs.col(0);
  double cost = delta.squaredNorm();
  Eigen::Vector3d gradR3 = 2 * delta;

  Eigen::VectorXd gdr = pobs.rightCols(n).transpose() * gradR3;
  gdr = gdr.array() * r.array() * 2.0;
  gradp = gdr * 2.0 / qnsqrp1 -
          p * 4.0 * gdr.dot(p) / qnsqrp1sqr;

  return cost;
}

static void backwardP(const Eigen::Ref<const Eigen::MatrixXd>& inP,
                      const std::vector<Eigen::MatrixXd>& cfgPolyVs,
                      Eigen::VectorXd& p) {
  int M = inP.cols();
  int j = 0, k;

  // Parameters for tiny nonlinear least squares
  double minSqrD;
  lbfgs::lbfgs_parameter_t nls_params;
  lbfgs::lbfgs_load_default_parameters(&nls_params);
  nls_params.g_epsilon = FLT_EPSILON;
  nls_params.max_iterations = 128;

  Eigen::MatrixXd pobs;
  for (int i = 0; i < M; i++) {
    k = cfgPolyVs[i].cols() - 1;
    p.segment(j, k).setConstant(1.0 / (sqrt(k + 1.0) + 1.0));
    pobs.resize(3, k + 2);
    pobs << inP.col(i), cfgPolyVs[i];
    lbfgs::lbfgs_optimize(k,
                          p.data() + j,
                          &minSqrD,
                          &objectiveNLS,
                          nullptr,
                          nullptr,
                          &pobs,
                          &nls_params);
    j += k;
  }
  return;
}
static void addLayerPGrad(const Eigen::Ref<const Eigen::VectorXd>& p,
                          const std::vector<Eigen::MatrixXd>& cfgPolyVs,
                          const Eigen::Ref<const Eigen::MatrixXd>& gradInPs,
                          Eigen::Ref<Eigen::VectorXd> grad) {
  int M = gradInPs.cols();

  int j = 0, k;
  double qnsqr, qnsqrp1, qnsqrp1sqr;
  Eigen::VectorXd q, r, gdr;
  for (int i = 0; i < M; i++) {
    k = cfgPolyVs[i].cols() - 1;
    q = p.segment(j, k);
    qnsqr = q.squaredNorm();
    qnsqrp1 = qnsqr + 1.0;
    qnsqrp1sqr = qnsqrp1 * qnsqrp1;
    r = 2.0 / qnsqrp1 * q;
    gdr = cfgPolyVs[i].rightCols(k).transpose() * gradInPs.col(i);
    gdr = gdr.array() * r.array() * 2.0;

    grad.segment(j, k) = gdr * 2.0 / qnsqrp1 -
                         q * 4.0 * gdr.dot(q) / qnsqrp1sqr;
    j += k;
  }
  return;
}
// !SECTION variables transformation and gradient transmission

// SECTION object function

// === OLD objectiveFunc (hard constraint via forwardP) ===
// static inline double objectiveFunc(void* ptrObj,
//                                    const double* x,
//                                    double* grad,
//                                    const int n) {
//   TrajOpt& obj = *(TrajOpt*)ptrObj;
//
//   Eigen::Map<const Eigen::VectorXd> t(x, obj.dim_t_);
//   Eigen::Map<const Eigen::VectorXd> p(x + obj.dim_t_, obj.dim_p_);
//   Eigen::Map<Eigen::VectorXd> gradt(grad, obj.dim_t_);
//   Eigen::Map<Eigen::VectorXd> gradp(grad + obj.dim_t_, obj.dim_p_);
//   double deltaT = x[obj.dim_t_ + obj.dim_p_];
//
//   Eigen::VectorXd T(obj.N_);
//   Eigen::MatrixXd P(3, obj.N_ - 1);
//   double sumT = obj.sum_T_ + deltaT * deltaT;
//   forwardT(t, sumT, T);
//   forwardP(p, obj.cfgVs_, P);
//
//   obj.jerkOpt_.generate(P, T);
//   double cost = obj.jerkOpt_.getTrajJerkCost();
//   obj.jerkOpt_.calGrads_CT();
//
//   obj.debug_cost_corridor_  = 0;
//   obj.debug_cost_vel_       = 0;
//   obj.debug_cost_acc_       = 0;
//   obj.debug_cost_collision_ = 0;
//   obj.debug_cost_formation_ = 0;
//   obj.debug_cost_tracking_  = 0;
//   obj.debug_cost_vis_       = 0;
//
//   obj.addTimeIntPenalty(cost);
//   obj.addTimeCost(cost);
//   obj.jerkOpt_.calGrads_PT();
//   grad[obj.dim_t_ + obj.dim_p_] = obj.jerkOpt_.gdT.dot(T) / sumT + obj.rhoT_;
//   cost += obj.rhoT_ * deltaT * deltaT;
//   grad[obj.dim_t_ + obj.dim_p_] *= 2 * deltaT;
//   addLayerTGrad(t, sumT, obj.jerkOpt_.gdT, gradt);
//   addLayerPGrad(p, obj.cfgVs_, obj.jerkOpt_.gdP, gradp);
//
//   if (obj.debug_print_once_) {
//     obj.debug_print_once_ = false;
//     printf("\033[36m[drone %d cost] corridor=%.1f  vel=%.1f  acc=%.1f  "
//            "collision=%.1f  formation=%.1f  tracking=%.1f  vis=%.1f  total=%.1f\033[0m\n",
//            obj.drone_id_,
//            obj.debug_cost_corridor_, obj.debug_cost_vel_, obj.debug_cost_acc_,
//            obj.debug_cost_collision_, obj.debug_cost_formation_,
//            obj.debug_cost_tracking_, obj.debug_cost_vis_, cost);
//   }
//
//   return cost;
// }

// === NEW objectiveFunc (soft constraint: P directly optimized, corridor as penalty) ===
static inline double objectiveFunc(void* ptrObj,
                                   const double* x,
                                   double* grad,
                                   const int n) {
  TrajOpt& obj = *(TrajOpt*)ptrObj;

  Eigen::Map<const Eigen::VectorXd> t(x, obj.dim_t_);
  Eigen::Map<const Eigen::VectorXd> p(x + obj.dim_t_, obj.dim_p_);
  Eigen::Map<Eigen::VectorXd> gradt(grad, obj.dim_t_);
  Eigen::Map<Eigen::VectorXd> gradp(grad + obj.dim_t_, obj.dim_p_);
  double deltaT = x[obj.dim_t_ + obj.dim_p_];

  Eigen::VectorXd T(obj.N_);
  Eigen::MatrixXd P(3, obj.N_ - 1);
  double sumT = obj.sum_T_ + deltaT * deltaT;
  forwardT(t, sumT, T);

  // Soft constraint: P directly from optimization variables
  for (int i = 0; i < obj.N_ - 1; ++i) {
    P.col(i) = p.segment<3>(3 * i);
  }

  obj.jerkOpt_.generate(P, T);
  double cost = obj.jerkOpt_.getTrajJerkCost();
  obj.jerkOpt_.calGrads_CT();

  obj.debug_cost_corridor_  = 0;
  obj.debug_cost_vel_       = 0;
  obj.debug_cost_acc_       = 0;
  obj.debug_cost_collision_ = 0;
  obj.debug_cost_formation_ = 0;
  obj.debug_cost_tracking_  = 0;
  obj.debug_cost_vis_       = 0;

  obj.addTimeIntPenalty(cost);
  obj.addTimeCost(cost);
  obj.jerkOpt_.calGrads_PT();
  grad[obj.dim_t_ + obj.dim_p_] = obj.jerkOpt_.gdT.dot(T) / sumT + obj.rhoT_;
  cost += obj.rhoT_ * deltaT * deltaT;
  grad[obj.dim_t_ + obj.dim_p_] *= 2 * deltaT;
  addLayerTGrad(t, sumT, obj.jerkOpt_.gdT, gradt);

  // Soft constraint: gradient directly from gdP
  for (int i = 0; i < obj.N_ - 1; ++i) {
    gradp.segment<3>(3 * i) = obj.jerkOpt_.gdP.col(i);
  }

  if (obj.debug_print_once_) {
    obj.debug_print_once_ = false;
    printf("\033[36m[drone %d cost] corridor=%.1f  vel=%.1f  acc=%.1f  "
           "collision=%.1f  formation=%.1f  tracking=%.1f  vis=%.1f  total=%.1f\033[0m\n",
           obj.drone_id_,
           obj.debug_cost_corridor_, obj.debug_cost_vel_, obj.debug_cost_acc_,
           obj.debug_cost_collision_, obj.debug_cost_formation_,
           obj.debug_cost_tracking_, obj.debug_cost_vis_, cost);
  }

  return cost;
}

static inline int earlyExit(void* ptrObj,
                            const double* x,
                            const double* grad,
                            const double fx,
                            const double xnorm,
                            const double gnorm,
                            const double step,
                            int n,
                            int k,
                            int ls) {
  return k > 1e3;
}

bool TrajOpt::extractVs(const std::vector<Eigen::MatrixXd>& hPs,
                        std::vector<Eigen::MatrixXd>& vPs) const {
  const int M = hPs.size() - 1;

  vPs.clear();
  vPs.reserve(2 * M + 1);

  int nv;
  Eigen::MatrixXd curIH, curIV, curIOB;
  for (int i = 0; i < M; i++) {
    if (!geoutils::enumerateVs(hPs[i], curIV)) {
      return false;
    }
    nv = curIV.cols();
    curIOB.resize(3, nv);
    curIOB << curIV.col(0), curIV.rightCols(nv - 1).colwise() - curIV.col(0);
    vPs.push_back(curIOB);

    curIH.resize(6, hPs[i].cols() + hPs[i + 1].cols());
    curIH << hPs[i], hPs[i + 1];
    if (!geoutils::enumerateVs(curIH, curIV)) {
      return false;
    }
    nv = curIV.cols();
    curIOB.resize(3, nv);
    curIOB << curIV.col(0), curIV.rightCols(nv - 1).colwise() - curIV.col(0);
    vPs.push_back(curIOB);
  }

  if (!geoutils::enumerateVs(hPs.back(), curIV)) {
    return false;
  }
  nv = curIV.cols();
  curIOB.resize(3, nv);
  curIOB << curIV.col(0), curIV.rightCols(nv - 1).colwise() - curIV.col(0);
  vPs.push_back(curIOB);

  return true;
}

void TrajOpt::ensureWorkspace(size_t required_size) {
  if (required_size <= x_capacity_) {
    return;
  }
  x_buffer_.resize(required_size);
  x_capacity_ = x_buffer_.size();
  x_ = x_buffer_.data();
}

TrajOpt::TrajOpt(ros::NodeHandle& nh) : nh_(nh) {
  // nh.getParam("N", N_);
  nh.getParam("K", K_);
  // load dynamic paramters
  nh.getParam("vmax", vmax_);
  nh.getParam("amax", amax_);
  nh.getParam("rhoT", rhoT_);
  nh.getParam("rhoP", rhoP_);
  nh.getParam("rhoTracking", rhoTracking_);
  nh.getParam("rhosVisibility", rhosVisibility_);
  nh.getParam("theta_clearance", theta_clearance_);
  nh.getParam("rhoV", rhoV_);
  nh.getParam("rhoA", rhoA_);
  nh.getParam("tracking_dur", tracking_dur_);
  nh.getParam("tracking_dist", tracking_dist_);
  nh.getParam("tracking_dt", tracking_dt_);
  nh.getParam("clearance_d", clearance_d_);
  nh.getParam("tolerance_d", tolerance_d_);

  // Formation-related parameters
  nh.param("optimization/weight_formation", wei_formation_, 0.0);
  nh.param("optimization/formation_grad_clip", formation_grad_clip_, 5.0);
  nh.param("optimization/formation_size", formation_size_, 0);
  nh.param("optimization/drone_id", drone_id_, -1);
  nh.param("optimization/use_formation", use_formation_, false);
  nh.param("optimization/formation_type", formation_type_, 0);
  nh.param("optimization/use_tracking_cost", use_tracking_cost_, false);
  // Backward-compatible flat name.
  nh.param("use_tracking_cost", use_tracking_cost_, use_tracking_cost_);

  if (!nh.getParam("rhoSwarm", rhoSwarm_)) {
    rhoSwarm_ = 1000.0;
  }
  nh.param("use_soft_constraint", use_soft_constraint_, false);

  // Initialize SwarmGraph
  swarm_graph_.reset(new SwarmGraph());
  setDesiredFormation(formation_type_);

  swarm_trajs_.clear(); // To be set externally
}

// === OLD setBoundConds (hard constraint via backwardP) ===
// void TrajOpt::setBoundConds(const Eigen::MatrixXd& iniState,
//                             const Eigen::MatrixXd& finState) {
//   Eigen::MatrixXd initS = iniState;
//   Eigen::MatrixXd finalS = finState;
//   double tempNorm = initS.col(1).norm();
//   initS.col(1) *= tempNorm > vmax_ ? (vmax_ / tempNorm) : 1.0;
//   tempNorm = finalS.col(1).norm();
//   finalS.col(1) *= tempNorm > vmax_ ? (vmax_ / tempNorm) : 1.0;
//   tempNorm = initS.col(2).norm();
//   initS.col(2) *= tempNorm > amax_ ? (amax_ / tempNorm) : 1.0;
//   tempNorm = finalS.col(2).norm();
//   finalS.col(2) *= tempNorm > amax_ ? (amax_ / tempNorm) : 1.0;
//   // 对初始状态和最终状态的速度和加速度进行限制
//
//   Eigen::VectorXd T(N_);
//   T.setConstant(sum_T_ / N_);
//   backwardT(T, t_);
//   Eigen::MatrixXd P(3, N_ - 1);
//   for (int i = 0; i < N_ - 1; ++i) {
//     int k = cfgVs_[i].cols() - 1;
//     P.col(i) = cfgVs_[i].rightCols(k).rowwise().sum() / (1.0 + k) + cfgVs_[i].col(0);
//   }
//   backwardP(P, cfgVs_, p_);
//   jerkOpt_.reset(initS, finalS, N_);
//   return;
// }

// === NEW setBoundConds (soft constraint: P directly flattened to p_) ===
void TrajOpt::setBoundConds(const Eigen::MatrixXd& iniState,
                            const Eigen::MatrixXd& finState,
                            const std::vector<Eigen::Vector3d>& path) {
  Eigen::MatrixXd initS = iniState;
  Eigen::MatrixXd finalS = finState;
  double tempNorm = initS.col(1).norm();
  initS.col(1) *= tempNorm > vmax_ ? (vmax_ / tempNorm) : 1.0;
  tempNorm = finalS.col(1).norm();
  finalS.col(1) *= tempNorm > vmax_ ? (vmax_ / tempNorm) : 1.0;
  tempNorm = initS.col(2).norm();
  initS.col(2) *= tempNorm > amax_ ? (amax_ / tempNorm) : 1.0;
  tempNorm = finalS.col(2).norm();
  finalS.col(2) *= tempNorm > amax_ ? (amax_ / tempNorm) : 1.0;

  Eigen::VectorXd T(N_);
  T.setConstant(sum_T_ / N_);
  backwardT(T, t_);

  Eigen::MatrixXd P(3, N_ - 1);
  if (path.size() >= 2) {
    // 沿 path 计算累积弧长，然后等距采样 N_-1 个 P 点
    std::vector<double> arclen(path.size(), 0.0);
    for (size_t j = 1; j < path.size(); ++j) {
      arclen[j] = arclen[j - 1] + (path[j] - path[j - 1]).norm();
    }
    double total_len = arclen.back();
    for (int i = 0; i < N_ - 1; ++i) {
      double target_len = total_len * (double)(i + 1) / N_;
      // 二分查找所在线段
      size_t lo = 0, hi = path.size() - 1;
      while (lo + 1 < hi) {
        size_t mid = (lo + hi) / 2;
        if (arclen[mid] < target_len) lo = mid; else hi = mid;
      }
      double seg_len = arclen[hi] - arclen[lo];
      double alpha = seg_len > 1e-6 ? (target_len - arclen[lo]) / seg_len : 0.0;
      P.col(i) = (1.0 - alpha) * path[lo] + alpha * path[hi];
    }
  } else if ((int)cfgVs_.size() >= N_ - 1) {
    // Fallback: 走廊中心（假目标等不传 path 的情况）
    for (int i = 0; i < N_ - 1; ++i) {
      int k = cfgVs_[i].cols() - 1;
      P.col(i) = cfgVs_[i].rightCols(k).rowwise().sum() / (1.0 + k) + cfgVs_[i].col(0);
    }
  } else {
    // Fallback: linear interpolation
    for (int i = 0; i < N_ - 1; ++i) {
      double ratio = (double)(i + 1) / N_;
      P.col(i) = (1.0 - ratio) * initS.col(0) + ratio * finalS.col(0);
    }
  }

  // Flatten P directly to p_ (no backwardP)
  for (int i = 0; i < N_ - 1; ++i) {
    p_.segment<3>(3 * i) = P.col(i);
  }

  jerkOpt_.reset(initS, finalS, N_);
  return;
}

int TrajOpt::optimize(const double& delta) {
  t_now_ = ros::Time::now().toSec();
  debug_print_once_ = true;  // print cost breakdown on first L-BFGS iteration

  // Setup for L-BFGS solver
  lbfgs::lbfgs_parameter_t lbfgs_params;
  lbfgs::lbfgs_load_default_parameters(&lbfgs_params);
  lbfgs_params.mem_size = 16;
  lbfgs_params.past = 3;
  lbfgs_params.g_epsilon = 0.1;        // 对齐 Swarm-Formation：宽松收敛
  lbfgs_params.min_step = 1e-32;
  lbfgs_params.delta = delta;
  lbfgs_params.max_iterations = 60;   // 提高迭代上限，增强六机编队收敛能力
  Eigen::Map<Eigen::VectorXd> t(x_, dim_t_);
  Eigen::Map<Eigen::VectorXd> p(x_ + dim_t_, dim_p_);
  t = t_;
  p = p_;//将优化变量t和p的初始值赋给x_，准备进行优化
  double minObjective;
  auto ret = lbfgs::lbfgs_optimize(dim_t_ + dim_p_ + 1, x_, &minObjective,
                                   &objectiveFunc, nullptr,
                                   &earlyExit, this, &lbfgs_params);
  std::cout << "\033[32m"
            << "ret: " << ret << "\033[0m" << std::endl;
  t_ = t;
  p_ = p;

  // Print final cost breakdown after optimization (always, regardless of ret)
  {
    Eigen::VectorXd T(N_);
    Eigen::MatrixXd P(3, N_ - 1);
    double sumT = sum_T_ + x_[dim_p_ + dim_t_] * x_[dim_p_ + dim_t_];
    forwardT(t_, sumT, T);
    // Soft constraint: P directly from p_
    for (int i = 0; i < N_ - 1; ++i) {
      P.col(i) = p_.segment<3>(3 * i);
    }
    jerkOpt_.generate(P, T);
    double final_cost = jerkOpt_.getTrajJerkCost();
    jerkOpt_.calGrads_CT();
    debug_cost_corridor_ = 0;
    debug_cost_vel_ = 0;
    debug_cost_acc_ = 0;
    debug_cost_collision_ = 0;
    debug_cost_formation_ = 0;
    debug_cost_tracking_ = 0;
    debug_cost_vis_ = 0;
    addTimeIntPenalty(final_cost);
    addTimeCost(final_cost);
    printf("\033[33m[drone %d FINAL ret=%d] corridor=%.1f  vel=%.1f  acc=%.1f  "
           "collision=%.1f  formation=%.1f  tracking=%.1f  vis=%.1f  total=%.1f\033[0m\n",
           drone_id_, ret,
           debug_cost_corridor_, debug_cost_vel_, debug_cost_acc_,
           debug_cost_collision_, debug_cost_formation_,
           debug_cost_tracking_, debug_cost_vis_, final_cost);
  }
  return ret;
}

// === OLD generate_traj with visibility (hard constraint) ===
// bool TrajOpt::generate_traj(const Eigen::MatrixXd& iniState,
//                             const Eigen::MatrixXd& finState,
//                             const std::vector<Eigen::Vector3d>& target_predcit,
//                             const std::vector<Eigen::Vector3d>& visible_ps,
//                             const std::vector<double>& thetas,
//                             const std::vector<Eigen::MatrixXd>& hPolys,
//                             Trajectory& traj) {
//   landing_ = false;
//   cfgHs_ = hPolys;
//   if (cfgHs_.size() == 1) {
//     cfgHs_.push_back(cfgHs_[0]);
//   }
//   if (!extractVs(cfgHs_, cfgVs_)) {
//     ROS_ERROR("extractVs fail!");
//     return false;
//   }
//   N_ = 2 * cfgHs_.size();
//   sum_T_ = tracking_dur_;
//   dim_t_ = N_ - 1;
//   dim_p_ = 0;
//   for (const auto& cfgV : cfgVs_) {
//     dim_p_ += cfgV.cols() - 1;
//   }
//   p_.resize(dim_p_);
//   t_.resize(dim_t_);
//   x_ = new double[dim_p_ + dim_t_ + 1];
//   Eigen::VectorXd T(N_);
//   Eigen::MatrixXd P(3, N_ - 1);
//   tracking_ps_ = target_predcit;
//   tracking_visible_ps_ = visible_ps;
//   tracking_thetas_ = thetas;
//   setBoundConds(iniState, finState);
//   x_[dim_p_ + dim_t_] = 0.1;
//   int opt_ret = optimize();
//   if (opt_ret < 0) {
//     return false;
//   }
//   double sumT = sum_T_ + x_[dim_p_ + dim_t_] * x_[dim_p_ + dim_t_];
//   forwardT(t_, sumT, T);
//   forwardP(p_, cfgVs_, P);
//   jerkOpt_.generate(P, T);
//   traj = jerkOpt_.getTraj();
//   delete[] x_;
//   return true;
// }

// === NEW generate_traj with visibility (soft constraint) ===
bool TrajOpt::generate_traj(const Eigen::MatrixXd& iniState,
                            const Eigen::MatrixXd& finState,
                            const std::vector<Eigen::Vector3d>& target_predcit,
                            const std::vector<Eigen::Vector3d>& visible_ps,
                            const std::vector<double>& thetas,
                            const std::vector<Eigen::MatrixXd>& hPolys,
                            Trajectory& traj) {
  landing_ = false;
  cfgHs_ = hPolys;
  if (cfgHs_.size() == 1) {
    cfgHs_.push_back(cfgHs_[0]);
  }
  // 软约束：N_ 由路径弧长决定，不依赖走廊数量
  {
    constexpr double seg_len = 2.0;
    constexpr int N_min = 4;
    constexpr int N_max = 20;
    double dist = (finState.col(0) - iniState.col(0)).norm();
    N_ = std::max(N_min, std::min(N_max, (int)std::ceil(dist / seg_len)));
  }
  sum_T_ = tracking_dur_;

  ROS_INFO("[drone %d traj_opt] corridors=%zu, N=%d, sum_T=%.2f, time_per_piece=%.3f",
           drone_id_, cfgHs_.size(), N_, sum_T_, sum_T_ / N_);

  dim_t_ = N_ - 1;
  dim_p_ = 3 * (N_ - 1);  // P directly as optimization variables
  p_.resize(dim_p_);
  t_.resize(dim_t_);
  ensureWorkspace(static_cast<size_t>(dim_p_ + dim_t_ + 1));

  tracking_ps_ = target_predcit;
  tracking_visible_ps_ = visible_ps;
  tracking_thetas_ = thetas;

  setBoundConds(iniState, finState);
  x_[dim_p_ + dim_t_] = 0.1;
  int opt_ret = optimize();
  // 对齐 Swarm-Formation：不因 line search 失败丢弃结果，交给后续碰撞检查
  Eigen::VectorXd T(N_);
  Eigen::MatrixXd P(3, N_ - 1);
  double sumT = sum_T_ + x_[dim_p_ + dim_t_] * x_[dim_p_ + dim_t_];
  forwardT(t_, sumT, T);
  for (int i = 0; i < N_ - 1; ++i) {
    P.col(i) = p_.segment<3>(3 * i);
  }
  jerkOpt_.generate(P, T);
  traj = jerkOpt_.getTraj();
  return true;
}

// === OLD generate_traj without visibility (hard constraint) ===
// bool TrajOpt::generate_traj(const Eigen::MatrixXd& iniState,
//                             const Eigen::MatrixXd& finState,
//                             const std::vector<Eigen::Vector3d>& target_predcit,
//                             const std::vector<Eigen::MatrixXd>& hPolys,
//                             Trajectory& traj) {
//   landing_ = false;
//   cfgHs_ = hPolys;
//   if (cfgHs_.size() == 1) {
//     cfgHs_.push_back(cfgHs_[0]);
//   }
//   if (!extractVs(cfgHs_, cfgVs_)) {
//     ROS_ERROR("extractVs fail!");
//     return false;
//   }
//   N_ = 2 * cfgHs_.size();
//   sum_T_ = tracking_dur_;
//   dim_t_ = N_ - 1;
//   dim_p_ = 0;
//   for (const auto& cfgV : cfgVs_) {
//     dim_p_ += cfgV.cols() - 1;
//   }
//   p_.resize(dim_p_);
//   t_.resize(dim_t_);
//   x_ = new double[dim_p_ + dim_t_ + 1];
//   Eigen::VectorXd T(N_);
//   Eigen::MatrixXd P(3, N_ - 1);
//   tracking_ps_ = target_predcit;
//   setBoundConds(iniState, finState);
//   x_[dim_p_ + dim_t_] = 0.1;
//   int opt_ret = optimize();
//   if (opt_ret == -1005) {
//     ROS_WARN("[drone %d traj_opt] ret=-1005, retry without formation", drone_id_);
//     bool saved_use_formation = use_formation_;
//     use_formation_ = false;
//     setBoundConds(iniState, finState);
//     x_[dim_p_ + dim_t_] = 0.1;
//     opt_ret = optimize();
//     use_formation_ = saved_use_formation;
//   }
//   if (opt_ret < 0) {
//     return false;
//   }
//   double sumT = sum_T_ + x_[dim_p_ + dim_t_] * x_[dim_p_ + dim_t_];
//   forwardT(t_, sumT, T);
//   forwardP(p_, cfgVs_, P);
//   jerkOpt_.generate(P, T);
//   traj = jerkOpt_.getTraj();
//   delete[] x_;
//   return true;
// }

// === NEW generate_traj without visibility (soft constraint) ===
bool TrajOpt::generate_traj(const Eigen::MatrixXd& iniState,
                            const Eigen::MatrixXd& finState,
                            const std::vector<Eigen::Vector3d>& target_predcit,
                            const std::vector<Eigen::MatrixXd>& hPolys,
                            const std::vector<Eigen::Vector3d>& path,
                            Trajectory& traj) {
  landing_ = false;
  cfgHs_ = hPolys;
  if (cfgHs_.size() == 1) {
    cfgHs_.push_back(cfgHs_[0]);
  }
  // 软约束：N_ 由路径弧长决定，不依赖走廊数量
  {
    constexpr double seg_len = 2.0;  // 每段目标长度(m)
    constexpr int N_min = 4;
    constexpr int N_max = 20;
    if (path.size() >= 2) {
      double total_len = 0.0;
      for (size_t j = 1; j < path.size(); ++j)
        total_len += (path[j] - path[j-1]).norm();
      N_ = std::max(N_min, std::min(N_max, (int)std::ceil(total_len / seg_len)));
    } else {
      double dist = (finState.col(0) - iniState.col(0)).norm();
      N_ = std::max(N_min, std::min(N_max, (int)std::ceil(dist / seg_len)));
    }
  }
  sum_T_ = tracking_dur_;

  ROS_INFO("[drone %d traj_opt] corridors=%zu, N=%d, sum_T=%.2f, time_per_piece=%.3f",
           drone_id_, cfgHs_.size(), N_, sum_T_, sum_T_ / N_);

  dim_t_ = N_ - 1;
  dim_p_ = 3 * (N_ - 1);  // P directly as optimization variables
  p_.resize(dim_p_);
  t_.resize(dim_t_);
  ensureWorkspace(static_cast<size_t>(dim_p_ + dim_t_ + 1));

  tracking_ps_ = target_predcit;

  setBoundConds(iniState, finState, path);
  x_[dim_p_ + dim_t_] = 0.1;
  int opt_ret = optimize();
  // 对齐 Swarm-Formation：不因 line search 失败丢弃结果，交给后续碰撞检查
  Eigen::VectorXd T(N_);
  Eigen::MatrixXd P(3, N_ - 1);
  double sumT = sum_T_ + x_[dim_p_ + dim_t_] * x_[dim_p_ + dim_t_];
  forwardT(t_, sumT, T);
  for (int i = 0; i < N_ - 1; ++i) {
    P.col(i) = p_.segment<3>(3 * i);
  }
  jerkOpt_.generate(P, T);
  traj = jerkOpt_.getTraj();
  return true;
}

// === Hard constraint functions ===

static inline double objectiveFuncHard(void* ptrObj,
                                       const double* x,
                                       double* grad,
                                       const int n) {
  TrajOpt& obj = *(TrajOpt*)ptrObj;

  Eigen::Map<const Eigen::VectorXd> t(x, obj.dim_t_);
  Eigen::Map<const Eigen::VectorXd> p(x + obj.dim_t_, obj.dim_p_);
  Eigen::Map<Eigen::VectorXd> gradt(grad, obj.dim_t_);
  Eigen::Map<Eigen::VectorXd> gradp(grad + obj.dim_t_, obj.dim_p_);
  double deltaT = x[obj.dim_t_ + obj.dim_p_];

  Eigen::VectorXd T(obj.N_);
  Eigen::MatrixXd P(3, obj.N_ - 1);
  double sumT = obj.sum_T_ + deltaT * deltaT;
  forwardT(t, sumT, T);
  forwardP(p, obj.cfgVs_, P);

  obj.jerkOpt_.generate(P, T);
  double cost = obj.jerkOpt_.getTrajJerkCost();
  obj.jerkOpt_.calGrads_CT();

  obj.debug_cost_corridor_  = 0;
  obj.debug_cost_vel_       = 0;
  obj.debug_cost_acc_       = 0;
  obj.debug_cost_collision_ = 0;
  obj.debug_cost_formation_ = 0;
  obj.debug_cost_tracking_  = 0;
  obj.debug_cost_vis_       = 0;

  obj.addTimeIntPenalty(cost);
  obj.addTimeCost(cost);
  obj.jerkOpt_.calGrads_PT();
  grad[obj.dim_t_ + obj.dim_p_] = obj.jerkOpt_.gdT.dot(T) / sumT + obj.rhoT_;
  cost += obj.rhoT_ * deltaT * deltaT;
  grad[obj.dim_t_ + obj.dim_p_] *= 2 * deltaT;
  addLayerTGrad(t, sumT, obj.jerkOpt_.gdT, gradt);
  addLayerPGrad(p, obj.cfgVs_, obj.jerkOpt_.gdP, gradp);

  if (obj.debug_print_once_) {
    obj.debug_print_once_ = false;
    printf("\033[36m[drone %d cost] corridor=%.1f  vel=%.1f  acc=%.1f  "
           "collision=%.1f  formation=%.1f  tracking=%.1f  vis=%.1f  total=%.1f\033[0m\n",
           obj.drone_id_,
           obj.debug_cost_corridor_, obj.debug_cost_vel_, obj.debug_cost_acc_,
           obj.debug_cost_collision_, obj.debug_cost_formation_,
           obj.debug_cost_tracking_, obj.debug_cost_vis_, cost);
  }

  return cost;
}

void TrajOpt::setBoundCondsHard(const Eigen::MatrixXd& iniState,
                                const Eigen::MatrixXd& finState) {
  Eigen::MatrixXd initS = iniState;
  Eigen::MatrixXd finalS = finState;
  double tempNorm = initS.col(1).norm();
  initS.col(1) *= tempNorm > vmax_ ? (vmax_ / tempNorm) : 1.0;
  tempNorm = finalS.col(1).norm();
  finalS.col(1) *= tempNorm > vmax_ ? (vmax_ / tempNorm) : 1.0;
  tempNorm = initS.col(2).norm();
  initS.col(2) *= tempNorm > amax_ ? (amax_ / tempNorm) : 1.0;
  tempNorm = finalS.col(2).norm();
  finalS.col(2) *= tempNorm > amax_ ? (amax_ / tempNorm) : 1.0;

  Eigen::VectorXd T(N_);
  T.setConstant(sum_T_ / N_);
  backwardT(T, t_);
  Eigen::MatrixXd P(3, N_ - 1);
  for (int i = 0; i < N_ - 1; ++i) {
    int k = cfgVs_[i].cols() - 1;
    P.col(i) = cfgVs_[i].rightCols(k).rowwise().sum() / (1.0 + k) + cfgVs_[i].col(0);
  }
  backwardP(P, cfgVs_, p_);
  jerkOpt_.reset(initS, finalS, N_);
  return;
}

int TrajOpt::optimizeHard(const double& delta) {
  t_now_ = ros::Time::now().toSec();
  debug_print_once_ = true;

  lbfgs::lbfgs_parameter_t lbfgs_params;
  lbfgs::lbfgs_load_default_parameters(&lbfgs_params);
  lbfgs_params.mem_size = 16;
  lbfgs_params.past = 3;
  lbfgs_params.g_epsilon = 0.1;
  lbfgs_params.min_step = 1e-32;
  lbfgs_params.delta = delta;
  lbfgs_params.max_iterations = 60;
  Eigen::Map<Eigen::VectorXd> t(x_, dim_t_);
  Eigen::Map<Eigen::VectorXd> p(x_ + dim_t_, dim_p_);
  t = t_;
  p = p_;
  double minObjective;
  auto ret = lbfgs::lbfgs_optimize(dim_t_ + dim_p_ + 1, x_, &minObjective,
                                   &objectiveFuncHard, nullptr,
                                   &earlyExit, this, &lbfgs_params);
  std::cout << "\033[32m"
            << "ret: " << ret << "\033[0m" << std::endl;
  t_ = t;
  p_ = p;

  // Print final cost breakdown after optimization
  {
    Eigen::VectorXd T(N_);
    Eigen::MatrixXd P(3, N_ - 1);
    double sumT = sum_T_ + x_[dim_p_ + dim_t_] * x_[dim_p_ + dim_t_];
    forwardT(t_, sumT, T);
    forwardP(p_, cfgVs_, P);
    jerkOpt_.generate(P, T);
    double final_cost = jerkOpt_.getTrajJerkCost();
    jerkOpt_.calGrads_CT();
    debug_cost_corridor_ = 0;
    debug_cost_vel_ = 0;
    debug_cost_acc_ = 0;
    debug_cost_collision_ = 0;
    debug_cost_formation_ = 0;
    debug_cost_tracking_ = 0;
    debug_cost_vis_ = 0;
    addTimeIntPenalty(final_cost);
    addTimeCost(final_cost);
    printf("\033[33m[drone %d FINAL ret=%d] corridor=%.1f  vel=%.1f  acc=%.1f  "
           "collision=%.1f  formation=%.1f  tracking=%.1f  vis=%.1f  total=%.1f\033[0m\n",
           drone_id_, ret,
           debug_cost_corridor_, debug_cost_vel_, debug_cost_acc_,
           debug_cost_collision_, debug_cost_formation_,
           debug_cost_tracking_, debug_cost_vis_, final_cost);
  }
  return ret;
}

bool TrajOpt::generate_traj_hard(const Eigen::MatrixXd& iniState,
                                 const Eigen::MatrixXd& finState,
                                 const std::vector<Eigen::Vector3d>& target_predcit,
                                 const std::vector<Eigen::MatrixXd>& hPolys,
                                 Trajectory& traj) {
  landing_ = false;
  cfgHs_ = hPolys;
  if (cfgHs_.size() == 1) {
    cfgHs_.push_back(cfgHs_[0]);
  }
  if (!extractVs(cfgHs_, cfgVs_)) {
    ROS_ERROR("extractVs fail!");
    return false;
  }
  N_ = 2 * cfgHs_.size();
  sum_T_ = tracking_dur_;
  dim_t_ = N_ - 1;
  dim_p_ = 0;
  for (const auto& cfgV : cfgVs_) {
    dim_p_ += cfgV.cols() - 1;
  }
  p_.resize(dim_p_);
  t_.resize(dim_t_);
  ensureWorkspace(static_cast<size_t>(dim_p_ + dim_t_ + 1));
  Eigen::VectorXd T(N_);
  Eigen::MatrixXd P(3, N_ - 1);
  tracking_ps_ = target_predcit;
  setBoundCondsHard(iniState, finState);
  x_[dim_p_ + dim_t_] = 0.1;
  int opt_ret = optimizeHard();
  // 对齐软约束：不因 ret<0 丢弃结果，交给后续碰撞检查
  double sumT = sum_T_ + x_[dim_p_ + dim_t_] * x_[dim_p_ + dim_t_];
  forwardT(t_, sumT, T);
  forwardP(p_, cfgVs_, P);
  jerkOpt_.generate(P, T);
  traj = jerkOpt_.getTraj();
  return true;
}

bool TrajOpt::generate_traj_hard(const Eigen::MatrixXd& iniState,
                                 const Eigen::MatrixXd& finState,
                                 const std::vector<Eigen::MatrixXd>& hPolys,
                                 Trajectory& traj) {
  landing_ = false;
  cfgHs_ = hPolys;
  if (cfgHs_.size() == 1) {
    cfgHs_.push_back(cfgHs_[0]);
  }
  if (!extractVs(cfgHs_, cfgVs_)) {
    ROS_ERROR("extractVs fail!");
    return false;
  }
  N_ = 2 * cfgHs_.size();
  sum_T_ = tracking_dur_;
  dim_t_ = N_ - 1;
  dim_p_ = 0;
  for (const auto& cfgV : cfgVs_) {
    dim_p_ += cfgV.cols() - 1;
  }
  p_.resize(dim_p_);
  t_.resize(dim_t_);
  ensureWorkspace(static_cast<size_t>(dim_p_ + dim_t_ + 1));
  Eigen::VectorXd T(N_);
  Eigen::MatrixXd P(3, N_ - 1);
  tracking_ps_.clear();
  setBoundCondsHard(iniState, finState);
  x_[dim_p_ + dim_t_] = 0.1;
  int opt_ret = optimizeHard();
  // 对齐软约束：不因 ret<0 丢弃结果，交给后续碰撞检查
  double sumT = sum_T_ + x_[dim_p_ + dim_t_] * x_[dim_p_ + dim_t_];
  forwardT(t_, sumT, T);
  forwardP(p_, cfgVs_, P);
  jerkOpt_.generate(P, T);
  traj = jerkOpt_.getTraj();
  return true;
}

void TrajOpt::addTimeIntPenalty(double& cost) {//位置走廊约束、速度走廊约束、加速度约束代价，并加到总成本中；同时计算这些约束的梯度，并加到总梯度中
  Eigen::Vector3d pos, vel, acc, jer;
  Eigen::Vector3d grad_tmp;
  double cost_tmp;
  Eigen::Matrix<double, 6, 1> beta0, beta1, beta2, beta3;
  double s1, s2, s3, s4, s5;
  double step, alpha;
  Eigen::Matrix<double, 6, 3> gradViolaPc, gradViolaVc, gradViolaAc;
  double gradViolaPt, gradViolaVt, gradViolaAt;
  double omg;

  // t_acc: accumulated absolute trajectory time at the start of piece i
  double t_acc = 0.0;

  int innerLoop;
  for (int i = 0; i < N_; ++i) {//i为每段轨迹的索引
    const auto& c = jerkOpt_.b.block<6, 3>(i * 6, 0);//取多项式系数矩阵的第i段轨迹的系数矩阵
    step = jerkOpt_.T1(i) / K_;//每段轨迹内的采样步长
    s1 = 0.0;//s1为采样点在当前端轨迹的时间
    innerLoop = K_ + 1;

    for (int j = 0; j < innerLoop; ++j) {//j为每段轨迹内的采样点索引
      s2 = s1 * s1;
      s3 = s2 * s1;
      s4 = s2 * s2;
      s5 = s4 * s1;
      beta0 << 1.0, s1, s2, s3, s4, s5;//多项式轨迹就是a0+a1*s1+a2*s1^2+...，beta0就是每个系数前的s1的幂次项
      beta1 << 0.0, 1.0, 2.0 * s1, 3.0 * s2, 4.0 * s3, 5.0 * s4;//多项式轨迹求导的速度表达式，beta1就是每个系数前的s1的幂次项
      beta2 << 0.0, 0.0, 2.0, 6.0 * s1, 12.0 * s2, 20.0 * s3;//...
      beta3 << 0.0, 0.0, 0.0, 6.0, 24.0 * s1, 60.0 * s2;//...
      alpha = 1.0 / K_ * j;//alpha为当前采样点在当前段轨迹内的归一化时间，范围[0,1]
      pos = c.transpose() * beta0;
      vel = c.transpose() * beta1;
      acc = c.transpose() * beta2;
      jer = c.transpose() * beta3;//计算出采样点的pos到jerk

      omg = (j == 0 || j == innerLoop - 1) ? 0.5 : 1.0;

      // Accumulated trajectory time at this sample point
      double t_sample = t_acc + s1;

      // === corridor / obstacle penalty (switched by use_soft_constraint_) ===
      if (use_soft_constraint_) {
        // 软约束：ESDF梯度，不涉及走廊
        if (grad_cost_obstacle(pos, grad_tmp, cost_tmp)) {
          gradViolaPc = beta0 * grad_tmp.transpose();
          gradViolaPt = alpha * grad_tmp.transpose() * vel;
          jerkOpt_.gdC.block<6, 3>(i * 6, 0) += omg * step * gradViolaPc;
          jerkOpt_.gdT(i) += omg * (cost_tmp / K_ + step * gradViolaPt);
          cost += omg * step * cost_tmp;
          debug_cost_corridor_ += omg * step * cost_tmp;
        }
      } else {
        // 硬约束：走廊半平面惩罚
        const auto& hPoly = cfgHs_[i / 2];
        if (grad_cost_p_corridor(pos, hPoly, grad_tmp, cost_tmp)) {
          gradViolaPc = beta0 * grad_tmp.transpose();
          gradViolaPt = alpha * grad_tmp.transpose() * vel;
          jerkOpt_.gdC.block<6, 3>(i * 6, 0) += omg * step * gradViolaPc;
          jerkOpt_.gdT(i) += omg * (cost_tmp / K_ + step * gradViolaPt);
          cost += omg * step * cost_tmp;
          debug_cost_corridor_ += omg * step * cost_tmp;
        }
      }
      if (grad_cost_v(vel, grad_tmp, cost_tmp)) {
        gradViolaVc = beta1 * grad_tmp.transpose();
        gradViolaVt = alpha * grad_tmp.dot(acc);
        jerkOpt_.gdC.block<6, 3>(i * 6, 0) += omg * step * gradViolaVc;
        jerkOpt_.gdT(i) += omg * (cost_tmp / K_ + step * gradViolaVt);
        cost += omg * step * cost_tmp;
        debug_cost_vel_ += omg * step * cost_tmp;
      }
      if (grad_cost_a(acc, grad_tmp, cost_tmp)) {
        gradViolaAc = beta2 * grad_tmp.transpose();
        gradViolaAt = alpha * grad_tmp.dot(jer);
        jerkOpt_.gdC.block<6, 3>(i * 6, 0) += omg * step * gradViolaAc;
        jerkOpt_.gdT(i) += omg * (cost_tmp / K_ + step * gradViolaAt);
        cost += omg * step * cost_tmp;
        debug_cost_acc_ += omg * step * cost_tmp;
      }

      // ---- Swarm collision avoidance ----
      {
        double gradt_swarm = 0, grad_prev_t_swarm = 0, costp_swarm = 0;
        if (grad_cost_swarm_collision(i, t_sample, pos, vel,
                                      grad_tmp, gradt_swarm, grad_prev_t_swarm, costp_swarm)) {
          gradViolaPc = beta0 * grad_tmp.transpose();
          gradViolaPt = alpha * gradt_swarm;
          jerkOpt_.gdC.block<6, 3>(i * 6, 0) += omg * step * gradViolaPc;
          jerkOpt_.gdT(i) += omg * (costp_swarm / K_ + step * gradViolaPt);
          if (i > 0) {
            jerkOpt_.gdT.head(i).array() += omg * step * grad_prev_t_swarm;
          }
          cost += omg * step * costp_swarm;
          debug_cost_collision_ += omg * step * costp_swarm;
        }
      }

      // ---- Formation cost ----
      // 只在轨迹前2/3部分计算编队代价（对齐swarm-formation，避免末端约束冲突）
      if (use_formation_ && !emergency_recovery_ && !suppress_formation_cost_ &&
          t_sample > 0 && t_sample < sum_T_ * 2.0 / 3.0) {
        double gradt_form = 0, grad_prev_t_form = 0, costp_form = 0;
        if (grad_cost_swarm_formation(i, t_sample, pos, vel,
                                       grad_tmp, gradt_form, grad_prev_t_form, costp_form)) {
          gradViolaPc = beta0 * grad_tmp.transpose();
          gradViolaPt = alpha * gradt_form;
          jerkOpt_.gdC.block<6, 3>(i * 6, 0) += omg * step * gradViolaPc;
          jerkOpt_.gdT(i) += omg * (costp_form / K_ + step * gradViolaPt);
          if (i > 0) {
            jerkOpt_.gdT.head(i).array() += omg * step * grad_prev_t_form;
          }
          cost += omg * step * costp_form;
          debug_cost_formation_ += omg * step * costp_form;
        }
      }

      s1 += step;
    }
    t_acc += jerkOpt_.T1(i);
  }
}

bool TrajOpt::grad_cost_swarm_collision(const int piece,
                                        const double t,
                                        const Eigen::Vector3d& p,
                                        const Eigen::Vector3d& v,
                                        Eigen::Vector3d& gradp,
                                        double& gradt,
                                        double& grad_prev_t,
                                        double& costp) {
  if (swarm_trajs_.empty()) return false;

  bool ret = false;
  gradp.setZero();
  gradt = 0;
  grad_prev_t = 0;
  costp = 0;

  // Ellipsoid semi-axes: keep small enough to not conflict with 2m triangle formation
  // horizontal clearance 0.8m, vertical clearance 0.5m
  constexpr double a = 0.8, b = 0.5;
  constexpr double inv_a2 = 1.0 / (a * a), inv_b2 = 1.0 / (b * b);
  const double CLEARANCE2 = (rhoSwarm_ > 0) ? 1.0 : 1.0; // normalized: penalty starts when ellip_dist2 < 1
  // NOTE: rhoSwarm_ is the weight; the clearance ellipsoid has semi-axes a,b (unit ellipsoid = 1).

  double pt_time = t_now_ + t;  // absolute time of this sample point

  for (size_t id = 0; id < swarm_trajs_.size(); ++id) {
    if (swarm_trajs_[id].drone_id < 0 || swarm_trajs_[id].drone_id == drone_id_) continue;

    double traj_start = swarm_trajs_[id].start_time;
    Eigen::Vector3d swarm_p, swarm_v;

    double t_other = pt_time - traj_start;
    if (t_other >= 0 && t_other < swarm_trajs_[id].duration) {
      swarm_p = swarm_trajs_[id].traj.getPos(t_other);
      swarm_v = swarm_trajs_[id].traj.getVel(t_other);
    } else if (t_other >= swarm_trajs_[id].duration) {
      // Extrapolate with constant velocity after trajectory ends
      double exceed = t_other - swarm_trajs_[id].duration;
      swarm_v = swarm_trajs_[id].traj.getVel(swarm_trajs_[id].duration);
      swarm_p = swarm_trajs_[id].traj.getPos(swarm_trajs_[id].duration) + exceed * swarm_v;
    } else {
      continue;  // t_other < 0: other drone's traj hasn't started yet
    }

    Eigen::Vector3d dp = p - swarm_p;
    double ellip_dist2 = dp(0) * dp(0) * inv_b2 + dp(1) * dp(1) * inv_b2 + dp(2) * dp(2) * inv_a2;
    double dist2_err = CLEARANCE2 - ellip_dist2;
    double dist2_err2 = dist2_err * dist2_err;
    double dist2_err3 = dist2_err2 * dist2_err;

    if (dist2_err3 > 0) {
      ret = true;
      costp += rhoSwarm_ * dist2_err3;

      // dJ/dp  (chain rule: dJ/d(ellip_dist2) * d(ellip_dist2)/dp)
      Eigen::Vector3d dJ_dp = rhoSwarm_ * 3.0 * dist2_err2 * (-2.0) *
                               Eigen::Vector3d(inv_b2 * dp(0), inv_b2 * dp(1), inv_a2 * dp(2));
      gradp += dJ_dp;

      // dJ/dT_i  = dJ/dp · (dp/dt)  = dJ/dp · (v - swarm_v)
      gradt += dJ_dp.dot(v - swarm_v);

      // dJ/dT_{0..i-1}  = dJ/dp · (-swarm_v)  (other drone's pos depends on prev T segments)
      grad_prev_t += dJ_dp.dot(-swarm_v);
    }
  }
  return ret;
}

bool TrajOpt::grad_cost_swarm_formation(const int piece,
                                        const double t,
                                        const Eigen::Vector3d& p,
                                        const Eigen::Vector3d& v,
                                        Eigen::Vector3d& gradp,
                                        double& gradt,
                                        double& grad_prev_t,
                                        double& costp) {
  if (!use_formation_ || swarm_graph_ == nullptr) return false;
  if (drone_id_ < 0 || drone_id_ >= formation_size_) return false;

  // 严格对齐 Swarm-Formation 的冷启动逻辑：
  // 只有当所有其他无人机的轨迹都就绪时才计算编队代价；
  // 唯一例外是最后一架无人机（drone_id_ == formation_size_ - 1），
  // 它强制认为所有轨迹都就绪，保证编队代价从冷启动开始就至少有一架无人机在执行。
  int ready_count = 0;
  for (int id = 0; id < formation_size_; ++id) {
    if (id == drone_id_) continue;
    if ((int)swarm_trajs_.size() > id && swarm_trajs_[id].drone_id >= 0)
      ready_count++;
  }
  bool is_last_drone = (drone_id_ == formation_size_ - 1);
  if (!is_last_drone && ready_count < formation_size_ - 1)
    return false;  // 轨迹未全部就绪，跳过（最后一架无人机除外）

  bool ret = false;
  gradp.setZero();
  gradt = 0;
  grad_prev_t = 0;
  costp = 0;

  double pt_time = t_now_ + t;

  std::vector<Eigen::Vector3d> swarm_pos(formation_size_, Eigen::Vector3d::Zero());
  std::vector<Eigen::Vector3d> swarm_vel(formation_size_, Eigen::Vector3d::Zero());

  swarm_pos[drone_id_] = p;
  swarm_vel[drone_id_] = v;

  for (int id = 0; id < formation_size_; ++id) {
    if (id == drone_id_) continue;

    bool traj_ready = ((int)swarm_trajs_.size() > id && swarm_trajs_[id].drone_id >= 0);

    if (traj_ready) {
      double traj_start = swarm_trajs_[id].start_time;
      double t_other = pt_time - traj_start;
      if (t_other >= 0 && t_other < swarm_trajs_[id].duration) {
        swarm_pos[id] = swarm_trajs_[id].traj.getPos(t_other);
        swarm_vel[id] = swarm_trajs_[id].traj.getVel(t_other);
      } else if (t_other >= swarm_trajs_[id].duration) {
        swarm_vel[id] = swarm_trajs_[id].traj.getVel(swarm_trajs_[id].duration);
        swarm_pos[id] = swarm_trajs_[id].traj.getPos(swarm_trajs_[id].duration) +
                        (t_other - swarm_trajs_[id].duration) * swarm_vel[id];
      } else {
        return false;  // t_other < 0，时间对不上，跳过
      }
    } else {
      // 最后一架无人机且轨迹未就绪：用自身当前位置占位，速度设零
      // 这样 SNL 至少能用本机 + 已就绪的无人机位置计算，不会退化
      swarm_pos[id] = p;  // 占位，SNL 会产生排斥力让它们分散
      swarm_vel[id].setZero();
    }
  }

  if (!swarm_graph_->updateGraph(swarm_pos)) {
    ROS_WARN_THROTTLE(1.0, "[drone %d traj_opt] skip formation: updateGraph failed (formation_size=%d)", drone_id_, formation_size_);
    return false;
  }

  double similarity_error;
  if (!swarm_graph_->calcFNorm2(similarity_error)) return false;

  if (similarity_error > 0) {
    ret = true;
    costp = wei_formation_ * similarity_error;

    std::vector<Eigen::Vector3d> swarm_grad;
    if (!swarm_graph_->getGrad(swarm_grad)) return false;
    if ((int)swarm_grad.size() <= drone_id_ || (int)swarm_grad.size() < formation_size_) {
      ROS_WARN_THROTTLE(1.0,
                        "[drone %d traj_opt] skip formation: invalid grad size=%zu, formation_size=%d",
                        drone_id_, swarm_grad.size(), formation_size_);
      return false;
    }

    if (formation_grad_clip_ > 0.0) {
      for (auto& g : swarm_grad) {
        const double g_norm = g.norm();
        if (g_norm > formation_grad_clip_) {
          g *= (formation_grad_clip_ / g_norm);
        }
      }
    }

    gradp = wei_formation_ * swarm_grad[drone_id_];

    // 对齐 swarm-formation 的梯度计算：
    // gradt 累加所有无人机，grad_prev_t 只累加其他无人机
    gradt = 0;
    grad_prev_t = 0;
    for (int id = 0; id < formation_size_; ++id) {
      double contrib = wei_formation_ * swarm_grad[id].dot(swarm_vel[id]);
      gradt += contrib;
      if (id != drone_id_) {
        grad_prev_t += contrib;
      }
    }
  }

  return ret;
}

void TrajOpt::addTimeCost(double& cost) {
  if (emergency_recovery_) return;   // 紧急恢复：跳过追踪代价
  if (!use_tracking_cost_) return;  // front-end target guidance only, no tracking term in optimizer
  const auto& T = jerkOpt_.T1;
  int piece = 0;
  int M = tracking_ps_.size() * 4 / 5;
  double t = 0;
  double t_pre = 0;

  double step = tracking_dt_;
  Eigen::Matrix<double, 6, 1> beta0, beta1;
  double s1, s2, s3, s4, s5;
  Eigen::Vector3d pos, vel;
  Eigen::Vector3d grad_tmp;
  double cost_tmp;
  Eigen::Matrix<double, 6, 3> gradViolaPc;

  for (int i = 0; i < M; ++i) {
    double rho = exp2(-3.0 * i / M);
    while (t - t_pre > T(piece)) {
      t_pre += T(piece);
      piece++;
    }
    s1 = t - t_pre;
    s2 = s1 * s1;
    s3 = s2 * s1;
    s4 = s2 * s2;
    s5 = s4 * s1;
    beta0 << 1.0, s1, s2, s3, s4, s5;
    beta1 << 0.0, 1.0, 2.0 * s1, 3.0 * s2, 4.0 * s3, 5.0 * s4;
    const auto& c = jerkOpt_.b.block<6, 3>(piece * 6, 0);
    pos = c.transpose() * beta0;
    vel = c.transpose() * beta1;
    Eigen::Vector3d target_p = tracking_ps_[i];

    if (landing_) {
      if (grad_cost_p_landing(pos, target_p, grad_tmp, cost_tmp)) {
        gradViolaPc = beta0 * grad_tmp.transpose();
        cost += rho * step * cost_tmp;
        debug_cost_tracking_ += rho * step * cost_tmp;
        jerkOpt_.gdC.block<6, 3>(piece * 6, 0) += rho * step * gradViolaPc;
        if (piece > 0) {
          jerkOpt_.gdT.head(piece).array() += -rho * step * grad_tmp.dot(vel);
        }
      }
    } else {
      if (grad_cost_p_tracking(pos, target_p, grad_tmp, cost_tmp)) {
        gradViolaPc = beta0 * grad_tmp.transpose();
        cost += rho * step * cost_tmp;
        debug_cost_tracking_ += rho * step * cost_tmp;
        jerkOpt_.gdC.block<6, 3>(piece * 6, 0) += rho * step * gradViolaPc;
        if (piece > 0) {
          jerkOpt_.gdT.head(piece).array() += -rho * step * grad_tmp.dot(vel);
        }
      }
      // TODO occlusion
      /* Commented out visibility constraint
      if (grad_cost_visibility(pos, target_p, tracking_visible_ps_[i], tracking_thetas_[i],
                               grad_tmp, cost_tmp)) {
        gradViolaPc = beta0 * grad_tmp.transpose();
        cost += rho * step * cost_tmp;
        debug_cost_vis_ += rho * step * cost_tmp;
        jerkOpt_.gdC.block<6, 3>(piece * 6, 0) += rho * step * gradViolaPc;
        if (piece > 0) {
          jerkOpt_.gdT.head(piece).array() += -rho * step * grad_tmp.dot(vel);
        }
      }*/
    }

    t += step;
  }
}

bool TrajOpt::grad_cost_p_corridor(const Eigen::Vector3d& p,
                                   const Eigen::MatrixXd& hPoly,
                                   Eigen::Vector3d& gradp,
                                   double& costp) {
  // return false;
  bool ret = false;
  gradp.setZero();
  costp = 0;
  for (int i = 0; i < hPoly.cols(); ++i) {
    Eigen::Vector3d norm_vec = hPoly.col(i).head<3>();
    double pen = norm_vec.dot(p - hPoly.col(i).tail<3>() + clearance_d_ * norm_vec);
    if (pen > 0) {
      double pen2 = pen * pen;
      gradp += rhoP_ * 3 * pen2 * norm_vec;
      costp += rhoP_ * pen2 * pen;
      ret = true;
    }
  }
  return ret;
}

// ESDF-based smooth obstacle penalty (replaces 6-direction ray search)
bool TrajOpt::grad_cost_obstacle(const Eigen::Vector3d& p,
                                 Eigen::Vector3d& gradp,
                                 double& costp) {
  if (!mapPtr_ || !mapPtr_->esdfReady()) return false;
  gradp.setZero();
  costp = 0;

  double dist;
  mapPtr_->evaluateEDT(p, dist);
  double dist_err = clearance_d_ - dist;
  if (dist_err > 0) {
    Eigen::Vector3d dist_grad;
    mapPtr_->evaluateFirstGrad(p, dist_grad);
    costp = rhoP_ * dist_err * dist_err * dist_err;
    gradp = -rhoP_ * 3.0 * dist_err * dist_err * dist_grad;
    return true;
  }
  return false;
}
// y2 = 2x0^3 x - x0^4
// static double penF(const double& x, double& grad) {
//   static double x0 = 0.1;
//   static double x02 = x0 * x0;
//   static double x03 = x0 * x02;
//   static double x04 = x02 * x02;
//   if (x < x0) {
//     double x2 = x * x;
//     double x3 = x * x2;
//     grad = x2 * (6 * x0 - 4 * x);
//     return x3 * (2 * x0 - x);
//   } else {
//     grad = 2 * x03;
//     return 2 * x03 * x - x04;
//   }
// }
static double penF(const double& x, double& grad) {
  static double eps = 0.05;
  static double eps2 = eps * eps;
  static double eps3 = eps * eps2;
  if (x < 2 * eps) {
    double x2 = x * x;
    double x3 = x * x2;
    double x4 = x2 * x2;
    grad = 12 / eps2 * x2 - 4 / eps3 * x3;
    return 4 / eps2 * x3 - x4 / eps3;
  } else {
    grad = 16;
    return 16 * (x - eps);
  }
}

static double penF2(const double& x, double& grad) {
  double x2 = x * x;
  grad = 3 * x2;
  return x * x2;
}

bool TrajOpt::grad_cost_p_tracking(const Eigen::Vector3d& p,
                                   const Eigen::Vector3d& target_p,
                                   Eigen::Vector3d& gradp,
                                   double& costp) {
  // Simple point-tracking penalty:
  // only penalize when distance to target point exceeds a threshold.
  // J = max(0, ||p-target|| - d_th)^2
  const double d_th = std::max(1e-3, tolerance_d_);
  const Eigen::Vector3d dp = (p - target_p);
  const double d = dp.norm();

  gradp.setZero();
  costp = 0;
  if (d <= d_th) return false;

  const double err = d - d_th;
  costp = err * err;
  if (d > 1e-6) {
    gradp = 2.0 * err * (dp / d);
  }

  gradp *= rhoTracking_;
  costp *= rhoTracking_;

  return true;
}

bool TrajOpt::grad_cost_p_landing(const Eigen::Vector3d& p,
                                  const Eigen::Vector3d& target_p,
                                  Eigen::Vector3d& gradp,
                                  double& costp) {
  Eigen::Vector3d dp = (p - target_p);
  double dr2 = dp.head(2).squaredNorm();
  double dz2 = dp.z() * dp.z();

  bool ret = false;
  gradp.setZero();
  costp = 0;

  double pen = dr2 - tolerance_d_ * tolerance_d_;
  if (pen > 0) {
    double pen2 = pen * pen;
    gradp.head(2) += 6 * pen2 * dp.head(2);
    costp += pen * pen2;
    ret = true;
  }
  pen = dz2 - tolerance_d_ * tolerance_d_;
  if (pen > 0) {
    double pen2 = pen * pen;
    gradp.z() += 6 * pen2 * dp.z();
    costp += pen * pen2;
    ret = true;
  }

  gradp *= rhoTracking_;
  costp *= rhoTracking_;

  return ret;
}

bool TrajOpt::grad_cost_visibility(const Eigen::Vector3d& p,
                                   const Eigen::Vector3d& center,
                                   const Eigen::Vector3d& vis_p,
                                   const double& theta,
                                   Eigen::Vector3d& gradp,
                                   double& costp) {
  Eigen::Vector3d a = p - center;
  Eigen::Vector3d b = vis_p - center;
  double inner_product = a.dot(b);
  double norm_a = a.norm();
  double norm_b = b.norm();
  double theta_less = theta - theta_clearance_ > 0 ? theta - theta_clearance_ : 0;
  double cosTheta = cos(theta_less);
  double pen = cosTheta - inner_product / norm_a / norm_b;
  if (pen > 0) {
    double grad = 0;
    costp = penF2(pen, grad);
    gradp = grad * -(norm_a * b - inner_product / norm_a * a) / norm_a / norm_a / norm_b;
    // gradp = grad * (norm_b * cosTheta / norm_a * a - b);
    gradp *= rhosVisibility_;
    costp *= rhosVisibility_;
    return true;
  } else {
    return false;
  }
}

bool TrajOpt::grad_cost_v(const Eigen::Vector3d& v,
                          Eigen::Vector3d& gradv,
                          double& costv) {
  double vpen = v.squaredNorm() - vmax_ * vmax_;
  if (vpen > 0) {
    gradv = rhoV_ * 6 * vpen * vpen * v;
    costv = rhoV_ * vpen * vpen * vpen;
    return true;
  }
  return false;
}

bool TrajOpt::grad_cost_a(const Eigen::Vector3d& a,
                          Eigen::Vector3d& grada,
                          double& costa) {
  double apen = a.squaredNorm() - amax_ * amax_;
  if (apen > 0) {
    grada = rhoA_ * 6 * apen * apen * a;
    costa = rhoA_ * apen * apen * apen;
    return true;
  }
  return false;
}

void TrajOpt::setDesiredFormation(int type) {
  std::vector<Eigen::Vector3d> swarm_des;
  Eigen::Vector3d v0, v1, v2, v3, v4, v5;
  switch (type) {
    case 0: // NONE_FORMATION
      use_formation_ = false;
      formation_size_ = 0;
      formation_offset_.setZero();
      break;
    case 1: // REGULAR_HEXAGON_RING (6 drones, no center point)
      // side length = 2m, centered at origin
      v0 <<  1.7321, -1.0000, 0.0;
      v1 <<  0.0000, -2.0000, 0.0;
      v2 << -1.7321, -1.0000, 0.0;
      v3 << -1.7321,  1.0000, 0.0;
      v4 <<  0.0000,  2.0000, 0.0;
      v5 <<  1.7321,  1.0000, 0.0;

      swarm_des.push_back(v0);
      swarm_des.push_back(v1);
      swarm_des.push_back(v2);
      swarm_des.push_back(v3);
      swarm_des.push_back(v4);
      swarm_des.push_back(v5);

      use_formation_ = true;
      formation_size_ = swarm_des.size();
      swarm_graph_->setDesiredForm(swarm_des);
      if (drone_id_ >= 0 && drone_id_ < (int)swarm_des.size()) {
        formation_offset_ = swarm_des[drone_id_];
      } else {
        formation_offset_.setZero();
      }
      break;
    case 2: // EQUILATERAL_TRIANGLE (3 drones, edge length = 2m)
      v0 <<  0.0,    1.1547, 0;
      v1 <<  1.0,   -0.5774, 0;
      v2 << -1.0,   -0.5774, 0;
      swarm_des.push_back(v0);
      swarm_des.push_back(v1);
      swarm_des.push_back(v2);
      formation_size_ = swarm_des.size();
      use_formation_ = true;
      swarm_graph_->setDesiredForm(swarm_des);
      // 保存本机的期望偏移
      if (drone_id_ >= 0 && drone_id_ < (int)swarm_des.size())
        formation_offset_ = swarm_des[drone_id_];
      else
        formation_offset_.setZero();
      break;
    case 3: // SQUARE (4 drones, side length = 2m)
      v0 <<  1.0,  1.0, 0.0;
      v1 <<  1.0, -1.0, 0.0;
      v2 << -1.0, -1.0, 0.0;
      v3 << -1.0,  1.0, 0.0;
      swarm_des.push_back(v0);
      swarm_des.push_back(v1);
      swarm_des.push_back(v2);
      swarm_des.push_back(v3);
      formation_size_ = swarm_des.size();
      use_formation_ = true;
      swarm_graph_->setDesiredForm(swarm_des);
      if (drone_id_ >= 0 && drone_id_ < (int)swarm_des.size())
        formation_offset_ = swarm_des[drone_id_];
      else
        formation_offset_.setZero();
      break;
    case 4: // REGULAR_PENTAGON (5 drones, side length ~= 2m)
      // Centered at origin, one vertex facing +x direction.
      // Radius R = s / (2*sin(pi/5)) ~= 1.7013 for s=2.
      v0 <<  1.7013,  0.0000, 0.0;
      v1 <<  0.5257,  1.6180, 0.0;
      v2 << -1.3764,  1.0000, 0.0;
      v3 << -1.3764, -1.0000, 0.0;
      v4 <<  0.5257, -1.6180, 0.0;
      swarm_des.push_back(v0);
      swarm_des.push_back(v1);
      swarm_des.push_back(v2);
      swarm_des.push_back(v3);
      swarm_des.push_back(v4);
      formation_size_ = swarm_des.size();
      use_formation_ = true;
      swarm_graph_->setDesiredForm(swarm_des);
      if (drone_id_ >= 0 && drone_id_ < (int)swarm_des.size())
        formation_offset_ = swarm_des[drone_id_];
      else
        formation_offset_.setZero();
      break;
    default:
      break;
  }
}

}  // namespace traj_opt
