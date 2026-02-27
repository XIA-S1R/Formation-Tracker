#include <traj_opt/traj_opt.h>

#include <random>
#include <traj_opt/geoutils.hpp>
#include <traj_opt/lbfgs_raw.hpp>

namespace traj_opt {

// SECTION  variables transformation and gradient transmission
static double expC2(double t) {
  return t > 0.0 ? ((0.5 * t + 1.0) * t + 1.0)
                 : 1.0 / ((0.5 * t - 1.0) * t + 1.0);
}
static double logC2(double T) {
  return T > 1.0 ? (sqrt(2.0 * T - 1.0) - 1.0) : (1.0 - sqrt(2.0 / T - 1.0));
}
static void forwardT(const Eigen::Ref<const Eigen::VectorXd>& t, Eigen::Ref<Eigen::VectorXd> vecT) {
  int M = t.size();
  for (int i = 0; i < M; ++i) {
    vecT(i) = expC2(t(i));
  }
  return;
}
static void backwardT(const Eigen::Ref<const Eigen::VectorXd>& vecT, Eigen::Ref<Eigen::VectorXd> t) {
  int M = vecT.size();
  for (int i = 0; i < M; ++i) {
    t(i) = logC2(vecT(i));
  }
  return;
}
static inline double gdT2t(double t) {
  if (t > 0) {
    return t + 1.0;
  } else {
    double denSqrt = (0.5 * t - 1.0) * t + 1.0;
    return (1.0 - t) / (denSqrt * denSqrt);
  }
}
static void addLayerTGrad(const Eigen::Ref<const Eigen::VectorXd>& t,
                          const Eigen::Ref<const Eigen::VectorXd>& gradT,
                          Eigen::Ref<Eigen::VectorXd> gradt) {
  int M = t.size();
  for (int i = 0; i < M; ++i) {
    gradt(i) = gradT(i) * gdT2t(t(i));
  }
  return;
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
static inline double objectiveFunc(void* ptrObj,
                                   const double* x,
                                   double* grad,
                                   const int n) {
  TrajOpt& obj = *(TrajOpt*)ptrObj;

  Eigen::Map<const Eigen::VectorXd> t(x, obj.dim_t_);
  Eigen::Map<const Eigen::VectorXd> p(x + obj.dim_t_, obj.dim_p_);
  Eigen::Map<Eigen::VectorXd> gradt(grad, obj.dim_t_);
  Eigen::Map<Eigen::VectorXd> gradp(grad + obj.dim_t_, obj.dim_p_);

  Eigen::VectorXd T(obj.N_);
  Eigen::MatrixXd P(3, obj.N_ - 1);
  forwardT(t, T);
  forwardP(p, obj.cfgVs_, P);//从x中提取出优化变量t和p，并映射出原先的优化变量T和P

  obj.jerkOpt_.generate(P, T);//用当前P和T生成MINCO轨迹
  double cost = obj.jerkOpt_.getTrajJerkCost();//计算MINCO轨迹的jerk成本
  obj.jerkOpt_.calGrads_CT();//计算jerk代价对C和T的成本
  obj.addTimeIntPenalty(cost);//计算位置走廊约束、速度约束、加速度约束转化成的成本，并将它们加到总成本中；同时计算这些约束的梯度，并加到总梯度中
  obj.jerkOpt_.calGrads_PT();//现在综合jerk代价和时间积分代价，它们对多项式曲线参数(C(P,T),T)的梯度已经计算出来了，接下来把它们转换为对MINCO曲线参数P和T的梯度
  obj.jerkOpt_.gdT.array() += obj.rhoT_;//rhoT_*T.sum()是总时间成本，直接把rhoT_添加入代价对T的梯度。
  cost += obj.rhoT_ * T.sum();//总成本加上总时间成本
  addLayerTGrad(t, obj.jerkOpt_.gdT, gradt);
  addLayerPGrad(p, obj.cfgVs_, obj.jerkOpt_.gdP, gradp);//将代价对MINCO曲线参数(P,T)的梯度转换为代价对优化变量p,t的梯度

  return cost;
}
// !SECTION object function

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

bool TrajOpt::generate_traj(const Eigen::MatrixXd& iniState,
                            const Eigen::MatrixXd& finState,
                            const std::vector<Eigen::MatrixXd>& hPolys,
                            Trajectory& traj) {
  cfgHs_ = hPolys;//cfgHs_存储飞行走廊（每个元素表示每段走廊的多边形数据），cfgVs_存储飞行走廊的顶点
  if (cfgHs_.size() == 1) {
    cfgHs_.push_back(cfgHs_[0]);
  }
  if (!extractVs(cfgHs_, cfgVs_)) {
    ROS_ERROR("extractVs fail!");
    return false;
  }
  N_ = 2 * cfgHs_.size();// 一个走廊设置两个轨迹段

  // NOTE: one corridor two pieces
  dim_t_ = N_;
  dim_p_ = 0;
  for (const auto& cfgV : cfgVs_) {
    dim_p_ += cfgV.cols() - 1;
  }//MINCO轨迹中T的维度为轨迹段数N，P的维度为所有轨迹段的控制点数之和（每段轨迹的控制点数为该段走廊顶点数减1）
  // std::cout << "dim_p_: " << dim_p_ << std::endl;
  p_.resize(dim_p_);
  t_.resize(dim_t_);
  x_ = new double[dim_p_ + dim_t_];
  Eigen::VectorXd T(N_);
  Eigen::MatrixXd P(3, N_ - 1);

  // NOTE set boundary conditions
  Eigen::MatrixXd initS = iniState;
  Eigen::MatrixXd finalS = finState;
  double tempNorm = initS.col(1).norm();
  initS.col(1) *= tempNorm > vmax_ ? (vmax_ / tempNorm) : 1.0;
  tempNorm = finalS.col(1).norm();
  finalS.col(1) *= tempNorm > vmax_ ? (vmax_ / tempNorm) : 1.0;
  tempNorm = initS.col(2).norm();
  initS.col(2) *= tempNorm > amax_ ? (amax_ / tempNorm) : 1.0;
  tempNorm = finalS.col(2).norm();
  finalS.col(2) *= tempNorm > amax_ ? (amax_ / tempNorm) : 1.0;// 对初始状态和最终状态的速度和加速度进行限制，确保它们不超过最大速度vmax_和最大加速度amax_

  T.setConstant((finState.col(0) - iniState.col(0)).norm() / vmax_ / N_);
  backwardT(T, t_);//把正实数域的T微分同胚映射为实数域的t_
  for (int i = 0; i < N_ - 1; ++i) {
    int k = cfgVs_[i].cols() - 1;
    P.col(i) = cfgVs_[i].rightCols(k).rowwise().sum() / (1.0 + k) + cfgVs_[i].col(0);
  }// 将每段走廊的顶点进行平均，得到每段轨迹的初始控制点位置P
  backwardP(P, cfgVs_, p_);//把有避障约束的控制点位置P微分同胚映射为实数域的p_
  jerkOpt_.reset(initS, finalS, N_);

  // NOTE optimization
  lbfgs::lbfgs_parameter_t lbfgs_params;
  lbfgs::lbfgs_load_default_parameters(&lbfgs_params);
  lbfgs_params.mem_size = 16;
  lbfgs_params.past = 3;
  lbfgs_params.g_epsilon = 1e-10;
  lbfgs_params.min_step = 1e-32;
  lbfgs_params.delta = 1e-4;
  Eigen::Map<Eigen::VectorXd> t(x_, dim_t_);
  Eigen::Map<Eigen::VectorXd> p(x_ + dim_t_, dim_p_);
  t = t_;
  p = p_;//将优化变量t和p的初始值赋给x_，准备进行优化
  double minObjective;
  auto opt_ret = lbfgs::lbfgs_optimize(dim_t_ + dim_p_, x_, &minObjective,
                                       &objectiveFunc, nullptr,
                                       &earlyExit, this, &lbfgs_params);
  std::cout << "\033[32m"
            << "ret: " << opt_ret << "\033[0m" << std::endl;
  t_ = t;
  p_ = p;
  if (opt_ret < 0) {
    return false;
  }
  forwardT(t_, T);
  forwardP(p_, cfgVs_, P);//优化完的t_和p_微分同胚映射得到优化后的T和P
  jerkOpt_.generate(P, T);
  traj = jerkOpt_.getTraj();//用T和P生成最终的轨迹traj
  delete[] x_;
  return true;
}

}  // namespace traj_opt
