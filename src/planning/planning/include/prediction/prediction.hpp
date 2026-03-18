#pragma once

#include <mapping/mapping.h>
#include <ros/ros.h>

#include <Eigen/Core>
#include <queue>
#include <memory>

namespace prediction {

struct Node {
  Eigen::Vector3d p, v, a;
  double t;
  double score;
  double h;
  Node* parent = nullptr;
};
typedef Node* NodePtr;
class NodeComparator {
 public:
  bool operator()(NodePtr& lhs, NodePtr& rhs) {
    return lhs->score + lhs->h > rhs->score + rhs->h;
  }
};
struct Predict {
 private:
  static constexpr int MAX_MEMORY = 1 << 22;

  double dt;
  double pre_dur;
  double rho_a;
  double car_z, vmax;
  mapping::OccGridMap map;
  // Single heap allocation — O(1) construction, no 4M individual new/delete.
  std::unique_ptr<Node[]> data_pool_;
  NodePtr data[MAX_MEMORY];
  int stack_top;

  inline bool isValid(const Eigen::Vector3d& p, const Eigen::Vector3d& v) const {
    return (v.norm() < vmax) && (!map.isOccupied(p));
  }

 public:
  inline Predict(ros::NodeHandle& nh) {
    nh.getParam("tracking_dur", pre_dur);
    nh.getParam("tracking_dt", dt);
    nh.getParam("prediction/rho_a", rho_a);
    nh.getParam("prediction/vmax", vmax);
    // One allocation for the whole pool, then point each slot into it.
    data_pool_.reset(new Node[MAX_MEMORY]);
    for (int i = 0; i < MAX_MEMORY; ++i) {
      data[i] = &data_pool_[i];
    }
  }
  inline void setMap(const mapping::OccGridMap& _map) {
    map = _map;
    // map.inflate_last();
  }

  inline bool predict(const Eigen::Vector3d& target_p,
                      const Eigen::Vector3d& target_v,
                      std::vector<Eigen::Vector3d>& target_predcit,
                      const double& max_time = 0.1) {
    // 检查起点是否在障碍物内（容忍估计误差）
    bool start_in_obstacle = map.isOccupied(target_p);
    auto isValidLocal = [&](const Eigen::Vector3d& p, const Eigen::Vector3d& v) -> bool {
      if (start_in_obstacle && (p - target_p).norm() < 0.5) {
        return (v.norm() < vmax);  // 起点附近不检查障碍物
      }
      return (v.norm() < vmax) && (!map.isOccupied(p));
    };

    auto score = [&](const NodePtr& ptr) -> double {
      return rho_a * ptr->a.norm();
    };
    Eigen::Vector3d end_p = target_p + target_v * pre_dur;
    auto calH = [&](const NodePtr& ptr) -> double {
      return 0.001 * (ptr->p - end_p).norm();
    };
    ros::Time t_start = ros::Time::now();
    std::priority_queue<NodePtr, std::vector<NodePtr>, NodeComparator> open_set;

    Eigen::Vector3d input(0, 0, 0);

    stack_top = 0;
    NodePtr curPtr = data[stack_top++];
    curPtr->p = target_p;
    curPtr->v = target_v;
    curPtr->a.setZero();
    curPtr->parent = nullptr;
    curPtr->score = 0;
    curPtr->h = 0;
    curPtr->t = 0;
    double dt2_2 = dt * dt / 2;
    while (curPtr->t < pre_dur) {
      for (input.x() = -3; input.x() <= 3; input.x() += 3)
        for (input.y() = -3; input.y() <= 3; input.y() += 3) {
          Eigen::Vector3d p = curPtr->p + curPtr->v * dt + input * dt2_2;
          Eigen::Vector3d v = curPtr->v + input * dt;
          if (!isValidLocal(p, v)) {
            continue;
          }
          if (stack_top == MAX_MEMORY) {
            std::cout << "[prediction] out of memory!" << std::endl;
            return false;
          }
          double t_cost = (ros::Time::now() - t_start).toSec();
          if (t_cost > max_time) {
            std::cout << "[prediction] too slow!" << std::endl;
            return false;
          }
          NodePtr ptr = data[stack_top++];
          ptr->p = p;
          ptr->v = v;
          ptr->a = input;
          ptr->parent = curPtr;
          ptr->t = curPtr->t + dt;
          ptr->score = curPtr->score + score(ptr);
          ptr->h = calH(ptr);
          open_set.push(ptr);
          // std::cout << "open set push: " << state.transpose() << std::endl;
        }
      if (open_set.empty()) {
        std::cout << "[prediction] no way!" << std::endl;
        return false;
      }
      curPtr = open_set.top();
      open_set.pop();
    }
    target_predcit.clear();
    while (curPtr != nullptr) {
      target_predcit.push_back(curPtr->p);
      curPtr = curPtr->parent;
    }
    std::reverse(target_predcit.begin(), target_predcit.end());
    return true;
  }
};

}  // namespace prediction
