#ifndef UAV_UTILS_UTILS_H
#define UAV_UTILS_UTILS_H

#include <cmath>

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <nav_msgs/Odometry.h>

namespace uav_utils {

template <typename Scalar>
inline Scalar normalize_angle(Scalar a) {
  constexpr Scalar kPi = static_cast<Scalar>(M_PI);
  while (a > kPi) {
    a -= static_cast<Scalar>(2.0) * kPi;
  }
  while (a < -kPi) {
    a += static_cast<Scalar>(2.0) * kPi;
  }
  return a;
}

template <typename Scalar>
inline Scalar get_yaw_from_quaternion(const Eigen::Quaternion<Scalar>& q) {
  return std::atan2(
      static_cast<Scalar>(2.0) * (q.w() * q.z() + q.x() * q.y()),
      static_cast<Scalar>(1.0) -
          static_cast<Scalar>(2.0) * (q.y() * q.y() + q.z() * q.z()));
}

inline void extract_odometry(const nav_msgs::OdometryConstPtr& msg,
                             Eigen::Vector3d& p,
                             Eigen::Vector3d& v,
                             Eigen::Quaterniond& q,
                             Eigen::Vector3d& w) {
  p << msg->pose.pose.position.x,
       msg->pose.pose.position.y,
       msg->pose.pose.position.z;
  v << msg->twist.twist.linear.x,
       msg->twist.twist.linear.y,
       msg->twist.twist.linear.z;
  q = Eigen::Quaterniond(msg->pose.pose.orientation.w,
                         msg->pose.pose.orientation.x,
                         msg->pose.pose.orientation.y,
                         msg->pose.pose.orientation.z);
  if (q.norm() > 1e-6) {
    q.normalize();
  } else {
    q.setIdentity();
  }
  w << msg->twist.twist.angular.x,
       msg->twist.twist.angular.y,
       msg->twist.twist.angular.z;
}

}  // namespace uav_utils

#endif  // UAV_UTILS_UTILS_H
