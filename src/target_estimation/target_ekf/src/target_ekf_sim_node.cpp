// 模拟用的目标状态估计节点，接收yolo检测结果(实际上是目标的里程计，用于模拟情况下的简化测试)和无人机里程计，经过坐标变换和EKF滤波，发布目标的状态估计结果
// 如果目标不在无人机的视野范围内，短期内依靠纯预测0观测的ekf维持状态估计，长期则停止发送状态估计结果，直到目标重新进入视野范围内
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/time_synchronizer.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <quadrotor_msgs/OccMap3d.h>

#include <target_ekf/target_ekf.hpp>

typedef message_filters::sync_policies::ApproximateTime<nav_msgs::Odometry, nav_msgs::Odometry>
    YoloOdomSyncPolicy;
typedef message_filters::Synchronizer<YoloOdomSyncPolicy>
    YoloOdomSynchronizer;
ros::Publisher target_odom_pub_, yolo_odom_pub_;
Eigen::Matrix3d cam2body_R_;
Eigen::Vector3d cam2body_p_;
double fx_, fy_, cx_, cy_, width_, height_;
ros::Time last_update_stamp_;
double pitch_thr_ = 30;
bool check_fov_ = false;

std::shared_ptr<Ekf> ekfPtr_;
quadrotor_msgs::OccMap3d gridmap_;

bool isLineOfSightClear(const quadrotor_msgs::OccMap3d& map, const Eigen::Vector3d& start, const Eigen::Vector3d& end) {
  // 用于检查目标点与无人机之间的视线是否被障碍阻挡
  // Check if map is valid
  if (map.data.empty() || map.size_x == 0 || map.size_y == 0 || map.size_z == 0) {
    ROS_WARN("[ekf] Map not available, assuming line of sight clear.");
    return true;
  }
  // Add distance check to avoid long raycasting
  if ((end - start).norm() > 10.0) {
    return false;  // Assume blocked if too far
  }
  // 3D map
  double res = map.resolution;
  int sx = map.size_x;
  int sy = map.size_y;
  int sz = map.size_z;
  int ox = map.offset_x;
  int oy = map.offset_y;
  int oz = map.offset_z;

  double dist = (end - start).norm();
  int steps = std::max(1, (int)(dist / res));
  for (int i = 0; i <= steps; ++i) {
    double t = (double)i / steps;
    Eigen::Vector3d pt = start + t * (end - start);
    int x = std::round(pt.x() / res) - ox;
    int y = std::round(pt.y() / res) - oy;
    int z = std::round(pt.z() / res) - oz;
    if (x >= 0 && x < sx && y >= 0 && y < sy && z >= 0 && z < sz) {
      int index = (z * sy + y) * sx + x;
      if (map.data[index] == 1) {  // Occupied
        return false;
      }
    }
  }
  return true;
}

void gridmap_callback(const quadrotor_msgs::OccMap3dConstPtr& msg) {
  gridmap_ = *msg;
}

void predict_state_callback(const ros::TimerEvent& event) {
  double update_dt = (ros::Time::now() - last_update_stamp_).toSec();
  if (update_dt < 2.0) {
    ekfPtr_->predict();
  } else {
    ROS_WARN("[ekf] too long time no update!");
    return;
  }
  // publish target odom
  nav_msgs::Odometry target_odom;
  target_odom.header.stamp = ros::Time::now();
  target_odom.header.frame_id = "world";
  target_odom.pose.pose.position.x = ekfPtr_->pos().x();
  target_odom.pose.pose.position.y = ekfPtr_->pos().y();
  target_odom.pose.pose.position.z = ekfPtr_->pos().z();
  target_odom.twist.twist.linear.x = ekfPtr_->vel().x();
  target_odom.twist.twist.linear.y = ekfPtr_->vel().y();
  target_odom.twist.twist.linear.z = ekfPtr_->vel().z();
  Eigen::Vector3d rpy = ekfPtr_->rpy();
  Eigen::Quaterniond q = euler2quaternion(rpy);
  target_odom.pose.pose.orientation.w = q.w();
  target_odom.pose.pose.orientation.x = q.x();
  target_odom.pose.pose.orientation.y = q.y();
  target_odom.pose.pose.orientation.z = q.z();
  target_odom_pub_.publish(target_odom);
}

void update_state_callback(const nav_msgs::OdometryConstPtr& target_msg, const nav_msgs::OdometryConstPtr& odom_msg) {
  // std::cout << "yolo stamp: " << bboxes_msg->header.stamp << std::endl;
  // std::cout << "odom stamp: " << odom_msg->header.stamp << std::endl;
  Eigen::Vector3d odom_p, p;
  Eigen::Quaterniond odom_q, q;
  odom_p(0) = odom_msg->pose.pose.position.x;
  odom_p(1) = odom_msg->pose.pose.position.y;
  odom_p(2) = odom_msg->pose.pose.position.z;
  odom_q.w() = odom_msg->pose.pose.orientation.w;
  odom_q.x() = odom_msg->pose.pose.orientation.x;
  odom_q.y() = odom_msg->pose.pose.orientation.y;
  odom_q.z() = odom_msg->pose.pose.orientation.z;

  Eigen::Vector3d cam_p = odom_q.toRotationMatrix() * cam2body_p_ + odom_p;
  Eigen::Quaterniond cam_q = odom_q * Eigen::Quaterniond(cam2body_R_);

  p.x() = target_msg->pose.pose.position.x;
  p.y() = target_msg->pose.pose.position.y;
  p.z() = target_msg->pose.pose.position.z;
  q.w() = target_msg->pose.pose.orientation.w;
  q.x() = target_msg->pose.pose.orientation.x;
  q.y() = target_msg->pose.pose.orientation.y;
  q.z() = target_msg->pose.pose.orientation.z;

  Eigen::Vector3d rpy = quaternion2euler(q);

  // NOTE check whether it's in FOV
  if (check_fov_) {
    Eigen::Vector3d p_in_body = cam_q.inverse() * (p - cam_p);
    if (p_in_body.z() < 0.1 || p_in_body.z() > 5.0) {
      return;
    }
    double x = p_in_body.x() * fx_ / p_in_body.z() + cx_;
    if (x < 0 || x > height_) {
      return;
    }
    double y = p_in_body.y() * fy_ / p_in_body.z() + cy_;
    if (y < 0 || y > width_) {
      return;
    }
  }

  // Check line of sight 新增的障碍物遮挡检查，障碍物遮挡的情况下无法观测目标
  if (!isLineOfSightClear(gridmap_, cam_p, p)) {
    return;
  }

  // update target odom
  double update_dt = (ros::Time::now() - last_update_stamp_).toSec();
  if (update_dt > 5.0) {
    ekfPtr_->reset(p, rpy);
    ROS_WARN("[ekf] reset!");
  } else if (ekfPtr_->update(p, rpy)) {
    // ROS_WARN("[ekf] update!");
  } else {
    ROS_ERROR("[ekf] update invalid!");
    return;
  }
  last_update_stamp_ = ros::Time::now();
}

void odom_callback(const nav_msgs::OdometryConstPtr& odom_msg) {
  // std::cout << "_now stamp: " << odom_msg->header.stamp << std::endl;
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "target_ekf");
  ros::NodeHandle nh("~");
  last_update_stamp_ = ros::Time::now() - ros::Duration(10.0);

  std::vector<double> tmp;
  if (nh.param<std::vector<double>>("cam2body_R", tmp, std::vector<double>())) {
    cam2body_R_ = Eigen::Map<const Eigen::Matrix<double, -1, -1, Eigen::RowMajor>>(tmp.data(), 3, 3);
  }
  if (nh.param<std::vector<double>>("cam2body_p", tmp, std::vector<double>())) {
    cam2body_p_ = Eigen::Map<const Eigen::Matrix<double, -1, -1, Eigen::RowMajor>>(tmp.data(), 3, 1);
  }
  nh.getParam("cam_fx", fx_);
  nh.getParam("cam_fy", fy_);
  nh.getParam("cam_cx", cx_);
  nh.getParam("cam_cy", cy_);
  nh.getParam("cam_width", width_);
  nh.getParam("cam_height", height_);
  nh.getParam("pitch_thr", pitch_thr_);
  nh.getParam("check_fov", check_fov_);

  message_filters::Subscriber<nav_msgs::Odometry> yolo_sub_;
  message_filters::Subscriber<nav_msgs::Odometry> odom_sub_;
  std::shared_ptr<YoloOdomSynchronizer> yolo_odom_sync_Ptr_;
  ros::Timer ekf_predict_timer_;
  ros::Subscriber single_odom_sub = nh.subscribe("odom", 100, &odom_callback, ros::TransportHints().tcpNoDelay());
  target_odom_pub_ = nh.advertise<nav_msgs::Odometry>("target_odom", 1);
  yolo_odom_pub_ = nh.advertise<nav_msgs::Odometry>("yolo_odom", 1);
  ros::Subscriber gridmap_sub = nh.subscribe("gridmap_inflate", 1, &gridmap_callback);

  int ekf_rate = 20;
  nh.getParam("ekf_rate", ekf_rate);
  ekfPtr_ = std::make_shared<Ekf>(1.0 / ekf_rate);

  yolo_sub_.subscribe(nh, "yolo", 1, ros::TransportHints().tcpNoDelay());
  odom_sub_.subscribe(nh, "odom", 100, ros::TransportHints().tcpNoDelay());
  yolo_odom_sync_Ptr_ = std::make_shared<YoloOdomSynchronizer>(YoloOdomSyncPolicy(200), yolo_sub_, odom_sub_);
  yolo_odom_sync_Ptr_->registerCallback(boost::bind(&update_state_callback, _1, _2));
  ekf_predict_timer_ = nh.createTimer(ros::Duration(1.0 / ekf_rate), &predict_state_callback);

  ros::spin();
  return 0;
}
