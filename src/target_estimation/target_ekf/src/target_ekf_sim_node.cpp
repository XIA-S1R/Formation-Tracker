// 模拟用的目标状态估计节点，接收yolo检测结果(实际上是目标的里程计，用于模拟情况下的简化测试)和无人机里程计，经过坐标变换和EKF滤波，发布目标的状态估计结果
// 如果目标不在无人机的视野范围内，短期内依靠纯预测0观测的ekf维持状态估计，长期则停止发送状态估计结果，直到目标重新进入视野范围内
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/time_synchronizer.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <unordered_set>
#include <cmath>

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
bool ekf_initialized_ = false;      // ekf 是否已经有过至少一次有效 update
int ekf_reset_suppress_count_ = 0;  // reset 后抑制发布的帧数

std::shared_ptr<Ekf> ekfPtr_;

// 直接用全局点云构建简易占据栅格
struct SimpleOccMap {
  double resolution = 0.3;
  std::unordered_set<int64_t> occ_cells;
  bool received = false;

  int64_t toKey(int x, int y, int z) const {
    return ((int64_t)(x + 32768) << 32) | ((int64_t)(y + 32768) << 16) | (int64_t)(z + 32768);
  }

  void fromPointCloud(const pcl::PointCloud<pcl::PointXYZ>& cloud, double res) {
    resolution = res;
    occ_cells.clear();
    for (const auto& pt : cloud) {
      int x = (int)std::floor(pt.x / resolution);
      int y = (int)std::floor(pt.y / resolution);
      int z = (int)std::floor(pt.z / resolution);
      occ_cells.insert(toKey(x, y, z));
    }
    received = true;
  }

  bool isOccupied(const Eigen::Vector3d& p) const {
    int x = (int)std::floor(p.x() / resolution);
    int y = (int)std::floor(p.y() / resolution);
    int z = (int)std::floor(p.z() / resolution);
    return occ_cells.count(toKey(x, y, z)) > 0;
  }
} occMap_;

bool isLineOfSightClear(const Eigen::Vector3d& start, const Eigen::Vector3d& end) {
  // 地图未收到时不做遮挡检测，直接认为视线畅通，避免 ekf 无法初始化
  if (!occMap_.received) return true;

  double dist = (end - start).norm();
  if (dist > 15.0) return false;

  int steps = std::max(1, (int)(dist / (occMap_.resolution * 0.5)));
  for (int i = 1; i < steps; ++i) {  // i从1开始，跳过起点（无人机自身位置）
    double t = (double)i / steps;
    Eigen::Vector3d pt = start + t * (end - start);
    if (occMap_.isOccupied(pt)) return false;
  }
  return true;
}

void global_map_callback(const sensor_msgs::PointCloud2ConstPtr& msg) {
  pcl::PointCloud<pcl::PointXYZ> cloud;
  pcl::fromROSMsg(*msg, cloud);
  occMap_.fromPointCloud(cloud, 0.3);  // 与mockamap的resolution一致
  ROS_INFO_ONCE("[ekf] Global map received, %zu obstacle points.", cloud.size());
}

void predict_state_callback(const ros::TimerEvent& event) {
  // reset 后静默几帧
  if (ekf_reset_suppress_count_ > 0) {
    ekf_reset_suppress_count_--;
    return;
  }

  // ekf 未初始化则不发布
  if (!ekf_initialized_) {
    return;
  }

  double update_dt = (ros::Time::now() - last_update_stamp_).toSec();
  if (update_dt < 2.0) {
    ekfPtr_->predict();
  } else {
    ROS_WARN("[ekf] too long time no update!");
    return;
  }

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
    if (p_in_body.z() < 0.1 || p_in_body.z() > 5.0) return;
    double x = p_in_body.x() * fx_ / p_in_body.z() + cx_;
    if (x < 0 || x > height_) return;
    double y = p_in_body.y() * fy_ / p_in_body.z() + cy_;
    if (y < 0 || y > width_) return;
  }

  if (!occMap_.received) {
    return;  // 地图未到，跳过但不刷新 last_update_stamp_
  }

  // 视线遮挡检测：只在 ekf 已初始化后才生效
  // 首次初始化时无论视线是否遮挡都必须完成，否则视线被遮挡的无人机永远无法启动
  if (ekf_initialized_ && !isLineOfSightClear(cam_p, p)) {
    return;
  }

  double update_dt = (ros::Time::now() - last_update_stamp_).toSec();
  if (!ekf_initialized_ || update_dt > 1.0) {
    // 首次初始化或长时间丢失后 reset：
    // 静默 3 帧（约 150ms@20Hz），让 ekf 先用真实观测稳定再发布给规划器
    ekfPtr_->reset(p, rpy);
    ekf_initialized_ = true;
    ekf_reset_suppress_count_ = 3;
    ROS_WARN("[ekf] reset! suppressing %d frames.", ekf_reset_suppress_count_);
  } else if (ekfPtr_->update(p, rpy)) {
    // 正常更新，清空静默计数（如果还在静默期也立即解除）
    ekf_reset_suppress_count_ = 0;
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
  ros::Subscriber global_map_sub = nh.subscribe("global_map", 10, &global_map_callback);

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
