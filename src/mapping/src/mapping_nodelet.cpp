#include <mapping/mapping.h>
#include <nav_msgs/Odometry.h>
#include <nodelet/nodelet.h>
#include <pcl_conversions/pcl_conversions.h>
#include <quadrotor_msgs/OccMap3d.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include <atomic>
#include <thread>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace mapping {

class Nodelet : public nodelet::Nodelet {
 private:
  std::thread initThread_;

  bool use_global_map_ = false;

  double sensor_range_;
  double sensor_rate_;

  // global map storage (shared by both modes)
  pcl::PointCloud<pcl::PointXYZ> global_cloud_;
  bool global_map_received_ = false;
  ros::Subscriber global_map_sub_;

  // lidar mode: odom
  ros::Subscriber odom_sub_;
  std::atomic_flag odom_lock_ = ATOMIC_FLAG_INIT;
  Eigen::Vector3d cur_pos_;
  bool odom_received_ = false;
  ros::Timer sense_timer_;

  // global map mode: publish timer
  ros::Timer global_map_timer_;

  ros::Publisher gridmap_inflate_pub_;

  // NOTE for mask target
  bool use_mask_ = false;
  ros::Subscriber target_odom_sub_;
  std::atomic_flag target_lock_ = ATOMIC_FLAG_INIT;
  Eigen::Vector3d target_odom_;

  OccGridMap gridmap_;
  int inflate_size_;

  // global map mode: receive once, setOcc + inflate, then publish periodically
  void global_map_callback(const sensor_msgs::PointCloud2ConstPtr& msgPtr) {
    if (global_map_received_) return;
    pcl::PointCloud<pcl::PointXYZ> cloud;
    pcl::fromROSMsg(*msgPtr, cloud);
    for (const auto& pt : cloud) {
      gridmap_.setOcc(Eigen::Vector3d(pt.x, pt.y, pt.z));
    }
    gridmap_.inflate(inflate_size_);
    global_map_received_ = true;
    ROS_WARN("[mapping] GLOBAL MAP RECEIVED!");
  }

  // lidar mode: just cache the raw point cloud
  void lidar_map_callback(const sensor_msgs::PointCloud2ConstPtr& msgPtr) {
    if (global_map_received_) return;
    pcl::fromROSMsg(*msgPtr, global_cloud_);
    global_map_received_ = true;
    ROS_INFO("[mapping] global map received, %zu points", global_cloud_.size());
  }

  void global_map_timer_callback(const ros::TimerEvent&) {
    if (!global_map_received_) return;
    quadrotor_msgs::OccMap3d gridmap_msg;
    gridmap_msg.header.frame_id = "world";
    gridmap_msg.header.stamp = ros::Time::now();
    gridmap_.to_msg(gridmap_msg);
    gridmap_inflate_pub_.publish(gridmap_msg);
  }

  // lidar mode: filter global cloud by sensor range, updateMap each frame
  void odom_callback(const nav_msgs::OdometryConstPtr& msgPtr) {
    while (odom_lock_.test_and_set());
    cur_pos_.x() = msgPtr->pose.pose.position.x;
    cur_pos_.y() = msgPtr->pose.pose.position.y;
    cur_pos_.z() = msgPtr->pose.pose.position.z;
    odom_received_ = true;
    odom_lock_.clear();
  }

  void sense_timer_callback(const ros::TimerEvent&) {
    if (!global_map_received_ || !odom_received_) return;

    while (odom_lock_.test_and_set());
    Eigen::Vector3d sensor_p = cur_pos_;
    odom_lock_.clear();

    double range2 = sensor_range_ * sensor_range_;
    std::vector<Eigen::Vector3d> obs_pts;
    for (const auto& pt : global_cloud_) {
      Eigen::Vector3d p(pt.x, pt.y, pt.z);
      if ((p - sensor_p).squaredNorm() <= range2)
        obs_pts.push_back(p);
    }

    gridmap_.updateMap(sensor_p, obs_pts);

    if (use_mask_) {
      while (target_lock_.test_and_set());
      Eigen::Vector3d ld = target_odom_, ru = target_odom_;
      ld.x() -= 0.5; ld.y() -= 0.5; ld.z() -= 1.0;
      ru.x() += 0.5; ru.y() += 0.5; ru.z() += 1.0;
      gridmap_.setFree(ld, ru);
      target_lock_.clear();
    }

    quadrotor_msgs::OccMap3d gridmap_msg;
    gridmap_msg.header.frame_id = "world";
    gridmap_msg.header.stamp = ros::Time::now();
    gridmap_.to_msg(gridmap_msg);
    gridmap_inflate_pub_.publish(gridmap_msg);
  }

  void target_odom_callback(const nav_msgs::OdometryConstPtr& msgPtr) {
    while (target_lock_.test_and_set());
    target_odom_.x() = msgPtr->pose.pose.position.x;
    target_odom_.y() = msgPtr->pose.pose.position.y;
    target_odom_.z() = msgPtr->pose.pose.position.z;
    target_lock_.clear();
  }

  void init(ros::NodeHandle& nh) {
    nh.param("use_global_map", use_global_map_, false);
    nh.getParam("inflate_size", inflate_size_);

    double res;
    nh.getParam("resolution", res);

    if (use_global_map_) {
      double x, y, z;
      nh.getParam("x_length", x);
      nh.getParam("y_length", y);
      nh.getParam("z_length", z);
      gridmap_.setup(res, Eigen::Vector3d(x, y, z), 10, true);
    } else {
      nh.getParam("sensor_range", sensor_range_);
      nh.param("sensor_rate", sensor_rate_, 10.0);
      Eigen::Vector3d map_size;
      nh.getParam("local_x", map_size.x());
      nh.getParam("local_y", map_size.y());
      nh.getParam("local_z", map_size.z());
      gridmap_.setup(res, map_size, sensor_range_);
      int p_min, p_max, p_hit, p_mis, p_occ, p_def;
      nh.getParam("p_min", p_min);
      nh.getParam("p_max", p_max);
      nh.getParam("p_hit", p_hit);
      nh.getParam("p_mis", p_mis);
      nh.getParam("p_occ", p_occ);
      nh.getParam("p_def", p_def);
      gridmap_.setupP(p_min, p_max, p_hit, p_mis, p_occ, p_def);
    }
    gridmap_.inflate_size = inflate_size_;

    nh.getParam("use_mask", use_mask_);

    gridmap_inflate_pub_ = nh.advertise<quadrotor_msgs::OccMap3d>("gridmap_inflate", 1);

    if (use_global_map_) {
      global_map_sub_ = nh.subscribe<sensor_msgs::PointCloud2>("global_map", 1, &Nodelet::global_map_callback, this);
      global_map_timer_ = nh.createTimer(ros::Duration(1.0), &Nodelet::global_map_timer_callback, this);
    } else {
      global_map_sub_ = nh.subscribe<sensor_msgs::PointCloud2>("global_map", 1, &Nodelet::lidar_map_callback, this);
      odom_sub_ = nh.subscribe<nav_msgs::Odometry>("odom", 10, &Nodelet::odom_callback, this,
                                                    ros::TransportHints().tcpNoDelay());
      sense_timer_ = nh.createTimer(ros::Duration(1.0 / sensor_rate_), &Nodelet::sense_timer_callback, this);
    }

    if (use_mask_) {
      target_odom_sub_ = nh.subscribe<nav_msgs::Odometry>("target", 1, &Nodelet::target_odom_callback, this,
                                                           ros::TransportHints().tcpNoDelay());
    }
  }

 public:
  void onInit(void) {
    ros::NodeHandle nh(getMTPrivateNodeHandle());
    initThread_ = std::thread(std::bind(&Nodelet::init, this, nh));
  }
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

}  // namespace mapping

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mapping::Nodelet, nodelet::Nodelet);
