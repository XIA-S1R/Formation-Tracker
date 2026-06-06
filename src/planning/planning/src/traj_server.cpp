#include <nav_msgs/Odometry.h>
#include <quadrotor_msgs/PolyTraj.h>
#include <quadrotor_msgs/PositionCommand.h>
#include <ros/ros.h>
#include <std_msgs/Empty.h>
#include <visualization_msgs/Marker.h>

#include <traj_opt/poly_traj_utils.hpp>

ros::Publisher pos_cmd_pub_;
ros::Time heartbeat_time_;
bool receive_traj_ = false;
bool flight_start_ = false;
quadrotor_msgs::PolyTraj trajMsg_, trajMsg_last_;
Eigen::Vector3d last_p_;
double last_yaw_ = 0;

struct ParsedTrajCache {
  bool valid = false;
  bool hover = false;
  int traj_id = -1;
  ros::Time start_time;
  std::vector<float> hover_p;
  quadrotor_msgs::PolyTraj msg;
  Trajectory traj;
  double total_duration = 0.0;
};

ParsedTrajCache traj_cache_, traj_cache_last_;

double wrap_angle(double angle) {
  return std::atan2(std::sin(angle), std::cos(angle));
}

std::pair<double, double> sample_yaw_cmd(const quadrotor_msgs::PolyTraj &trajMsg, double t_rel) {
  double yaw = trajMsg.yaw;
  double yaw_dot = 0.0;
  if (trajMsg.use_yaw_scan) {
    const double freq = std::max(0.0, (double)trajMsg.yaw_scan_freq);
    const double arg = 2.0 * M_PI * freq * t_rel + (double)trajMsg.yaw_scan_phase;
    yaw = (double)trajMsg.yaw_scan_base + (double)trajMsg.yaw_scan_amp * std::sin(arg);
    yaw_dot = (double)trajMsg.yaw_scan_amp * 2.0 * M_PI * freq * std::cos(arg);
    yaw = wrap_angle(yaw);
  }

  double d_yaw = wrap_angle(yaw - last_yaw_);
  const double max_yaw_step = 0.02;
  if (std::fabs(d_yaw) >= max_yaw_step) {
    yaw = wrap_angle(last_yaw_ + d_yaw / std::fabs(d_yaw) * max_yaw_step);
    yaw_dot = d_yaw / std::fabs(d_yaw) * max_yaw_step / 0.01;
  }
  return {yaw, yaw_dot};
}

void publish_cmd(int traj_id,
                 const Eigen::Vector3d &p,
                 const Eigen::Vector3d &v,
                 const Eigen::Vector3d &a,
                 double y, double yd) {
  quadrotor_msgs::PositionCommand cmd;
  cmd.header.stamp = ros::Time::now();
  cmd.header.frame_id = "world";
  cmd.trajectory_flag = quadrotor_msgs::PositionCommand::TRAJECTORY_STATUS_READY;
  cmd.trajectory_id = traj_id;

  cmd.position.x = p(0);
  cmd.position.y = p(1);
  cmd.position.z = p(2);
  cmd.velocity.x = v(0);
  cmd.velocity.y = v(1);
  cmd.velocity.z = v(2);
  cmd.acceleration.x = a(0);
  cmd.acceleration.y = a(1);
  cmd.acceleration.z = a(2);
  cmd.yaw = y;
  cmd.yaw_dot = yd;
  pos_cmd_pub_.publish(cmd);
  last_p_ = p;
}

bool build_traj_cache(const quadrotor_msgs::PolyTraj& trajMsg, ParsedTrajCache& cache) {
  cache = ParsedTrajCache();
  cache.msg = trajMsg;
  cache.traj_id = trajMsg.traj_id;
  cache.start_time = trajMsg.start_time;
  cache.hover = trajMsg.hover;
  cache.hover_p = trajMsg.hover_p;
  if (trajMsg.hover) {
    cache.valid = true;
    return true;
  }
  if (trajMsg.order != 5) {
    ROS_ERROR("[traj_server] Only support trajectory order equals 5 now!");
    return false;
  }
  if (trajMsg.duration.size() * (trajMsg.order + 1) != trajMsg.coef_x.size()) {
    ROS_ERROR("[traj_server] WRONG trajectory parameters!");
    return false;
  }

  int piece_nums = trajMsg.duration.size();
  std::vector<double> dura(piece_nums);
  std::vector<CoefficientMat> cMats(piece_nums);
  for (int i = 0; i < piece_nums; ++i) {
    int i6 = i * 6;
    cMats[i].row(0) << trajMsg.coef_x[i6 + 0], trajMsg.coef_x[i6 + 1], trajMsg.coef_x[i6 + 2],
        trajMsg.coef_x[i6 + 3], trajMsg.coef_x[i6 + 4], trajMsg.coef_x[i6 + 5];
    cMats[i].row(1) << trajMsg.coef_y[i6 + 0], trajMsg.coef_y[i6 + 1], trajMsg.coef_y[i6 + 2],
        trajMsg.coef_y[i6 + 3], trajMsg.coef_y[i6 + 4], trajMsg.coef_y[i6 + 5];
    cMats[i].row(2) << trajMsg.coef_z[i6 + 0], trajMsg.coef_z[i6 + 1], trajMsg.coef_z[i6 + 2],
        trajMsg.coef_z[i6 + 3], trajMsg.coef_z[i6 + 4], trajMsg.coef_z[i6 + 5];

    dura[i] = trajMsg.duration[i];
  }
  cache.traj = Trajectory(dura, cMats);
  cache.total_duration = cache.traj.getTotalDuration();
  cache.valid = true;
  return true;
}

bool exe_traj(const ParsedTrajCache& cache) {
  if (!cache.valid) {
    return false;
  }
  double t = (ros::Time::now() - cache.start_time).toSec();
  if (t > 0) {
    if (cache.hover) {
      if (cache.hover_p.size() != 3) {
        ROS_ERROR("[traj_server] hover_p is not 3d!");
      }
      Eigen::Vector3d p, v0;
      p.x() = cache.hover_p[0];
      p.y() = cache.hover_p[1];
      p.z() = cache.hover_p[2];
      v0.setZero();
      publish_cmd(cache.traj_id, p, v0, v0, last_yaw_, 0);  // TODO yaw
      return true;
    }
    if (t > cache.total_duration) {
      ROS_ERROR("[traj_server] trajectory too short left!");
      return false;
    }
    Eigen::Vector3d p, v, a;
    p = cache.traj.getPos(t);
    v = cache.traj.getVel(t);
    a = cache.traj.getAcc(t);
    auto yaw_cmd = sample_yaw_cmd(cache.msg, t);
    publish_cmd(cache.traj_id, p, v, a, yaw_cmd.first, yaw_cmd.second);
    last_yaw_ = yaw_cmd.first;
    return true;
  }
  return false;
}

void heartbeatCallback(const std_msgs::EmptyConstPtr &msg) {
  heartbeat_time_ = ros::Time::now();
}

void polyTrajCallback(const quadrotor_msgs::PolyTrajConstPtr &msgPtr) {
  trajMsg_ = *msgPtr;
  if (!build_traj_cache(trajMsg_, traj_cache_)) {
    return;
  }
  if (!receive_traj_) {
    trajMsg_last_ = trajMsg_;
    traj_cache_last_ = traj_cache_;
    receive_traj_ = true;
  }
}

void cmdCallback(const ros::TimerEvent &e) {
  if (!receive_traj_) {
    return;
  }
  ros::Time time_now = ros::Time::now();
  if ((time_now - heartbeat_time_).toSec() > 0.5) {
    ROS_ERROR_ONCE("[traj_server] Lost heartbeat from the planner, is he dead?");
    publish_cmd(trajMsg_.traj_id, last_p_, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), 0, 0);  // TODO yaw
    return;
  }
  if (exe_traj(traj_cache_)) {
    trajMsg_last_ = trajMsg_;
    traj_cache_last_ = traj_cache_;
    return;
  } else if (exe_traj(traj_cache_last_)) {
    return;
  }
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "traj_server");
  ros::NodeHandle nh("~");

  ros::Subscriber poly_traj_sub = nh.subscribe("trajectory", 10, polyTrajCallback);
  ros::Subscriber heartbeat_sub = nh.subscribe("heartbeat", 10, heartbeatCallback);

  pos_cmd_pub_ = nh.advertise<quadrotor_msgs::PositionCommand>("position_cmd", 50);

  ros::Timer cmd_timer = nh.createTimer(ros::Duration(0.01), cmdCallback);

  ros::Duration(1.0).sleep();

  ROS_WARN("[Traj server]: ready.");

  ros::spin();

  return 0;
}
