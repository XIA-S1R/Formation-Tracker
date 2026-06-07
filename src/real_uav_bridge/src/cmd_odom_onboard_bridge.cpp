#include "udp_ros_bridge_common.hpp"

#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <quadrotor_msgs/PositionCommand.h>
#include <ros/ros.h>

#include <algorithm>
#include <atomic>
#include <thread>

namespace rub = real_uav_bridge;

class OnboardBridge {
 public:
  OnboardBridge() : nh_(), pnh_("~") {
    pnh_.param("ground_ip", ground_ip_, std::string("127.0.0.1"));
    pnh_.param("ground_port", ground_port_, 18100);
    pnh_.param("cmd_listen_port", cmd_listen_port_, 18000);
    pnh_.param("cmd_topic", cmd_topic_, std::string("/tracker_cmd/position_cmd"));
    pnh_.param("odom_topic", odom_topic_, std::string("/vins_estimator/imu_propagate"));
    pnh_.param("trigger_topic", trigger_topic_, std::string("/traj_start_trigger"));
    pnh_.param("odom_send_hz", odom_send_hz_, 50.0);
    pnh_.param("stats_period", stats_period_, 2.0);
    odom_send_hz_ = std::max(1.0, odom_send_hz_);

    send_fd_ = rub::createUdpSocket();
    recv_fd_ = rub::createUdpSocket();
    if (send_fd_ < 0 || recv_fd_ < 0 ||
        !rub::fillAddress(ground_ip_, ground_port_, ground_addr_) ||
        !rub::bindUdpPort(recv_fd_, cmd_listen_port_) ||
        !rub::setReceiveTimeout(recv_fd_, 0.2)) {
      ros::shutdown();
      return;
    }

    cmd_pub_ = nh_.advertise<quadrotor_msgs::PositionCommand>(cmd_topic_, 20);
    odom_sub_ = nh_.subscribe(odom_topic_, 50, &OnboardBridge::odomCallback, this,
                              ros::TransportHints().tcpNoDelay());
    trigger_sub_ = nh_.subscribe(trigger_topic_, 10, &OnboardBridge::triggerCallback, this,
                                 ros::TransportHints().tcpNoDelay());
    stats_timer_ = nh_.createTimer(ros::Duration(std::max(0.5, stats_period_)),
                                   &OnboardBridge::statsCallback, this);

    recv_thread_ = std::thread(&OnboardBridge::recvLoop, this);
    ROS_WARN("[onboard_bridge] UDP cmd listen=%d -> %s, odom %s -> %s:%d, trigger=%s",
             cmd_listen_port_, cmd_topic_.c_str(), odom_topic_.c_str(),
             ground_ip_.c_str(), ground_port_, trigger_topic_.c_str());
  }

  ~OnboardBridge() {
    running_.store(false);
    rub::closeSocket(recv_fd_);
    rub::closeSocket(send_fd_);
    if (recv_thread_.joinable()) {
      recv_thread_.join();
    }
  }

 private:
  void odomCallback(const nav_msgs::OdometryConstPtr& msg) {
    const ros::Time now = ros::Time::now();
    if (last_odom_send_.isValid() &&
        (now - last_odom_send_).toSec() < 1.0 / odom_send_hz_) {
      return;
    }
    last_odom_send_ = now;
    if (rub::sendPacket(send_fd_, ground_addr_, rub::PacketType::ODOM, *msg)) {
      ++odom_sent_count_;
    }
  }

  void triggerCallback(const geometry_msgs::PoseStampedConstPtr& msg) {
    if (rub::sendPacket(send_fd_, ground_addr_, rub::PacketType::TRIGGER, *msg)) {
      ++trigger_sent_count_;
    }
  }

  void recvLoop() {
    std::vector<uint8_t> buf(rub::kMaxPacketSize);
    while (ros::ok() && running_.load()) {
      sockaddr_in client_addr;
      socklen_t addr_len = sizeof(client_addr);
      const ssize_t n = recvfrom(recv_fd_, buf.data(), buf.size(), 0,
                                 reinterpret_cast<sockaddr*>(&client_addr), &addr_len);
      if (n <= 0) {
        if (errno == EAGAIN || errno == EWOULDBLOCK) {
          continue;
        }
        if (running_.load()) {
          ROS_ERROR_THROTTLE(1.0, "[onboard_bridge] recvfrom failed: %s", strerror(errno));
        }
        continue;
      }

      rub::PacketType type;
      uint32_t msg_size = 0;
      if (!rub::readPacketHeader(buf.data(), static_cast<size_t>(n), type, msg_size)) {
        ROS_WARN_THROTTLE(1.0, "[onboard_bridge] drop malformed UDP packet len=%zd", n);
        continue;
      }
      if (type != rub::PacketType::POSITION_CMD) {
        ROS_WARN_THROTTLE(1.0, "[onboard_bridge] unexpected packet type=%u",
                          static_cast<unsigned>(type));
        continue;
      }

      quadrotor_msgs::PositionCommand cmd;
      if (rub::deserializePayload(buf.data() + rub::kHeaderSize, msg_size, cmd)) {
        cmd_pub_.publish(cmd);
        ++cmd_recv_count_;
      }
    }
  }

  void statsCallback(const ros::TimerEvent&) {
    ROS_INFO("[onboard_bridge] recv_cmd=%llu sent_odom=%llu sent_trigger=%llu",
             static_cast<unsigned long long>(cmd_recv_count_.load()),
             static_cast<unsigned long long>(odom_sent_count_.load()),
             static_cast<unsigned long long>(trigger_sent_count_.load()));
  }

  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  ros::Publisher cmd_pub_;
  ros::Subscriber odom_sub_;
  ros::Subscriber trigger_sub_;
  ros::Timer stats_timer_;

  std::string ground_ip_;
  std::string cmd_topic_;
  std::string odom_topic_;
  std::string trigger_topic_;
  int ground_port_ = 18100;
  int cmd_listen_port_ = 18000;
  double odom_send_hz_ = 50.0;
  double stats_period_ = 2.0;

  int send_fd_ = -1;
  int recv_fd_ = -1;
  sockaddr_in ground_addr_;
  ros::Time last_odom_send_;
  std::thread recv_thread_;
  std::atomic<bool> running_{true};
  std::atomic<uint64_t> cmd_recv_count_{0};
  std::atomic<uint64_t> odom_sent_count_{0};
  std::atomic<uint64_t> trigger_sent_count_{0};
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "cmd_odom_onboard_bridge");
  OnboardBridge bridge;
  ros::spin();
  return 0;
}
