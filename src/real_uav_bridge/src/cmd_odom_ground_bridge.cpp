#include "udp_ros_bridge_common.hpp"

#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <quadrotor_msgs/PositionCommand.h>
#include <ros/ros.h>

#include <algorithm>
#include <atomic>
#include <thread>

namespace rub = real_uav_bridge;

class GroundBridge {
 public:
  GroundBridge() : nh_(), pnh_("~") {
    pnh_.param("drone_ip", drone_ip_, std::string("127.0.0.1"));
    pnh_.param("cmd_port", cmd_port_, 18000);
    pnh_.param("listen_port", listen_port_, 18100);
    pnh_.param("cmd_topic", cmd_topic_, std::string("position_cmd"));
    pnh_.param("odom_topic", odom_topic_, std::string("odom"));
    pnh_.param("trigger_topic", trigger_topic_, std::string("traj_start_trigger"));
    pnh_.param("stats_period", stats_period_, 2.0);

    send_fd_ = rub::createUdpSocket();
    recv_fd_ = rub::createUdpSocket();
    if (send_fd_ < 0 || recv_fd_ < 0 ||
        !rub::fillAddress(drone_ip_, cmd_port_, drone_addr_) ||
        !rub::bindUdpPort(recv_fd_, listen_port_) ||
        !rub::setReceiveTimeout(recv_fd_, 0.2)) {
      ros::shutdown();
      return;
    }

    cmd_sub_ = nh_.subscribe(cmd_topic_, 20, &GroundBridge::cmdCallback, this,
                             ros::TransportHints().tcpNoDelay());
    odom_pub_ = nh_.advertise<nav_msgs::Odometry>(odom_topic_, 20);
    trigger_pub_ = nh_.advertise<geometry_msgs::PoseStamped>(trigger_topic_, 10);
    stats_timer_ = nh_.createTimer(ros::Duration(std::max(0.5, stats_period_)),
                                   &GroundBridge::statsCallback, this);

    recv_thread_ = std::thread(&GroundBridge::recvLoop, this);
    ROS_WARN("[ground_bridge] cmd %s -> %s:%d, UDP listen=%d, odom -> %s, trigger -> %s",
             cmd_topic_.c_str(), drone_ip_.c_str(), cmd_port_, listen_port_,
             odom_topic_.c_str(), trigger_topic_.c_str());
  }

  ~GroundBridge() {
    running_.store(false);
    rub::closeSocket(recv_fd_);
    rub::closeSocket(send_fd_);
    if (recv_thread_.joinable()) {
      recv_thread_.join();
    }
  }

 private:
  void cmdCallback(const quadrotor_msgs::PositionCommandConstPtr& msg) {
    if (rub::sendPacket(send_fd_, drone_addr_, rub::PacketType::POSITION_CMD, *msg)) {
      ++cmd_sent_count_;
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
          ROS_ERROR_THROTTLE(1.0, "[ground_bridge] recvfrom failed: %s", strerror(errno));
        }
        continue;
      }

      rub::PacketType type;
      uint32_t msg_size = 0;
      if (!rub::readPacketHeader(buf.data(), static_cast<size_t>(n), type, msg_size)) {
        ROS_WARN_THROTTLE(1.0, "[ground_bridge] drop malformed UDP packet len=%zd", n);
        continue;
      }

      const uint8_t* payload = buf.data() + rub::kHeaderSize;
      if (type == rub::PacketType::ODOM) {
        nav_msgs::Odometry odom;
        if (rub::deserializePayload(payload, msg_size, odom)) {
          odom_pub_.publish(odom);
          ++odom_recv_count_;
        }
      } else if (type == rub::PacketType::TRIGGER) {
        geometry_msgs::PoseStamped trigger;
        if (rub::deserializePayload(payload, msg_size, trigger)) {
          trigger_pub_.publish(trigger);
          ++trigger_recv_count_;
        }
      } else {
        ROS_WARN_THROTTLE(1.0, "[ground_bridge] unexpected packet type=%u",
                          static_cast<unsigned>(type));
      }
    }
  }

  void statsCallback(const ros::TimerEvent&) {
    ROS_INFO("[ground_bridge] sent_cmd=%llu recv_odom=%llu recv_trigger=%llu",
             static_cast<unsigned long long>(cmd_sent_count_.load()),
             static_cast<unsigned long long>(odom_recv_count_.load()),
             static_cast<unsigned long long>(trigger_recv_count_.load()));
  }

  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  ros::Subscriber cmd_sub_;
  ros::Publisher odom_pub_;
  ros::Publisher trigger_pub_;
  ros::Timer stats_timer_;

  std::string drone_ip_;
  std::string cmd_topic_;
  std::string odom_topic_;
  std::string trigger_topic_;
  int cmd_port_ = 18000;
  int listen_port_ = 18100;
  double stats_period_ = 2.0;

  int send_fd_ = -1;
  int recv_fd_ = -1;
  sockaddr_in drone_addr_;
  std::thread recv_thread_;
  std::atomic<bool> running_{true};
  std::atomic<uint64_t> cmd_sent_count_{0};
  std::atomic<uint64_t> odom_recv_count_{0};
  std::atomic<uint64_t> trigger_recv_count_{0};
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "cmd_odom_ground_bridge");
  GroundBridge bridge;
  ros::spin();
  return 0;
}
