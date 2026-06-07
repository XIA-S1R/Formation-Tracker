#pragma once

#include <arpa/inet.h>
#include <errno.h>
#include <netinet/in.h>
#include <ros/console.h>
#include <ros/serialization.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <unistd.h>

#include <cstdint>
#include <cstring>
#include <exception>
#include <string>
#include <vector>

namespace real_uav_bridge {

enum class PacketType : uint32_t {
  POSITION_CMD = 200,
  ODOM = 201,
  TRIGGER = 202,
};

constexpr size_t kHeaderSize = sizeof(uint32_t) * 2;
constexpr size_t kMaxPacketSize = 1024 * 1024;

inline int createUdpSocket() {
  int fd = socket(AF_INET, SOCK_DGRAM, 0);
  if (fd < 0) {
    ROS_FATAL("Failed to create UDP socket: %s", strerror(errno));
  }
  return fd;
}

inline bool fillAddress(const std::string& ip, int port, sockaddr_in& addr) {
  std::memset(&addr, 0, sizeof(addr));
  addr.sin_family = AF_INET;
  addr.sin_port = htons(static_cast<uint16_t>(port));
  if (inet_pton(AF_INET, ip.c_str(), &addr.sin_addr) <= 0) {
    ROS_ERROR("Invalid UDP address: %s:%d", ip.c_str(), port);
    return false;
  }
  return true;
}

inline bool bindUdpPort(int fd, int port) {
  int opt = 1;
  if (setsockopt(fd, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt)) < 0) {
    ROS_ERROR("setsockopt(SO_REUSEADDR) failed: %s", strerror(errno));
    return false;
  }

  sockaddr_in addr;
  std::memset(&addr, 0, sizeof(addr));
  addr.sin_family = AF_INET;
  addr.sin_addr.s_addr = INADDR_ANY;
  addr.sin_port = htons(static_cast<uint16_t>(port));

  if (bind(fd, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) < 0) {
    ROS_ERROR("Failed to bind UDP port %d: %s", port, strerror(errno));
    return false;
  }
  return true;
}

inline bool setReceiveTimeout(int fd, double timeout_sec) {
  timeval tv;
  tv.tv_sec = static_cast<time_t>(timeout_sec);
  tv.tv_usec = static_cast<suseconds_t>((timeout_sec - tv.tv_sec) * 1e6);
  if (setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv)) < 0) {
    ROS_ERROR("setsockopt(SO_RCVTIMEO) failed: %s", strerror(errno));
    return false;
  }
  return true;
}

template <typename MsgT>
std::vector<uint8_t> serializePacket(PacketType type, const MsgT& msg) {
  namespace ser = ros::serialization;
  const uint32_t msg_size = ser::serializationLength(msg);
  std::vector<uint8_t> packet(kHeaderSize + msg_size);

  uint32_t net_type = htonl(static_cast<uint32_t>(type));
  uint32_t net_size = htonl(msg_size);
  std::memcpy(packet.data(), &net_type, sizeof(net_type));
  std::memcpy(packet.data() + sizeof(net_type), &net_size, sizeof(net_size));

  ser::OStream stream(packet.data() + kHeaderSize, msg_size);
  ser::serialize(stream, msg);
  return packet;
}

inline bool readPacketHeader(const uint8_t* data, size_t len, PacketType& type, uint32_t& msg_size) {
  if (len < kHeaderSize) {
    return false;
  }

  uint32_t net_type = 0;
  uint32_t net_size = 0;
  std::memcpy(&net_type, data, sizeof(net_type));
  std::memcpy(&net_size, data + sizeof(net_type), sizeof(net_size));

  type = static_cast<PacketType>(ntohl(net_type));
  msg_size = ntohl(net_size);
  return len == kHeaderSize + msg_size && len <= kMaxPacketSize;
}

template <typename MsgT>
bool deserializePayload(const uint8_t* data, uint32_t msg_size, MsgT& msg) {
  namespace ser = ros::serialization;
  try {
    ser::IStream stream(const_cast<uint8_t*>(data), msg_size);
    ser::deserialize(stream, msg);
    return true;
  } catch (const std::exception& e) {
    ROS_ERROR("ROS message deserialize failed: %s", e.what());
    return false;
  }
}

template <typename MsgT>
bool sendPacket(int fd, const sockaddr_in& addr, PacketType type, const MsgT& msg) {
  const std::vector<uint8_t> packet = serializePacket(type, msg);
  const ssize_t sent = sendto(fd, packet.data(), packet.size(), 0,
                              reinterpret_cast<const sockaddr*>(&addr), sizeof(addr));
  if (sent != static_cast<ssize_t>(packet.size())) {
    ROS_ERROR_THROTTLE(1.0, "UDP send failed: sent=%zd expected=%zu errno=%s",
                       sent, packet.size(), strerror(errno));
    return false;
  }
  return true;
}

inline void closeSocket(int& fd) {
  if (fd >= 0) {
    close(fd);
    fd = -1;
  }
}

}  // namespace real_uav_bridge
