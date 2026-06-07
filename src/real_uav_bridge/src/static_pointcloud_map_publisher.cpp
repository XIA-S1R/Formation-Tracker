#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include <algorithm>
#include <cctype>
#include <fstream>
#include <sstream>
#include <string>

bool loadCsvLike(const std::string& path, pcl::PointCloud<pcl::PointXYZ>& cloud) {
  std::ifstream in(path);
  if (!in.is_open()) {
    ROS_ERROR("Cannot open map file: %s", path.c_str());
    return false;
  }

  std::string line;
  int line_no = 0;
  while (std::getline(in, line)) {
    ++line_no;
    const auto comment_pos = line.find('#');
    if (comment_pos != std::string::npos) {
      line = line.substr(0, comment_pos);
    }
    std::replace(line.begin(), line.end(), ',', ' ');
    std::stringstream ss(line);
    float x = 0.0f, y = 0.0f, z = 0.0f;
    if (!(ss >> x >> y >> z)) {
      continue;
    }
    cloud.push_back(pcl::PointXYZ(x, y, z));
  }
  cloud.width = cloud.size();
  cloud.height = 1;
  cloud.is_dense = false;
  ROS_WARN("[static_map] loaded %zu points from %s", cloud.size(), path.c_str());
  return !cloud.empty();
}

std::string lowerExt(const std::string& path) {
  const size_t dot = path.find_last_of('.');
  if (dot == std::string::npos) {
    return "";
  }
  std::string ext = path.substr(dot + 1);
  std::transform(ext.begin(), ext.end(), ext.begin(), ::tolower);
  return ext;
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "static_pointcloud_map_publisher");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  std::string map_file;
  std::string frame_id;
  double publish_hz = 1.0;
  pnh.param("map_file", map_file, std::string(""));
  pnh.param("frame_id", frame_id, std::string("world"));
  pnh.param("publish_hz", publish_hz, 1.0);

  if (map_file.empty()) {
    ROS_FATAL("[static_map] map_file is empty");
    return 1;
  }

  pcl::PointCloud<pcl::PointXYZ> cloud;
  const std::string ext = lowerExt(map_file);
  bool ok = false;
  if (ext == "pcd") {
    ok = pcl::io::loadPCDFile<pcl::PointXYZ>(map_file, cloud) == 0;
    if (ok) {
      ROS_WARN("[static_map] loaded %zu PCD points from %s", cloud.size(), map_file.c_str());
    }
  } else {
    ok = loadCsvLike(map_file, cloud);
  }

  if (!ok || cloud.empty()) {
    ROS_FATAL("[static_map] failed to load non-empty map: %s", map_file.c_str());
    return 1;
  }

  sensor_msgs::PointCloud2 msg;
  pcl::toROSMsg(cloud, msg);
  msg.header.frame_id = frame_id;

  ros::Publisher pub = nh.advertise<sensor_msgs::PointCloud2>("/global_map", 1, true);
  ros::Rate rate(std::max(0.1, publish_hz));
  while (ros::ok()) {
    msg.header.stamp = ros::Time::now();
    pub.publish(msg);
    ros::spinOnce();
    rate.sleep();
  }
  return 0;
}
