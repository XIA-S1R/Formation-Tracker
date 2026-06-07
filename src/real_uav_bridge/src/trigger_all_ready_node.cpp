#include <geometry_msgs/PoseStamped.h>
#include <boost/bind.hpp>
#include <ros/ros.h>

#include <algorithm>
#include <string>
#include <vector>

class TriggerAllReady {
 public:
  TriggerAllReady() : nh_(), pnh_("~") {
    pnh_.param("num_drones", num_drones_, 3);
    pnh_.param("input_topic_prefix", input_topic_prefix_, std::string("/drone"));
    pnh_.param("input_topic_suffix", input_topic_suffix_, std::string("/traj_start_trigger"));
    pnh_.param("output_topic", output_topic_, std::string("/triger"));
    pnh_.param("max_trigger_age", max_trigger_age_, 30.0);
    pnh_.param("publish_once", publish_once_, true);

    num_drones_ = std::max(1, num_drones_);
    stamps_.assign(num_drones_, ros::Time(0));
    poses_.assign(num_drones_, geometry_msgs::PoseStamped());

    for (int i = 0; i < num_drones_; ++i) {
      const std::string topic = input_topic_prefix_ + std::to_string(i) + input_topic_suffix_;
      subs_.push_back(nh_.subscribe<geometry_msgs::PoseStamped>(
          topic, 10, boost::bind(&TriggerAllReady::triggerCallback, this, _1, i)));
      ROS_WARN("[trigger_all_ready] waiting %s", topic.c_str());
    }
    pub_ = nh_.advertise<geometry_msgs::PoseStamped>(output_topic_, 1, true);
    status_timer_ = nh_.createTimer(ros::Duration(1.0), &TriggerAllReady::statusCallback, this);
  }

 private:
  void triggerCallback(const geometry_msgs::PoseStampedConstPtr& msg, int id) {
    if (id < 0 || id >= num_drones_) {
      return;
    }
    stamps_[id] = ros::Time::now();
    poses_[id] = *msg;
    tryPublish();
  }

  void tryPublish() {
    if (publish_once_ && published_) {
      return;
    }
    const ros::Time now = ros::Time::now();
    for (int i = 0; i < num_drones_; ++i) {
      if (!stamps_[i].isValid() || (now - stamps_[i]).toSec() > max_trigger_age_) {
        return;
      }
    }
    geometry_msgs::PoseStamped out = poses_.front();
    out.header.stamp = now;
    pub_.publish(out);
    published_ = true;
    ROS_WARN("[trigger_all_ready] all %d trackers ready, published %s",
             num_drones_, output_topic_.c_str());
  }

  void statusCallback(const ros::TimerEvent&) {
    if (publish_once_ && published_) {
      return;
    }
    const ros::Time now = ros::Time::now();
    std::string waiting;
    for (int i = 0; i < num_drones_; ++i) {
      if (!stamps_[i].isValid() || (now - stamps_[i]).toSec() > max_trigger_age_) {
        waiting += (waiting.empty() ? "" : ",") + std::to_string(i);
      }
    }
    if (!waiting.empty()) {
      ROS_INFO_THROTTLE(5.0, "[trigger_all_ready] waiting for drone ids: %s", waiting.c_str());
    }
  }

  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  ros::Publisher pub_;
  ros::Timer status_timer_;
  std::vector<ros::Subscriber> subs_;
  std::vector<ros::Time> stamps_;
  std::vector<geometry_msgs::PoseStamped> poses_;
  int num_drones_ = 3;
  std::string input_topic_prefix_;
  std::string input_topic_suffix_;
  std::string output_topic_;
  double max_trigger_age_ = 30.0;
  bool publish_once_ = true;
  bool published_ = false;
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "trigger_all_ready_node");
  TriggerAllReady node;
  ros::spin();
  return 0;
}
