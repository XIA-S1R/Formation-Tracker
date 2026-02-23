#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <visualization_msgs/Marker.h>
#include <tf/transform_datatypes.h>
#include <cmath>  // for atan2, M_PI
#include "quadrotor_sim/quadrotor_dynamics.hpp"

class QuadrotorSimNode {
public:
    QuadrotorSimNode() {
        // 订阅轨迹规划输出：期望位置 + 偏航
        pose_sub = nh.subscribe("desired_pose", 1, &QuadrotorSimNode::poseCallback, this);

        // 发布状态
        odom_pub = nh.advertise<nav_msgs::Odometry>("odom", 1);
        imu_pub = nh.advertise<sensor_msgs::Imu>("imu", 1);
        marker_pub = nh.advertise<visualization_msgs::Marker>("visualization_marker", 1);

        // 初始化期望状态
        desired_pose.position.x = 0;
        desired_pose.position.y = 0;
        desired_pose.position.z = 0;
        desired_pose.orientation = tf::createQuaternionMsgFromYaw(0);

        // 初始化当前加速度
        current_accel.linear.x = 0;
        current_accel.linear.y = 0;
        current_accel.linear.z = 0;
        current_accel.angular.x = 0;
        current_accel.angular.y = 0;
        current_accel.angular.z = 0;
    }

    void run() {
        ros::Rate rate(100);  // 100 Hz 仿真频率
        while (ros::ok()) {
            ros::spinOnce();
            // 推进动力学：基于期望和当前状态解算运动
            quad.update(desired_pose, quad.pose, quad.twist, current_accel, 0.01);
            publish();
            rate.sleep();
        }
    }


private:
    void poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
        desired_pose = msg->pose;
    }

    void publish() {
        ros::Time now = ros::Time::now();
        // Odometry：发布当前状态
        nav_msgs::Odometry odom;
        odom.header.stamp = now;
        odom.header.frame_id = "world";
        odom.pose.pose = quad.pose;
        odom.twist.twist = quad.twist;
        odom_pub.publish(odom);

        // IMU：发布当前加速度
        sensor_msgs::Imu imu;
        imu.header = odom.header;
        imu.orientation = quad.pose.orientation;
        imu.angular_velocity = quad.twist.angular;
        imu.linear_acceleration = current_accel.linear;
        imu_pub.publish(imu);

        // Marker：可视化无人机位置
        visualization_msgs::Marker marker;
        marker.header = odom.header;
        marker.id = 0;
        marker.type = visualization_msgs::Marker::CUBE;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose = quad.pose;
        marker.scale.x = 0.5;
        marker.scale.y = 0.5;
        marker.scale.z = 0.1;
        marker.color.r = 1.0;
        marker.color.g = 0.0;
        marker.color.b = 0.0;
        marker.color.a = 1.0;
        marker_pub.publish(marker);
    }

    ros::NodeHandle nh;
    ros::Subscriber pose_sub;
    ros::Publisher odom_pub, imu_pub, marker_pub;
    QuadrotorDynamics quad;
    geometry_msgs::Pose desired_pose;
    geometry_msgs::Accel current_accel;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "quadrotor_sim_node");
    QuadrotorSimNode node;
    node.run();
    return 0;
}