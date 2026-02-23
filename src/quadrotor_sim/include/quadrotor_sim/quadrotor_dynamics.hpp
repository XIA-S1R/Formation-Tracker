#ifndef QUADROTOR_DYNAMICS_HPP
#define QUADROTOR_DYNAMICS_HPP

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Accel.h>
#include <tf/transform_datatypes.h>
#include <Eigen/Dense>

using Vector3 = Eigen::Vector3d;

class QuadrotorDynamics {
public:
    QuadrotorDynamics() {
        // 物理参数
        mass = 1.0;
        g = 9.81;

        // 惯性矩阵 (对角矩阵)
        I = Eigen::Matrix3d::Zero();
        I(0,0) = 0.01;  // Ix
        I(1,1) = 0.01;  // Iy
        I(2,2) = 0.02;  // Iz

        // 位置控制增益 Kp
        Kp = 1.0;

        // 姿态控制增益KR和角速度控制增益Kw
        KR = 1.0;
        Kw = 1.0;

        // 重力向量
        g_vector = Eigen::Vector3d(0, 0, -g);
    }

    // 更新动力学：基于期望位置/偏航 + 当前状态，解算运动
    void update(const geometry_msgs::Pose& desired_pose, 
                geometry_msgs::Pose& current_pose, 
                geometry_msgs::Twist& current_twist, 
                geometry_msgs::Accel& current_accel, 
                double dt) {
        // 位置误差向量
        Vector3 pos_error(desired_pose.position.x - current_pose.position.x,
                          desired_pose.position.y - current_pose.position.y,
                          desired_pose.position.z - current_pose.position.z);

        // 所需推力加速度向量
        Vector3 a_T = Kp * pos_error + g_vector;

        // 更新当前加速度
        current_accel.linear.x = a_T[0];
        current_accel.linear.y = a_T[1];
        current_accel.linear.z = a_T[2];

        // 积分加速度到速度
        current_twist.linear.x += a_T[0] * dt;
        current_twist.linear.y += a_T[1] * dt;
        current_twist.linear.z += a_T[2] * dt;

        // 积分速度到位置
        current_pose.position.x += current_twist.linear.x * dt;
        current_pose.position.y += current_twist.linear.y * dt;
        current_pose.position.z += current_twist.linear.z * dt;

        // 构建期望姿态 R_desired
        double norm_a_T = norm(a_T);
        if (norm_a_T > 1e-6) {  // 避免除零
            // 期望机体z轴方向 b3
            Vector3 b3 = normalize(a_T);

            // 期望偏航角
            double desired_yaw = tf::getYaw(desired_pose.orientation);

            // 水平面参考向量b1_c
            Vector3 b1_c(cos(desired_yaw), sin(desired_yaw), 0);

            // 期望机体y轴方向 b2
            Vector3 b2 = cross(b3, b1_c);
            b2 = normalize(b2);
            if (b2.norm() < 1e-6) {
                // 如果 b2 为零，使用默认
                b2 = Eigen::Vector3d(-sin(desired_yaw), cos(desired_yaw), 0);
            }

            // 期望机体x轴方向 b1
            Vector3 b1 = cross(b2, b3);

            // 期望旋转矩阵 R_desired
            Eigen::Matrix3d R_desired;
            R_desired.col(0) = b1;
            R_desired.col(1) = b2;
            R_desired.col(2) = b3;

            // 当前旋转矩阵 R
            Eigen::Quaterniond q_current(current_pose.orientation.w, current_pose.orientation.x, current_pose.orientation.y, current_pose.orientation.z);
            Eigen::Matrix3d R = q_current.toRotationMatrix();

            // 计算姿态误差 Rotation_error
            Eigen::Matrix3d M = R_desired.transpose() * R - R.transpose() * R_desired;
            Vector3 Rotation_error;
            Rotation_error[0] = -0.5 * (M(2,1) - M(1,2));
            Rotation_error[1] = -0.5 * (M(0,2) - M(2,0));
            Rotation_error[2] = -0.5 * (M(1,0) - M(0,1));

            // 当前角速度向量
            Vector3 w(current_twist.angular.x, current_twist.angular.y, current_twist.angular.z);

            // 需要的角加速度 angular_accel
            Vector3 angular_accel = KR * Rotation_error + Kw * (-w) + I.inverse() * w.cross(I * w);

            // 设置角加速度
            current_accel.angular.x = angular_accel[0];
            current_accel.angular.y = angular_accel[1];
            current_accel.angular.z = angular_accel[2];
        } else {
            // 如果 a_T 为零，直接设置 yaw
            double desired_yaw = tf::getYaw(desired_pose.orientation);
            current_pose.orientation = tf::createQuaternionMsgFromYaw(desired_yaw);

            // 姿态误差为0
            current_accel.angular.x = 0;
            current_accel.angular.y = 0;
            current_accel.angular.z = 0;
        }

        // 积分角加速度到角速度
        current_twist.angular.x += current_accel.angular.x * dt;
        current_twist.angular.y += current_accel.angular.y * dt;
        current_twist.angular.z += current_accel.angular.z * dt;

        // 积分角速度到姿态
        Eigen::Quaterniond q_current(current_pose.orientation.w, current_pose.orientation.x, current_pose.orientation.y, current_pose.orientation.z);
        Eigen::Vector3d omega(current_twist.angular.x, current_twist.angular.y, current_twist.angular.z);
        Eigen::Quaterniond q_dot = 0.5 * q_current * Eigen::Quaterniond(0, omega[0], omega[1], omega[2]);
        q_current.coeffs() += q_dot.coeffs() * dt;
        q_current.normalize();
        current_pose.orientation.w = q_current.w();
        current_pose.orientation.x = q_current.x();
        current_pose.orientation.y = q_current.y();
        current_pose.orientation.z = q_current.z();
    }

    double mass, g;
    double K;
    Eigen::Matrix3d I;
    double KR, Kw;
    Vector3 g_vector;
};

#endif