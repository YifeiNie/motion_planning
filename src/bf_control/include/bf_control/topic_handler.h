#pragma once
#include <ros/ros.h>
#include <mavros_msgs/RCIn.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include <Eigen/Dense>
#include <std_msgs/Float64MultiArray.h>
#include "quad_msgs/Target.h"

class Odom {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    Eigen::Vector3d position;
    Eigen::Vector3d velocity;
    Eigen::Quaterniond q;

    nav_msgs::Odometry msg;
    ros::Time rcv_stamp;
    double get_current_yaw();
    void feed(nav_msgs::OdometryConstPtr msg);
};

class RC {
public:
    uint16_t roll;
    uint16_t pitch;
    uint16_t yaw;
    uint16_t thrust;
    uint16_t is_offboard;
    uint16_t is_armed;
    uint16_t is_auto_land;
    ros::Time rcv_stamp;
    void feed(mavros_msgs::RCInConstPtr msg);
};

class Imu {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    Eigen::Vector3d linear_acc;
    Eigen::Vector3d angle_vel;
    Eigen::Quaterniond q;

    sensor_msgs::Imu msg;
    ros::Time rcv_stamp;
    double get_current_yaw();
    void feed(sensor_msgs::ImuConstPtr msg);
};



class Topic_handler {
public:
    Odom odom;
    RC rc;
    Imu imu;
    bool debug_flag;

    std::vector<std_msgs::Float64MultiArray> pos_target;
    std::vector<std_msgs::Float64MultiArray> vel_target;
    std::vector<std_msgs::Float64MultiArray> acc_target;
    std::vector<std_msgs::Float64MultiArray> yaw_target;
    bool is_traj_safe;
    bool has_target;
    double traj_size;
    int traj_idx_iter;

    ros::Timer exec_target;
    ros::Subscriber odom_sub;
    ros::Subscriber rc_sub;
    ros::Subscriber imu_sub;
    ros::Subscriber target_sub;

    ros::Publisher mav_cmd_pub; 
    bool is_rc_received(const ros::Time &now_time);
    bool is_odom_received(const ros::Time &now_time);
    bool is_imu_received(const ros::Time &now_time);
    void rcvTargetCallback(quad_msgs::TargetConstPtr msg);

    void init(ros::NodeHandle& nh);
};

