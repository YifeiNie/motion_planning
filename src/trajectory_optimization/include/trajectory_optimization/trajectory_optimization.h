#pragma once
#include <iostream>
#include <memory>
#include <ros/ros.h>
#include <Eigen/Eigen>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>

#include "trajectory.h"
#include "visualizer.h"
#include "quad_msgs/Target.h"
#include "quad_msgs/Des_target.h"

    
class Traj_opt{
public:
    std::unique_ptr<Visualizer> visualizer;
    
    bool coeff_ready;
    int order;
    int poly_order;
    int p_num;
    double max_vel, max_acc;
    int axis;

    // 当前状态
    bool has_odom;
    Eigen::VectorXd odom_pos;
    Eigen::VectorXd odom_vel; 
    Eigen::VectorXd odom_acc;

    // 三种形式的多项式系数矩阵，用于不同的模块
    std::vector<Eigen::MatrixXd> P_coef_mat;
    std::vector<Eigen::VectorXd> P_coef_vec;
    Eigen::MatrixX3d coef_mat_vis;

    // 各段轨迹时间分配
    Eigen::VectorXd time;

    ros::Time traj_start_time;

    ros::Subscriber odom_sub;

    void init(ros::NodeHandle &nh);
    int factorial(int x);
    Eigen::VectorXd time_allocation(std::vector<Eigen::Vector3d> &path);  // 时间分配
    bool calCoeffMat(const std::vector<Eigen::MatrixXd> &data);
    std::vector<Eigen::MatrixXd> data_config(std::vector<Eigen::Vector3d> &path);
    Eigen::MatrixX3d resize_coeff(std::vector<Eigen::VectorXd> P_coef_vec);
    Eigen::Vector3d getPos(Eigen::MatrixXd polyCoeff, int k, double t);
    void Visualize(std::vector<Eigen::Vector3d> &path);
    
    void odom_rcv_callback(nav_msgs::OdometryConstPtr msg);        // 用于获取轨迹规划的初始的pos，vel，和acc
    bool optimize(std::vector<Eigen::Vector3d> path_main_point);
    void reset();

private:
    
    Trajectory<7> traj;

};


