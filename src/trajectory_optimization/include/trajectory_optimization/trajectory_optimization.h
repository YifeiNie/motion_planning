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

void FSM_task(const ros::TimerEvent &event);

enum STATE {
    INIT,
    WAIT_TARGET,
    GEN_NEW_TRAJ,
    EXEC_TRAJ,
    REPLAN_TRAJ
};

    
class Traj_opt{
public:

    bool coeff_ready;
    int order;
    int poly_order;
    int p_num;
    double max_vel, max_acc;
    int axis;
    STATE state;
    std::string state_str[5] = {"INIT", "WAIT_TARGET", "GEN_NEW_TRAJ", "EXEC_TRAJ", "REPLAN_TRAJ"};

    // 当前状态
    bool has_odom;
    bool has_target;
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
    ros::Timer FSM_task_timer;

    void init(ros::NodeHandle &nh);
    int factorial(int x);
    Eigen::VectorXd time_allocation(std::vector<Eigen::Vector3d> &path);  // 时间分配
    bool traj_gen(const std::vector<Eigen::MatrixXd> &data);
    std::vector<Eigen::MatrixXd> data_config(std::vector<Eigen::Vector3d> &path);
    Eigen::MatrixX3d resize_coeff(std::vector<Eigen::VectorXd> P_coef_vec);
    Eigen::Vector3d getPos(Eigen::MatrixXd polyCoeff, int k, double t);
    void Visualize(std::vector<Eigen::Vector3d> &path);
    void change_state(STATE new_state);
    void print_state();                                     // 定期执行
    void odom_rcv_callback(nav_msgs::OdometryConstPtr msg);        // 用于获取轨迹规划的初始的pos，vel，和acc
    void optimize(std::vector<Eigen::Vector3d> path_main_point);
    void reset();

private:
    std::unique_ptr<Visualizer> visualizer;
    Trajectory<5> traj;

};


