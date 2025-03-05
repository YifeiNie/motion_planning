#pragma once
#include <iostream>
#include <memory>
#include <ros/ros.h>
#include <Eigen/Eigen>
#include "trajectory.h"
#include "visualizer.h"


class Traj_opt{
public:
    int order;
    int poly_order;
    int p_num;
    double max_vel, max_acc;

    void init(ros::NodeHandle &nh);
    int factorial(int x);
    Eigen::VectorXd time_allocation(std::vector<Eigen::Vector3d> &path);  // 时间分配
    std::vector<Eigen::VectorXd> traj_gen(const std::vector<Eigen::MatrixXd> &data, const Eigen::VectorXd &time);
    std::vector<Eigen::MatrixXd> data_config(std::vector<Eigen::Vector3d> &path);
    Eigen::MatrixX3d resize_coeff(std::vector<Eigen::VectorXd> P_coef_vec);
    Eigen::Vector3d getPos(Eigen::MatrixXd polyCoeff, int k, double t);
    void Visualize(std::vector<Eigen::VectorXd> P_coef_vec, 
                    std::vector<Eigen::Vector3d> &path, 
                    Eigen::VectorXd &time);



private:
    std::unique_ptr<Visualizer> visualizer;
    Trajectory<5> traj;

};


