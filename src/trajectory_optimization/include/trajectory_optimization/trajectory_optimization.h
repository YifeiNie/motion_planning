#pragma once
#include <iostream>
#include <ros/ros.h>
#include <Eigen/Eigen>
#include "A_star.h"


class Traj_opt{
public:
    void init(ros::NodeHandle &nh);
    int factorial(int x);
    Eigen::VectorXd time_allocation(Eigen::MatrixXd Path);  // 时间分配
    std::vector<Eigen::MatrixXd> traj_gen(int order, 
                             const std::vector<Eigen::MatrixXd> &data,
                             const Eigen::VectorXd &time);
    



private:



};


