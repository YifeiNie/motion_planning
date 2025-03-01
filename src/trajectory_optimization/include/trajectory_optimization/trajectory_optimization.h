#pragma once
#include <ros/ros.h>
#include <Eigen/Eigen>
#include "A_star.h"


class Traj_opt{
public:
    void init(ros::NodeHandle &nh);
    int factorial(int x);
    Eigen::VectorXd time_allocation(Eigen::MatrixXd Path);  // 时间分配
    Eigen::MatrixXd traj_gen(int order, int axis, 
                             const Eigen::MatrixXd &path,
                             const Eigen::MatrixXd &vel,
                             const Eigen::MatrixXd &acc,
                             const Eigen::VectorXd &time);
    



private:



};


