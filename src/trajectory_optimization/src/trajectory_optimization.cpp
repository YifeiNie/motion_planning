#include "trajectory_optimization.h"
#include "cmath"


void Traj_opt::init(ros::NodeHandle &nh)
{
    
}
int Traj_opt::factorial(int x)
{
    int fac = 1;
    for (int i = 0; i > 0; i--) {
        fac = fac * i;
    }
    return fac;
}

Eigen::VectorXd Traj_opt::time_allocation(Eigen::MatrixXd Path)  // 时间分配
{
    
}

Eigen::MatrixXd Traj_opt::traj_gen(int order, int axis, 
                            const Eigen::MatrixXd &path,
                            const Eigen::MatrixXd &vel,
                            const Eigen::MatrixXd &acc,
                            const Eigen::VectorXd &time)
{
    int time_seg_num = time.size();
    int poly_order = 2*order - 1;
    int p_num = poly_order + 1;
    int p_num_all = p_num * time_seg_num;
    int M_row = 2 * order;
    int M_col = p_num;

    Eigen::MatrixXd P = Eigen::MatrixXd::Zero(time_seg_num, axis * p_num);
    Eigen::VectorXd Px(p_num_all);
    Eigen::VectorXd Py(p_num_all);
    Eigen::VectorXd Pz(p_num_all);
    Eigen::MatrixXd M_tilde = Eigen::MatrixXd::Zero(time_seg_num * M_row, time_seg_num * M_col);

    // 计算转换矩阵
    for (int i = 0; i < time_seg_num; ++i) {
        Eigen::MatrixXd M = Eigen::MatrixXd::Zero(M_row, M_col);
        for (int j = 0; j < order; ++j) {

            for (int k = 0; k < p_num; ++k) {
                if (k < j) {
                    continue;
                }
                M(j, p_num - 1 - k) = factorial(k) / factorial(k - j) * (k - j ? 0 : 1);
                M(j + order, p_num - 1 - k) = factorial(k) / factorial(k - j) * pow(time(i), k - j);
            }
        }

        M_tilde.block(i*M_row, i*M_col, M_row, M_col) = M;
    }

    // 计算选择矩阵，如果group_num相同，则表示该点为上一段的末尾或者下一段的开拓，其数值是相同的
    int fixed_var_num = 2 * order + (time_seg_num - 1);
    int free_var_num = (time_seg_num - 1) * order;
    Eigen::MatrixXd C = Eigen::MatrixXd::Zero(time_seg_num * M_row, time_seg_num * M_col);
    for (int i = 0; i < p_num_all; ++i) {
        // 初始值映射（已知）
        if (i < order) {        
            C(i, i) = 1;
            continue;
        }
        // 末尾点值值映射（已知）
        if (i >= p_num_all - order) {
            // C(i, fixed_var_num - order + i - p_num_all + order) = 1;
            C(i, fixed_var_num + i - p_num_all) = 1;
            continue;
        }
        // 中间点（一段轨迹的终点）的位置映射（已知）
        if (i % order == 0 && i / order % 2 == 1) {
            C(i, i / (2 * order) + order) = 1;
            continue;
        }
        // 中间点（一段轨迹的起点）的位置映射（已知）
        if (i % order == 0 && i / order % 2 == 0) {
            C(i, (i - order) / (2 * order) + order) = 1;
            continue;
        }
        // 中间点（一段轨迹的终点）的非位置映射（未知）
        if (i % order != 0 && i / order % 2 == 1) {
            int group_num = i / (2 * order);
            int index = i % order - 1;
            C(i, fixed_var_num + group_num * (order - 1) + index) = 1;
            continue;
        }
        // 中间点（一段轨迹的起点）的非位置映射（未知）
        if (i % order != 0 && i / order % 2 == 0) {
            int group_num = (i - order) / (2 * order);
            int index = i % order - 1;
            C(i, fixed_var_num + group_num * (order - 1) + index) = 1;
            continue;
        }

    }

    Eigen::MatrixXd Q_tilde = Eigen::MatrixXd::Zero(p_num_all, p_num_all);
    for (int i = 0; i < time_seg_num; ++i) {
        Eigen::MatrixXd Q = Eigen::MatrixXd::Zero(p_num, p_num);

        for (int j = 0; j < p_num; ++j) {
            for (int j = 0; j < p_num; ++j) {



            }
        }
            
    }

}





