#include "trajectory_optimization.h"
#include "cmath"


void Traj_opt::init(ros::NodeHandle &nh)
{
    
}
int Traj_opt::factorial(int x)
{
    int fac = 1;
    for (int i = x; i >= 1; i--) {
        fac = fac * i;
    }
    return fac;
}

Eigen::VectorXd Traj_opt::time_allocation(Eigen::MatrixXd Path)  // 时间分配
{
    Eigen::VectorXd time(Path.rows() - 1);
    Eigen::MatrixXd piece;
    double dist;
    const double t = _Vel / _Acc;
    const double d = 0.5 * _Acc * t * t;
    //使用梯形曲线分配时间
    for(int i=0;i<int(time.size());i++)
    {
    piece= Path.row(i+1)-Path.row(i);
    dist = piece.norm();
        if (dist < d + d) {
            time(i)= 2.0 * sqrt(dist / _Acc);
        }
        else {
            time(i) =2.0 * t + (dist - 2.0 * d) / _Vel;
        }
    }
    return time;
}
/**
 * @brief 求解多项式轨迹的系数
 * @param order 要最小化位置的几阶导数
 * @param data 第二个加数
 * @return 多项式轨迹的系数矩阵容器
 */
std::vector<Eigen::MatrixXd> Traj_opt::traj_gen(int order, 
                            const std::vector<Eigen::MatrixXd> &data,
                            const Eigen::VectorXd &time)
{
    std::vector<Eigen::MatrixXd> P_coef;
    int axis = data.size();
    int time_seg_num = time.size();

    for (int i = 0; i < axis; ++i) {
        if (data[i].rows() != order) {
            std::cout << "\033[31m[ERROR]: Data does not match order\033[0m" << std::endl;
            return {};
        }
        if (data[i].cols() != time_seg_num + 1) {
            std::cout << "\033[31m[ERROR]: Data does not match number of time segment\033[0m" << std::endl;
            return {};
        }
    }
    int poly_order = 2*order - 1;
    int p_num = poly_order + 1;
    int p_num_all = p_num * time_seg_num;
    int M_row = p_num;
    int M_col = p_num;

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

    // 计算选择矩阵，如果group_num相同，则表示该点为上一段的末尾或者下一段的开始，其数值是相同的
    int fixed_var_num = 2 * order + (time_seg_num - 1);
    int free_var_num = (time_seg_num - 1) * (order - 1);
    Eigen::MatrixXd C = Eigen::MatrixXd::Zero(p_num_all, fixed_var_num + free_var_num);
    for (int i = 0; i < p_num_all; ++i) {
        // 初始值映射（已知）
        if (i < order) {        
            C(i, i) = 1;
            continue;
        }
        // 末尾值映射（已知）
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

    // 计算权重矩阵Q
    Eigen::MatrixXd Q_tilde = Eigen::MatrixXd::Zero(p_num_all, p_num_all);
    for (int i = 0; i < time_seg_num; ++i) {
        Eigen::MatrixXd Q = Eigen::MatrixXd::Zero(p_num, p_num);

        for (int j = 0; j < p_num; ++j) {
            for (int k = 0; k < p_num; ++k) {
                if (poly_order - j < order || poly_order - k < order) {
                    continue;
                }
                Q(j, k) = factorial(poly_order - j) / factorial(poly_order - order - j) * 
                          factorial(poly_order - k) / factorial(poly_order - order - k) /
                          (poly_order - j - order + poly_order - k - order + 1) * 
                          pow(time(i), poly_order - j - order + poly_order - k - order + 1);
            }
        }
        int row = i * p_num;
        Q_tilde.block(row, row, p_num, p_num) = Q;
    }

    // 计算合并后的二次型权重R
    Eigen::MatrixXd R = C.transpose() * M_tilde.transpose().inverse() * Q_tilde * M_tilde.inverse() * C;

    // 填充数据向量并求解
    for (int i = 0; i < axis; ++i) {
        Eigen::VectorXd d_fixed(fixed_var_num);
        Eigen::VectorXd d_free(free_var_num);
        d_fixed.head(order) = data[i].col(0);
        d_fixed.segment(order, time_seg_num - 1) = data[i].row(0).segment(1, time_seg_num - 1).transpose();
        d_fixed.tail(order) = data[i].col(time_seg_num);

        // 求解
        Eigen::MatrixXd R_PP = R.block(fixed_var_num, fixed_var_num,
                                free_var_num, free_var_num );

        Eigen::MatrixXd R_FP = R.block(0, fixed_var_num, fixed_var_num,
                                free_var_num);

        d_free = -R_PP.inverse() * R_FP.transpose() * d_fixed;
        Eigen::VectorXd d(d_fixed.size() + d_free.size());
        d.head(d_fixed.size()) = d_fixed;
        d.tail(d_free.size()) = d_free;
        Eigen::VectorXd P = M_tilde.inverse() * (C * d);

        Eigen::MatrixXd temp_mat(time_seg_num, p_num);
        for (int j = 0; j < time_seg_num; ++j) {
            temp_mat.row(j) = P.segment(j * p_num, p_num).transpose();
        }

        P_coef.emplace_back(temp_mat);

    }
    return P_coef;

}





