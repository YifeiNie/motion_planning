#include "trajectory_optimization.h"

#include "cmath"


void Traj_opt::init(ros::NodeHandle &nh)
{
    visualizer = std::make_unique<Visualizer>(nh);
    max_vel = nh.param("Opt/max_vel", 0.5);
    max_acc = nh.param("Opt/max_acc", 0.5);
    order = nh.param("Opt/order", ORDER);
    axis = nh.param("Opt/axis", 3);

    poly_order = 2 * order - 1;
    p_num = poly_order + 1;
    traj_start_time= ros::TIME_MAX;

    odom_pos = Eigen::VectorXd::Zero(axis);
    odom_pos(2) = 2;
    odom_vel = Eigen::VectorXd::Zero(axis);
    odom_acc = Eigen::VectorXd::Zero(axis);

    odom_sub = nh.subscribe<nav_msgs::Odometry>("/vins_fusion/odometry", 1, boost::bind(&Traj_opt::odom_rcv_callback, this, _1));  
}

void Traj_opt::reset() {
    P_coef_vec.clear();
    P_coef_mat.clear();
}

int Traj_opt::factorial(int x)
{
    int fac = 1;
    for (int i = x; i >= 1; i--) {
        fac = fac * i;
    }
    return fac;
}

Eigen::VectorXd Traj_opt::time_allocation(std::vector<Eigen::Vector3d> &path)  // 时间分配
{

    int rows = path.size();
    int cols = path[0].size();
    Eigen::MatrixXd Path(rows, cols);
    for (int i = 0; i < rows; ++i) {
        Path.row(i) = path[i].transpose(); // 转置成行向量并赋值
    }

    Eigen::VectorXd time(Path.rows() - 1);
    Eigen::MatrixXd piece;
    double dist;
    const double t = max_vel / max_acc;
    const double d = 0.5 * max_acc * t * t;
    //使用梯形曲线分配时间
    for(int i = 0; i < int(time.size()); i++)
    {
    piece = Path.row(i + 1) - Path.row(i);
    dist = piece.norm();
        if (dist < d + d) {
            time(i)= 2.0 * sqrt(dist / max_acc);
        }
        else {
            time(i) = 2.0 * t + (dist - 2.0 * d) / max_vel;
        }
    }
    return time;
}

std::vector<Eigen::MatrixXd> Traj_opt::data_config(std::vector<Eigen::Vector3d> &path)
{
    int seg_num = path.size() - 1;
    std::vector<Eigen::MatrixXd> data;

    Eigen::Matrix3Xd mat(3, path.size());
    for (size_t i = 0; i < path.size(); ++i)
    {
        mat.col(i) = path[i];
    }

    for (int i = 0; i < axis; ++i) {
        Eigen::MatrixXd temp = Eigen::MatrixXd::Zero(order, seg_num + 1);
        temp.row(0) = mat.row(i);       // pos
        temp(1, 1) = odom_vel(i);      // vel
        temp(2, 1) = odom_acc(i);      // acc
        data.emplace_back(temp);
    }

    return data;
}


bool Traj_opt::calCoeffMat(const std::vector<Eigen::MatrixXd> &data)
{
    coeff_ready = false;
    int time_seg_num = time.size();

    for (int i = 0; i < axis; ++i) {
        if (data[i].rows() != order) {
            std::cout << "\033[31m[ERROR]: Data does not match order\033[0m" << std::endl;
            return false;
        }
        if (data[i].cols() != time_seg_num + 1) {
            std::cout << "\033[31m[ERROR]: Data does not match number of time segment\033[0m" << std::endl;
            return false;
        }
    }
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

        P_coef_mat.push_back(temp_mat);
        P_coef_vec.push_back(P);
    }
    coef_mat_vis = resize_coeff(P_coef_vec);
    coeff_ready = true;
    return true;
}

// 用于格式转化，从容器+向量+从高到低幂次排列 -->> 矩阵+每一列表示一个维度+从低到高幂次排列
// 在使用getPos函数的时候，传参必须经过该格式转化
Eigen::MatrixX3d Traj_opt::resize_coeff(std::vector<Eigen::VectorXd> P_coef_vec)
{
    int num_segments = P_coef_vec[0].size() / p_num;
    Eigen::MatrixX3d coefficientMatrix = Eigen::MatrixXd::Zero(p_num * num_segments, 3);

    for (int i = 0; i < num_segments; i++) {
        Eigen::VectorXd segment = P_coef_vec[0].segment(i * p_num, p_num);
        segment.reverseInPlace();
        coefficientMatrix.col(0).segment(i * p_num, p_num) = segment;
    }
    for (int i = 0; i < num_segments; i++) {
        Eigen::VectorXd segment = P_coef_vec[1].segment(i * p_num, p_num);
        segment.reverseInPlace();
        coefficientMatrix.col(1).segment(i * p_num, p_num) = segment;
    }
    for (int i = 0; i < num_segments; i++) {
        Eigen::VectorXd segment = P_coef_vec[2].segment(i * p_num, p_num);
        segment.reverseInPlace();
        coefficientMatrix.col(2).segment(i * p_num, p_num) = segment;
    }
    return coefficientMatrix;

}

void Traj_opt::Visualize(std::vector<Eigen::Vector3d> &path)
{
    int num_segments = P_coef_vec[0].size() / p_num;

    traj.clear();
    traj.reserve(num_segments);
    for (int i = 0; i < num_segments; i++)
    {
        traj.emplace_back(time(i), coef_mat_vis.block<2*ORDER, 3>(2 * ORDER * i, 0).transpose().rowwise().reverse());
    }

    int cols = path.size();
    int rows = path[0].size();
    Eigen::MatrixXd Path(rows, cols);
    for (int i = 0; i < cols; ++i) {
        Path.col(i) = path[i].transpose();
    }
    // std::cout << Path << std::endl;
    // std::cout << coef_mat_vis << std::endl;
    visualizer->visualize(traj, Path);
}


Eigen::Vector3d Traj_opt::getPos(Eigen::MatrixXd polyCoeff, int k, double t) {
    
    Eigen::Vector3d ret;
    for (int dim = 0; dim < 3; dim++) {
        Eigen::VectorXd coeff = (polyCoeff.col(dim)).segment(k * p_num, p_num);
        Eigen::VectorXd time = Eigen::VectorXd::Zero(p_num);

        for (int j = 0; j < p_num; j++) {
            if (j == 0){
                time(j) = 1.0;
            }
            else {
                time(j) = pow(t, j);
            }  
            ret(dim) = coeff.dot(time);
        }
    }
    return ret;
}

void Traj_opt::odom_rcv_callback(nav_msgs::OdometryConstPtr msg)
{
    odom_pos(0) = msg->pose.pose.position.x;
    odom_pos(1) = msg->pose.pose.position.y;
    odom_pos(2) = msg->pose.pose.position.z;

    odom_vel(0) = msg->twist.twist.linear.x;
    odom_vel(1) = msg->twist.twist.linear.y;
    odom_vel(2) = msg->twist.twist.linear.z;

    has_odom = true;    
}

bool Traj_opt::optimize(std::vector<Eigen::Vector3d> path_main_point)
{
    if (path_main_point.size() == 1) {
        std::cout << "[trajectory_optimization] Current and target are in the same grid, invalid target!" << std::endl;
        return false;
    }
    reset();
    time = time_allocation(path_main_point);
    std::vector<Eigen::MatrixXd> data = data_config(path_main_point);    
    bool ret = calCoeffMat(data);
    return ret;
}

