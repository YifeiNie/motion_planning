#pragma once 
#include <Eigen/Dense>
#include <topic_handler.h>
#include <queue>

#define CTRL_FREQUENCY 100
#define RAD2DEG 57.29577951308232
#define DEG2RAD 0.017453292519943
#define G 9.80665

struct Gain{
    // 双环参数
    double Kp_x, Kp_y, Kp_z;
    double Ki_x, Ki_y, Ki_z;
    double Kd_x, Kd_y, Kd_z;

    double Kp_vx, Kp_vy, Kp_vz;
    double Ki_vx, Ki_vy, Ki_vz;
    double Kd_vx, Kd_vy, Kd_vz;
    double K_yaw;

    // 单环参数
    double Kp_x_single, Kp_y_single, Kp_z_single;
    double Kv_x_single, Kv_y_single, Kv_z_single;
    Eigen::Vector3d Kp_single, Kv_single;      
};

struct Lp3_filter {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    Eigen::Vector3d state;
    Eigen::Vector3d state1;
    Eigen::Vector3d state2;
    double k;
};

class PID_controller {
private:
    std::queue<std::pair<ros::Time, double>> timed_thrust;
    const double rho = 0.998; // do not change
    double thr2acc;
    double P;
public:
    double hover_height;

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW                
    double desire_yaw;                         // 期望偏航角
    Eigen::Vector3d desire_position;           // 期望位置
    Eigen::Vector3d desire_velocity;           // 期望速度（储存外环的输出）
    Eigen::Vector3d desire_accelerate;         // 期望加速度
    Eigen::Vector3d desire_jerk;               // 期望加加速度

    Eigen::Vector3d current_position;
    Eigen::Vector3d current_velocity;
    double lp3_k;                               // 低通滤波器参数    
    
    Eigen::Vector3d position_error_sum;
    Eigen::Vector3d velocity_error_sum;
    double balance_thrust;                   // 平衡时的油门
    double position_x_i_bound, position_y_i_bound, position_z_i_bound;     
    double velocity_x_i_bound, velocity_y_i_bound, velocity_z_i_bound;

    double thrust_bound;                        // 油门限幅
    double x_bound, y_bound;                    // 姿态控制限幅
    Eigen::Vector3d g_vector;

    Gain gain;                                  // PID参数结构体
    mavros_msgs::AttitudeTarget att_cmd_msg;    // 发到飞控的控制指令

    void init(ros::NodeHandle& nh);
    void reset();
    void outer_position_loop(Topic_handler& th);
    void inner_velocity_loop(Topic_handler& th);
    void single_accelerate_loop(Topic_handler& th);
    void setDesire(double x, double y, double z, double yaw);
    double get_thrust_out();
    bool estimate_thrust_model();
    double limit(double ub, double lb, double value);
    Eigen::Vector3d ve2vb(Eigen::Vector3d input, double yaw); // 全局坐标系到机体坐标系的转换
    Eigen::Vector3d lp3_filter(Lp3_filter lpf3, Eigen::Vector3d input);
};