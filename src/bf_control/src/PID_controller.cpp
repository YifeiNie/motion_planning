
#include <mavros_msgs/AttitudeTarget.h>
#include <ros/ros.h>
#include "PID_controller.h"
#include "topic_handler.h"

void PID_controller::init(ros::NodeHandle &nh){
    // 如果是全局参数需要在最前面夹斜杠，但是我这是四有参数，所以不加
    nh.getParam("PID/Kp_x", gain.Kp_x);
    nh.getParam("PID/Kp_y", gain.Kp_y);
    nh.getParam("PID/Kp_z", gain.Kp_z);
    nh.getParam("PID/Ki_x", gain.Ki_x);
    nh.getParam("PID/Ki_y", gain.Ki_y);
    nh.getParam("PID/Ki_z", gain.Ki_z);
    nh.getParam("PID/Kd_x", gain.Kd_x);
    nh.getParam("PID/Kd_y", gain.Kd_y);
    nh.getParam("PID/Kd_z", gain.Kd_z);
    nh.getParam("PID/Kp_vx", gain.Kp_vx);
    nh.getParam("PID/Kp_vy", gain.Kp_vy);
    nh.getParam("PID/Kp_vz", gain.Kp_vz);
    nh.getParam("PID/Ki_vx", gain.Ki_vx);
    nh.getParam("PID/Ki_vy", gain.Ki_vy);
    nh.getParam("PID/Ki_vz", gain.Ki_vz);
    nh.getParam("PID/Kd_vx", gain.Kd_vx);
    nh.getParam("PID/Kd_vy", gain.Kd_vy);
    nh.getParam("PID/Kd_vz", gain.Kd_vz);
    nh.getParam("PID/K_yaw", gain.K_yaw);
    nh.getParam("PID/position_x_i_bound", position_x_i_bound);
    nh.getParam("PID/position_y_i_bound", position_y_i_bound);
    nh.getParam("PID/position_z_i_bound", position_z_i_bound);
    nh.getParam("PID/velocity_x_i_bound", velocity_x_i_bound);
    nh.getParam("PID/velocity_y_i_bound", velocity_y_i_bound);
    nh.getParam("PID/velocity_z_i_bound", velocity_z_i_bound);
    nh.getParam("PID/x_bound", x_bound);
    nh.getParam("PID/y_bound", y_bound);
    nh.getParam("PID/thrust_bound", thrust_bound);
    nh.getParam("Lowpass_filter/lp3_k", lp3_k);
    nh.getParam("Drone/balance_thrust", balance_thrust);

    nh.getParam("PID_single/Kpx_single", gain.Kp_x_single);
    nh.getParam("PID_single/Kpy_single", gain.Kp_y_single);
    nh.getParam("PID_single/Kpz_single", gain.Kp_z_single);
    nh.getParam("PID_single/Kvx_single", gain.Kv_x_single);
    nh.getParam("PID_single/Kvy_single", gain.Kv_y_single);
    nh.getParam("PID_single/Kvz_single", gain.Kv_z_single);
    // nh.getParam("Drone/g", balance_thrust);
    gain.Kp_single << gain.Kp_x_single, gain.Kp_y_single, gain.Kp_z_single;
    gain.Kv_single << gain.Kv_x_single, gain.Kv_y_single, gain.Kv_z_single;
    desire_position << 0, 0, 0;
    current_position << 0, 0, 0;
    current_velocity << 0, 0, 0;
    position_error_sum << 0, 0, 0;
    velocity_error_sum << 0, 0, 0;
    g_vector << 0, 0, G;

    hover_height = nh.param("PID/hover_height", 0.4);
}

void PID_controller::reset(){
    this->velocity_error_sum.setZero();
    this->position_error_sum.setZero();
    this->desire_position.setZero();
    this->desire_velocity.setZero();
    this->desire_accelerate.setZero();
    this->desire_jerk.setZero();
    this->desire_yaw = 0;

    att_cmd_msg.body_rate.x = 0;
    att_cmd_msg.body_rate.y = 0;
    att_cmd_msg.body_rate.z = 0;
    att_cmd_msg.thrust = 0;
}

double PID_controller::limit(double ub, double lb, double value) {
    if(value > ub)
        value = ub;
    if(value < lb)
        value = lb;
    return value;
}

// 3阶低通滤波器
Eigen::Vector3d PID_controller::lp3_filter(Lp3_filter lp3_filter, Eigen::Vector3d input) {
    lp3_filter.state1 = lp3_filter.state1 + lp3_filter.k * (input - lp3_filter.state1);
    lp3_filter.state2 = lp3_filter.state2 + lp3_filter.k * (lp3_filter.state1 - lp3_filter.state2);
    lp3_filter.state = lp3_filter.state + lp3_filter.k * (lp3_filter.state2 - lp3_filter.state);
    return lp3_filter.state;
}

// 世界坐标系转机体坐标系
Eigen::Vector3d PID_controller::
ve2vb(Eigen::Vector3d input, double yaw){
    Eigen::Matrix3d R;
    R << cos(yaw), sin(yaw), 0,
         -sin(yaw), cos(yaw), 0,
         0, 0, 1;
    return R * input;
}

// 外环位置控制
void PID_controller::outer_position_loop(Topic_handler& th) {
    // 位置误差 = 期望位置 - 当前位置
    Eigen::Vector3d position_error = desire_position - th.odom.position;
    // ROS_INFO("x cmd is %f", position_error[0]);
    // ROS_INFO("y cmd is %f", position_error[1]);
    // ROS_INFO("thrust cmd is %f", position_error[2]);
    position_error_sum += position_error / CTRL_FREQUENCY;
    
    double temp_position_i_x = gain.Ki_x * position_error_sum[0];
    double temp_position_i_y = gain.Ki_y * position_error_sum[1];
    double temp_position_i_z = gain.Ki_z * position_error_sum[2];
    temp_position_i_x = limit(position_x_i_bound, -position_x_i_bound, temp_position_i_x);
    temp_position_i_y = limit(position_y_i_bound, -position_y_i_bound, temp_position_i_y);
    temp_position_i_z = limit(position_z_i_bound, -position_z_i_bound, temp_position_i_z);

    desire_velocity[0] = gain.Kp_x * position_error[0] + temp_position_i_x;
    desire_velocity[1] = gain.Kp_y * position_error[1] + temp_position_i_y;
    desire_velocity[2] = gain.Kp_z * position_error[2] + temp_position_i_z;
}

// 内环姿态控制
void PID_controller::inner_velocity_loop(Topic_handler& th){

    // 获取当前yaw角用于将全局坐标系的线速度转换为机体坐标系的线速度
    double current_yaw_body = th.odom.get_current_yaw();
    desire_velocity = ve2vb(desire_velocity, current_yaw_body);
    // 速度误差 = 期望速度 - 当前速度
    Eigen::Vector3d velocity_error = desire_velocity - th.odom.velocity;

    // yaw的控制
    double yaw_error = desire_yaw - current_yaw_body;
    velocity_error_sum += velocity_error / CTRL_FREQUENCY;

    double temp_velocity_i_x = gain.Ki_vx * velocity_error_sum[0];
    double temp_velocity_i_y = gain.Ki_vy * velocity_error_sum[1];
    double temp_velocity_i_z = gain.Ki_vz * velocity_error_sum[2];
    temp_velocity_i_x = limit(velocity_x_i_bound, -velocity_x_i_bound, temp_velocity_i_x);
    temp_velocity_i_y = limit(velocity_y_i_bound, -velocity_y_i_bound, temp_velocity_i_y);
    temp_velocity_i_z = limit(velocity_z_i_bound, -velocity_z_i_bound, temp_velocity_i_z);

    double temp_x_out = (gain.Kp_vx * velocity_error[0] + temp_velocity_i_x);
    double temp_y_out = (gain.Kp_vy * velocity_error[1] + temp_velocity_i_y);
    double temp_thrust_out = balance_thrust + gain.Kp_vz * velocity_error[2] + temp_velocity_i_z;
    double temp_yaw_out = gain.K_yaw * yaw_error;

    temp_x_out = limit(x_bound, -x_bound, temp_x_out);
    temp_y_out = limit(y_bound, -y_bound, temp_y_out);
    temp_thrust_out = limit(thrust_bound, -thrust_bound, temp_thrust_out);  

    // 设置飞控指令(body_rate的含义由att_cmd_msg.type_mask决定: 4是角度，1是角速度)
    att_cmd_msg.type_mask = 4;
    att_cmd_msg.body_rate.x = -temp_y_out * RAD2DEG;
    att_cmd_msg.body_rate.y = temp_x_out * RAD2DEG;
    //att_cmd_msg.body_rate.z = temp_yaw_out;
    att_cmd_msg.thrust = temp_thrust_out;
    // if(abs(att_cmd_msg.thrust) <= 0.01){
    //     att_cmd_msg.thrust = 0;
    // }    
}

void PID_controller::setDesire(double x, double y, double z, double yaw)
{
    desire_yaw = yaw;                        
    desire_position.x() = x;
    desire_position.y() = y;
    desire_position.z() = z;
}

void PID_controller::single_accelerate_loop(Topic_handler& th){
    // double current_yaw = th.imu.get_current_yaw();
    // double sin = std::sin(current_yaw);
    // double cos = std::cos(current_yaw);
    // Eigen::Vector3d  temp_acc_output = desire_accelerate + 
    //                                gain.Kv_single.asDiagonal() * (desire_velocity - th.odom.velocity) + 
    //                                gain.Kp_single.asDiagonal() * (desire_position - th.odom.position);
    // temp_acc_output += g_vector;
    // double roll_out = (temp_acc_output(0)*sin - temp_acc_output(1)*cos )/G;
    // double pitch_out = (temp_acc_output(0)*cos + temp_acc_output(1)*sin )/G;
    // double yaw_out = this->desire_yaw;
    // double thrust_out = get_thrust_out();
    
    // att_cmd_msg.type_mask = 4;
    // att_cmd_msg.body_rate.x = roll_out * RAD2DEG;
    // att_cmd_msg.body_rate.y = pitch_out * RAD2DEG;
    // att_cmd_msg.body_rate.z = yaw_out;
    // att_cmd_msg.thrust = thrust_out;
    // if(abs(att_cmd_msg.thrust) <= 0.01){
    //     att_cmd_msg.thrust = 0;
    // }    
}

// bool PID_controller::estimateThrustModel(const Eigen::Vector3d &est_a) {
//     ros::Time t_now = ros::Time::now();
//     while (timed_thrust.size() >= 1) {
//         // Choose data before 35~45ms ago
//         std::pair<ros::Time, double> t_t = timed_thrust.front();
//         double time_passed = (t_now - t_t.first).toSec();
//         if (time_passed > 0.045) {  // 45ms
//             // printf("continue, time_passed=%f\n", time_passed);
//             timed_thrust.pop();
//             continue;
//         }
//         if (time_passed < 0.035) {  // 35ms
//             // printf("skip, time_passed=%f\n", time_passed);
//             return false;
//         }

//         /***********************************************************/
//         /* Recursive least squares algorithm with vanishing memory */
//         /***********************************************************/
//         double thr = t_t.second;
//         timed_thrust.pop();
        
//         /***********************************/
//         /* Model: est_a(2) = thr1acc_ * thr */
//         /***********************************/
//         double gamma = 1 / (rho + thr * P * thr);
//         double K = gamma * P * thr;
//         thr2acc = thr2acc + K * (est_a(2) - thr * thr2acc);
//         P = (1 - K * thr) * P / rho;
//         //printf("%6.3f,%6.3f,%6.3f,%6.3f\n", thr2acc_, gamma, K, P_);
//         //fflush(stdout);

//         // debug_msg_.thr2acc = thr2acc_;
//         return true;
//     }
//     return false;
// }
 