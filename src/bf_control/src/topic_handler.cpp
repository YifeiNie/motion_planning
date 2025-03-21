#include <mavros_msgs/AttitudeTarget.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <mavros_msgs/RCIn.h>

#include "topic_handler.h"
// #include
  
bool Topic_handler::is_rc_received(const ros::Time &now_time){
	return (now_time - rc.rcv_stamp).toSec() < 0.5;   // 设置0.5秒为超时
}

bool Topic_handler::is_odom_received(const ros::Time &now_time){
    return (now_time - odom.rcv_stamp).toSec() < 0.5;
}

bool Topic_handler::is_imu_received(const ros::Time &now_time){
    return (now_time - imu.rcv_stamp).toSec() < 0.5;
}


void RC::feed(mavros_msgs::RCInConstPtr msg){
    roll = msg->channels[0];
    pitch = msg->channels[1];
    yaw = msg->channels[2];
    thrust = msg->channels[3];
    if (msg->channels[4] > 1700){
        is_armed = 1;
    }else{
        is_armed = 0;
    }
    if (msg->channels[7] > 1700){
        is_offboard = 1;
    }else{
        is_offboard = 0;
    }

    if (msg->channels[6] > 1400 && msg->channels[6] < 1600){
        is_auto_land = 1;
    }else{
        is_auto_land = 0;
    }
    rcv_stamp = ros::Time::now();
    // ROS_INFO("channel 7 = %d",msg->channels[7] );
    // ROS_INFO("is_offboard = %d", is_offboard);
    // ROS_INFO("roll = %d", roll);
}


void Odom::feed(nav_msgs::OdometryConstPtr msg){
    position(0) = msg->pose.pose.position.x;
    position(1) = msg->pose.pose.position.y;
    position(2) = msg->pose.pose.position.z;

    velocity(0) = msg->twist.twist.linear.x;
    velocity(1) = msg->twist.twist.linear.y;
    velocity(2) = msg->twist.twist.linear.z;

    q.w() = msg->pose.pose.orientation.w;
    q.x() = msg->pose.pose.orientation.x;
    q.y() = msg->pose.pose.orientation.y;
    q.z() = msg->pose.pose.orientation.z;
    rcv_stamp = ros::Time::now();
    
    // ROS_INFO("Get odom data!!!");
}

double Odom::get_current_yaw(){
    double yaw = atan2(2*(q.x()*q.y() + q.w()*q.z()), q.w()*q.w() + q.x()*q.x() - q.y()*q.y() - q.z()*q.z());
    return yaw;
}

void Imu::feed(sensor_msgs::ImuConstPtr msg){
    linear_acc(0) = msg->linear_acceleration.x;
    linear_acc(1) = msg->linear_acceleration.y;
    linear_acc(2) = msg->linear_acceleration.z;
    q.x() = msg->orientation.x;
    q.y() = msg->orientation.y;
    q.z() = msg->orientation.z;
    q.w() = msg->orientation.w;
    angle_vel(0) = msg->angular_velocity.x;
    angle_vel(1) = msg->angular_velocity.y;
    angle_vel(2) = msg->angular_velocity.z;
    rcv_stamp = ros::Time::now();
    // ROS_INFO("Get imu data!!!");
}

double Imu::get_current_yaw(){
    double yaw = atan2(2*(q.x()*q.y() + q.w()*q.z()), q.w()*q.w() + q.x()*q.x() - q.y()*q.y() - q.z()*q.z());
    return yaw;
}

void Topic_handler::rcvTargetCallback(quad_msgs::TargetConstPtr msg) 
{
    pos_target = msg->pos;
    yaw_target = msg->yaw;
    this->is_traj_safe = msg->is_traj_safe;
    has_target = true;
    traj_size = pos_target.size();
    traj_idx_iter = 1;
    std::cout << "[Topic handler] Receive trajector! " << std::endl;
}

void Topic_handler::init(ros::NodeHandle& nh) {

    odom_sub = nh.subscribe<nav_msgs::Odometry>("/vins_fusion/odometry", 10, boost::bind(&Odom::feed, &(this->odom), _1));
    rc_sub = nh.subscribe<mavros_msgs::RCIn>("/mavros/rc/in", 10, boost::bind(&RC::feed, &(this->rc), _1));
    imu_sub = nh.subscribe<sensor_msgs::Imu>("/mavros/imu/data", 10, boost::bind(&Imu::feed, &(this->imu), _1));
    target_sub = nh.subscribe<quad_msgs::Target>("/target", 1, boost::bind(&Topic_handler::rcvTargetCallback, this, _1));

    mav_cmd_pub = nh.advertise<mavros_msgs::AttitudeTarget>("/mavros/setpoint_raw/attitude", 10);
    debug_flag = 1;
    int traj_idx_iter = 0;
}
