
#include <ros/ros.h>
#include <Eigen/Eigen>
#include <nav_msgs/Odometry.h>

enum STATE {
    INIT,
    WAIT_TARGET,
    GEN_NEW_TRAJ,
    EXEC_TRAJ,
    REPLAN_TRAJ
};


class Plan_manage {
private:
    bool has_target;
    Eigen::Vector3d target_pt;
    ros::Timer FSM_task_timer;

    ros::Subscirber odom_sub;
public:
    STATE state;
    std::string state_str[5] = {"INIT", "WAIT_TARGET", "GEN_NEW_TRAJ", "EXEC_TRAJ", "REPLAN_TRAJ"};
    void FSM_task(const ros::TimerEvent &event);
    void init(ros::NodeHandle &nh);

    void change_state(STATE new_state);
    void print_state();                                     // 定期执行    

    void rcvWaypointsCallback(const nav_msgs::Path & wp);
    void rcvOdomCallback(nav_msgs::OdometryConstPtr msg);        // 用于获取轨迹规划的初始的pos，vel，和acc
    void trajGen();
};