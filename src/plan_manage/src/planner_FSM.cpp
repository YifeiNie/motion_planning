

#include "trajectory_optimization.h"
#include "A_star.h"
#include "plan_manage.h"
extern AStarManager * Astar_path_finder;
extern Traj_opt * traj_opt;

void Plan_manage::FSM_task(const ros::TimerEvent &event)
{
    static int cnt = 0;
    cnt++;
    if (cnt == 100) {
        print_state();
        cnt = 0; 
    }
    // if (!has_odom) {
    //     std::cout << "wait for odom." << std::endl;
    // }
        
    // if (!has_target) {
    //     std::cout << "wait for goal." << std::endl;
    //     cnt = 0;
    // }

    switch (state)
    {
        case INIT:
            if (!has_odom || !has_target) {
                return;
            }
            change_state(WAIT_TARGET);
            break;

             
        case WAIT_TARGET:
            if (!has_target) {
                return;
            }
            change_state(GEN_NEW_TRAJ);
            break;

        case GEN_NEW_TRAJ:


            break;
        case EXEC_TRAJ:

            break;
        case REPLAN_TRAJ:

            break;            
        default:
            break;
    }
}

void Plan_manage::rcvWaypointsCallback(const nav_msgs::Path & wp)
{
    if( wp.poses[0].pose.position.z < 0.0 ) {
        ROS_WARN("Coordinate Z smaller than zero!!");
        return;
    }
    if (_has_map == false) {
        ROS_WARN("No map!!");
        return;
    }
    target_pt << wp.poses[0].pose.position.x,
                 wp.poses[0].pose.position.y,
                 wp.poses[0].pose.position.z;
    has_target = true;
    ROS_INFO("[Simple planner] receive the way-points");
}

void Plan_manage::init(ros::NodeHandle &nh)
{
    FSM_task_timer = nh.createTimer(ros::Duration(0.01), boost::bind(&Plan_manage::FSM_task, this, _1));
    odom_sub = nh.subscribe<nav_msgs::Odometry>("/vins_fusion/odometry", 1, boost::bind(&Traj_opt::odom_rcv_callback, this, _1));  
}

void Plan_manage::change_state(STATE new_state) {
    int old_state = int(state);
    state = new_state;
    std::cout << "[state]: from " + state_str[old_state] + " to " + state_str[int(new_state)] << std::endl;
}

void Plan_manage::print_state() {
  std::cout << "[State]: " + state_str[int(state)] << std::endl;
}


void Plan_manage::trajGen()
{
    // A_star寻路
    bool is_path_found = Astar_path_finder->A_star_search(traj_opt->odom_pos, target_pt);
    if (!is_path_found) {
        return;
    }
    std::vector<Eigen::Vector3d> grid_path = Astar_path_finder->get_path();
    std::vector<Eigen::Vector3d> path_main_point = Astar_path_finder->path_simplify(grid_path);
    visGridPath(path_main_point, false);

    // // 轨迹优化和碰撞检测
    traj_opt->optimize(path_main_point);
    int safecheck_iter = 0;
    int unsafe_segment = Astar_path_finder->safeCheck(*traj_opt);
    while (unsafe_segment != -1) {

        if (safecheck_iter >= Astar_path_finder->max_safecheck_iter) { // 说明插入了很多也无法避免碰撞，只能忽略此处碰撞防止无限循环
            break;  
        }
        Eigen::Vector3d insert_point = (path_main_point[unsafe_segment] + path_main_point[unsafe_segment + 1]) / 2;
        // 注意insert是在指定位置前插入，所以需要+1
        path_main_point.insert(path_main_point.begin() + unsafe_segment + 1, insert_point);
        ++ safecheck_iter;
        traj_opt->time = traj_opt->time_allocation(path_main_point);
        traj_opt->optimize(path_main_point);
        unsafe_segment = Astar_path_finder->safeCheck(*traj_opt);
    }
    traj_opt->Visualize(path_main_point);
}

void Plan_manage::rcvOdomCallback(nav_msgs::OdometryConstPtr msg)
{
    odom_pos(0) = msg->pose.pose.position.x;
    odom_pos(1) = msg->pose.pose.position.y;
    odom_pos(2) = msg->pose.pose.position.z;

    odom_vel(0) = msg->twist.twist.linear.x;
    odom_vel(1) = msg->twist.twist.linear.y;
    odom_vel(2) = msg->twist.twist.linear.z;

    has_odom = true;   
}
