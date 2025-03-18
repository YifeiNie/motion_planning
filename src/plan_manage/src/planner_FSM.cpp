

#include "trajectory_optimization.h"
#include "A_star.h"
#include "plan_manage.h"
#include "quad_msgs/Target.h"

extern AStarManager * Astar_path_finder;
extern Traj_opt * traj_opt;

void Plan_manage::FSM_task(const ros::TimerEvent &event)
{
    bool is_suc;
    int unsafe_segment;

    static int cnt = 0;
    ++cnt;
    if (cnt >= 100) {
        print_state();
        cnt = 0; 
    }
    // if (!has_odom) {
    //     std::cout << "[Simple Planner] wait for odom." << std::endl;
    // }
        
    // if (!has_new_target) {
    //     std::cout << "wait for goal." << std::endl;
    //     cnt = 0;
    // }

    switch (state)
    {
        case INIT:
            if (!traj_opt->has_odom || !has_new_target) {
                return;
            }
            change_state(WAIT_TARGET);
            break;

             
        case WAIT_TARGET:
            if (!has_new_target) {
                return;
            }
            if ((traj_opt->odom_pos - target_pt).norm() < no_plan_thresh) {  // 当前点距离目标点太近，不执行规划
                std::cout << "too close between current and target, do not execute plan" << std::endl;
                return;
            } 
            change_state(GEN_NEW_TRAJ);
            break;

        case GEN_NEW_TRAJ:
            is_suc = trajGen();
            has_new_target = false;         // 标记当前target已被处理
            if (!is_suc) {                    
                change_state(WAIT_TARGET);
                return;
            }
            else {
                change_state(EXEC_TRAJ);
                return;
            }
            break;

        case EXEC_TRAJ:
            if (has_new_target) {
                emergencyStop();
                change_state(GEN_NEW_TRAJ);
                return;
            }

            unsafe_segment = Astar_path_finder->safeCheck(*traj_opt);
            if (unsafe_segment != -1) {
                change_state(GEN_NEW_TRAJ);
                emergencyStop();
                return;
            }
            if ((traj_opt->odom_pos - target_pt).norm() < target_thresh) {    // 当前位置距离目标足够近，认为规划完成
                change_state(WAIT_TARGET);
                has_new_target = false;
                std::cout << "Reach target!" << std::endl;
                return;
            }
            break;
        case REPLAN_TRAJ:
            break;            
        default:
            break;
    }
}

void Plan_manage::rcvWaypointsCallback(nav_msgs::PathConstPtr wp)
{
    if( wp->poses[0].pose.position.z < 0.0 ) {
        ROS_WARN("Coordinate Z smaller than zero!!");
        return;
    }
    // if (_has_map == false) {
    //     ROS_WARN("No map!!");
    //     return;
    // }
    target_pt << wp->poses[0].pose.position.x,
                 wp->poses[0].pose.position.y,
                 wp->poses[0].pose.position.z;
    has_new_target = true;
    ROS_INFO("[Simple planner] receive the way-points");
}

void Plan_manage::init(ros::NodeHandle &nh)
{
    no_plan_thresh = nh.param("Plam/no_plan_thresh", 0.01);
    target_thresh = nh.param("Plam/target_thresh", 0.01);
    pts_sub  = nh.subscribe<nav_msgs::Path>( "waypoints", 1, boost::bind(&Plan_manage::rcvWaypointsCallback, this, _1 ));
    FSM_task_timer = nh.createTimer(ros::Duration(0.01), boost::bind(&Plan_manage::FSM_task, this, _1));
    // odom_sub = nh.subscribe<nav_msgs::Odometry>("/vins_fusion/odometry", 1, boost::bind(&Plan_manage::rcvOdomCallback, this, _1));  

    state = INIT;
}

void Plan_manage::change_state(STATE new_state) {
    int old_state = int(state);
    state = new_state;
    std::cout << "[state]: from " + state_str[old_state] + " to " + state_str[int(new_state)] << std::endl;
}

void Plan_manage::print_state() {
  std::cout << "[State]: " + state_str[int(state)] << std::endl;
}


bool Plan_manage::trajGen()
{
    // A_star寻路
    bool is_path_found = Astar_path_finder->A_star_search(traj_opt->odom_pos, target_pt);
    if (!is_path_found) {
        return false;
    }
    std::vector<Eigen::Vector3d> grid_path = Astar_path_finder->get_path();
    std::vector<Eigen::Vector3d> path_main_point = Astar_path_finder->path_simplify(grid_path);
    Astar_path_finder->visGridPath(path_main_point, false);

    // // 轨迹优化和碰撞检测
    if (! traj_opt->optimize(path_main_point)) 
    {
        return false;
    }
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
        if (! traj_opt->optimize(path_main_point)) 
        {
            return false;
        }
        unsafe_segment = Astar_path_finder->safeCheck(*traj_opt);
    }
    traj_opt->Visualize(path_main_point);
    return true;
}

// void Plan_manage::rcvOdomCallback(nav_msgs::OdometryConstPtr msg)
// {
//     traj_opt->odom_pos(0) = msg->pose.pose.position.x;
//     traj_opt->odom_pos(1) = msg->pose.pose.position.y;
//     traj_opt->odom_pos(2) = msg->pose.pose.position.z;

//     traj_opt->odom_vel(0) = msg->twist.twist.linear.x;
//     traj_opt->odom_vel(1) = msg->twist.twist.linear.y;
//     traj_opt->odom_vel(2) = msg->twist.twist.linear.z;

//     traj_opt->has_odom = true;   
// }

void Plan_manage::emergencyStop() {
    quad_msgs::Target target;
    target.is_traj_safe = false;
    traj_opt->visualizer->target_pub.publish(target);
}
