

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
            is_suc = trajGenVisPub();
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
                // emergencyStop();
                change_state(GEN_NEW_TRAJ);
                return;
            }

            unsafe_segment = Astar_path_finder->safeCheck(*traj_opt, skip_seg_num);
            if (unsafe_segment != -1) {
                change_state(GEN_NEW_TRAJ);
                // emergencyStop();
                return;
            }
            if ((traj_opt->odom_pos - target_pt).norm() < target_thresh) {    // 当前位置距离目标足够近，认为规划完成
                change_state(WAIT_TARGET);
                has_new_target = false;
                std::cout << "Reach target!" << std::endl;
                return;
            }
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
    int skip_seg_num = 0; 
}

void Plan_manage::change_state(STATE new_state) {
    int old_state = int(state);
    state = new_state;
    std::cout << "[state]: from " + state_str[old_state] + " to " + state_str[int(new_state)] << std::endl;
}

void Plan_manage::print_state() {
    std::cout << "[State]: " + state_str[int(state)] << std::endl;
}

bool Plan_manage::trajGenVisPub()
{
    skip_seg_num = 0;
    // A_star寻路
    bool is_path_found = Astar_path_finder->A_star_search(traj_opt->odom_pos, target_pt);
    if (!is_path_found) {
        return false;
    }
    std::vector<Eigen::Vector3d> grid_path = Astar_path_finder->get_path();
    std::vector<Eigen::Vector3d> path_main_point = Astar_path_finder->path_simplify(grid_path);
    std::vector<int> path_main_point_idx = getPathIdx(grid_path, path_main_point);
    Astar_path_finder->visGridPath(path_main_point, false);
    

    // 轨迹优化和碰撞检测
    if (! traj_opt->optimize(path_main_point)) 
    {
        return false;
    }

    int unsafe_segment = Astar_path_finder->safeCheck(*traj_opt, skip_seg_num);
    while (unsafe_segment != -1) {
        if (path_main_point_idx[unsafe_segment + 1] - path_main_point_idx[unsafe_segment] == 1) {  // 此时碰撞段的起点和终点已经在A_star路径里连续，无法中间插入，故忽略该段
            ++skip_seg_num;
            unsafe_segment = Astar_path_finder->safeCheck(*traj_opt, skip_seg_num);
            continue;
        }
        Eigen::Vector3d insert_point = grid_path[(path_main_point_idx[unsafe_segment + 1] + path_main_point_idx[unsafe_segment]) / 2];
        // 注意insert是在指定位置前插入，所以需要+1
        path_main_point.insert(path_main_point.begin() + unsafe_segment + 1, insert_point);
        path_main_point_idx = getPathIdx(grid_path, path_main_point);           // 不要忘了这句话
        traj_opt->time = traj_opt->time_allocation(path_main_point);
        if (! traj_opt->optimize(path_main_point)) 
        {
            return false;
        }
        unsafe_segment = Astar_path_finder->safeCheck(*traj_opt, skip_seg_num);
    }
    // for (size_t i = 0; i < path_main_point.size(); ++i) {
    //     const Eigen::Vector3d& point = path_main_point[i];
    //     std::cout << "Point " << i << ": (" 
    //                 << point.x() << ", " 
    //                 << point.y() << ", " 
    //                 << point.z() << ")" << std::endl;
    // }
    traj_opt->Visualize(path_main_point);
    return true;
}

void Plan_manage::emergencyStop() {
    quad_msgs::Target target;
    target.is_traj_safe = false;
    traj_opt->visualizer->target_pub.publish(target);
}

std::vector<int> Plan_manage::getPathIdx(const std::vector<Eigen::Vector3d> &path, const std::vector<Eigen::Vector3d> &path_main_point) 
{
    std::vector<int> path_main_point_idx_in_path;
    for (int i = 0; i < path_main_point.size(); ++i) {
        auto iter = std::find(path.begin(), path.end(), path_main_point[i]);
        if (iter != path.end()) {
            int idx = std::distance(path.begin(), iter);
            path_main_point_idx_in_path.push_back(idx);
        } else {
            path_main_point_idx_in_path.clear();
            path_main_point_idx_in_path.push_back(-1);
            return path_main_point_idx_in_path;
        }
    }
    // std::cout << "first idx is :" << path_main_point_idx_in_path[0] << std::endl;
    return path_main_point_idx_in_path;
}
