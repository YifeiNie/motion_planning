

#include "trajectory_optimization.h"
#include "A_star.h"
#include "plan_manage.h"
extern AStarManager * Astar_path_finder;
extern Traj_opt * traj_opt;

void Plan_manage::FSM_task(const ros::TimerEvent &event)
{
    // static int cnt = 0;
    // cnt++;
    // if (cnt == 100) {
    //     print_state();
    //     cnt = 0; 
    // }
    // if (!has_odom) {
    //     std::cout << "wait for odom." << std::endl;
    // }
        
    // if (!has_target) {
    //     std::cout << "wait for goal." << std::endl;
    //     cnt = 0;
    // }

    // switch (state)
    // {
    //     case INIT:
    //         if (!has_odom || !has_target) {
    //             return;
    //         }
    //         change_state(WAIT_TARGET);
    //         break;

            
    //     case WAIT_TARGET:
    //         if (!has_target) {
    //             return;
    //         }
    //         change_state(GEN_NEW_TRAJ);
    //         break;

    //     case GEN_NEW_TRAJ:


    //         break;
    //     case EXEC_TRAJ:

    //         break;
    //     case REPLAN_TRAJ:

    //         break;            
    //     default:
    //         break;
    // }
}

void Plan_manage::init(ros::NodeHandle &nh)
{
    FSM_task_timer = nh.createTimer(ros::Duration(0.01), boost::bind(&Plan_manage::FSM_task, this, _1));
}