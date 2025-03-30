#include "BFcontrol_FSM.h"

#define OFFBOARD_SAFE_CHECK() \
    do { \
        if (!(th).rc.is_armed) { \
            change_state(INIT); \
            break; \
        } \
        if (!(th).rc.is_offboard) { \
            change_state(MANUAL_CTRL); \
            break; \
        } \
    } while (0)

void BFcontrol_FSM::run(Topic_handler &th){

    ros::Time now_time = ros::Time::now();
    static int traj_exec_cnt = 0;
    static double takeofff_sum;
    static int cnt = 0;
    ++ cnt;
    if (cnt >= 50) {
        std::cout << "[BF control] state is: " + state_str[int(state)] << std::endl;
        std::cout << "desire is: \n" << pid.desire_position << "\r" << std::endl;
        std::cout << "error is: \n" << pid.desire_position - th.odom.position << std::endl;
        std::cout << "takeoff sum is: \r\n" << takeofff_sum << std::endl;
        cnt = 0; 
    }

    switch (state){
        case INIT:
            if (th.rc.is_armed) {
                change_state(MANUAL_CTRL);
            }
            break;
        
        case MANUAL_CTRL:
            if (!th.rc.is_armed) {
                change_state(INIT);
                break;
            }
            if ((abs(th.odom.position.z() - 0) < 0.06) && (th.rc.is_offboard)) {
                pid.reset();
                pid.setDesire(th.odom.position.x(), 
                              th.odom.position.y(), 
                              th.odom.position.z(), 
                              th.odom.get_current_yaw());                
                change_state(AUTO_TAKEOFF);
                break;
            }
            if ((abs(th.odom.position.z() - 0) > 0.06) && (th.rc.is_offboard)) {
                std::cout << "[BF control] UAV not on land or odom drift, cannot enter offboard mode!" << std::endl;
            }

            break;

        case AUTO_TAKEOFF:
            OFFBOARD_SAFE_CHECK();
            // std::cout << "[BF control] Take off!" << std::endl;

            if (takeofff_sum <= pid.hover_height && pid.hover_height - th.odom.position.z() >= 0.06 ) {
                takeofff_sum = takeofff_sum + 0.005;
                pid.desire_position.z() = takeofff_sum;
            }
            else {
                takeofff_sum = 0;
                change_state(HOVER);
                break;
            }
            pidProcess(th);
            break;


        case HOVER:
            OFFBOARD_SAFE_CHECK();
            if (th.rc.is_auto_land) {
                change_state(AUTO_LAND);
                break;
            }
            pidProcess(th);
            if ((abs(th.odom.position.z() - pid.desire_position.z()) < 0.08) && th.has_target) {
                change_state(CMD_CTRL);
            }
            break;

        case CMD_CTRL:
            OFFBOARD_SAFE_CHECK();
            if (th.rc.is_auto_land) {
                traj_exec_cnt = 0;
                th.traj_idx_iter = 0;
                th.has_target = false;
                change_state(AUTO_LAND);
                break;
            }
            if (!th.is_traj_safe) {
                pid.setDesire(th.odom.position.x(), th.odom.position.y(), pid.hover_height, th.odom.get_current_yaw());
                traj_exec_cnt = 0;
                th.traj_idx_iter = 0;
                th.has_target = false;
                change_state(HOVER);

                break;
            }
            if (th.traj_idx_iter >= th.traj_size - 1) {
                traj_exec_cnt = 0;
                th.traj_idx_iter = 0;
                th.has_target = false;
                std::cout << "[BF control] Reach the taget!" << std::endl;
                change_state(HOVER);
                break;
            }
            ++ traj_exec_cnt;
            if (traj_exec_cnt >= 9) {
                ++ th.traj_idx_iter; 
                pid.setDesire(th.pos_target[th.traj_idx_iter].data[0], 
                              th.pos_target[th.traj_idx_iter].data[1], 
                              th.pos_target[th.traj_idx_iter].data[2], 
                              th.yaw_target[th.traj_idx_iter].data[0]); 
                traj_exec_cnt = 0;
                // std::cout << "target pos x is: " << th.pos_target[th.traj_idx_iter].data[0] << std::endl; 
                // std::cout << "the rest idx is: " << th.traj_idx_iter - th.traj_size << std::endl;
            }
         
            pidProcess(th);
            break;

        case AUTO_LAND:
            OFFBOARD_SAFE_CHECK();
            pid.desire_position.z() = pid.desire_position.z() - 0.008;
            pidProcess(th);

            if (th.odom.position.z() <= -0.2 && !th.rc.is_offboard && !th.rc.is_armed) {
                change_state(INIT);
                break;
            }
            break;
        default :
            break;
    }

}

void BFcontrol_FSM::change_state(State new_state) {
    int old_state = int(state);
    state = new_state;
    std::cout << "[state]: from " + state_str[old_state] + " to " + state_str[int(new_state)] << std::endl;
}

void BFcontrol_FSM::printCmd() 
{
    ROS_INFO("x cmd is %f", pid.att_cmd_msg.body_rate.x);
    ROS_INFO("y cmd is %f", pid.att_cmd_msg.body_rate.y);
    ROS_INFO("thrust cmd is %f", pid.att_cmd_msg.thrust);
    ROS_INFO("desire pos is %f", pid.desire_position[2]);
}

inline void BFcontrol_FSM::pidProcess(Topic_handler &th) 
{
    pid.outer_position_loop(th);
    pid.inner_velocity_loop(th);
    th.mav_cmd_pub.publish(pid.att_cmd_msg);
}