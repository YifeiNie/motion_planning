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

    static int cnt = 0;
    ++cnt;
    if (cnt >= 100) {
        std::cout << "[State]: " + state_str[int(state)] << std::endl;
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
                change_state(AUTO_TAKEOFF);
                break;
            }
            if ((abs(th.odom.position.z() - 0) > 0.06) && (th.rc.is_offboard)) {
                std::cout << "UAV not on land or odom drift, cannot enter offboard mode!" << std::endl;
            }

            break;

        case AUTO_TAKEOFF:
            OFFBOARD_SAFE_CHECK();
            pid.reset();
            pid.setDesire(th.odom.position.x(), 
                          th.odom.position.y(), 
                          th.odom.position.z() + pid.hover_height, 
                          th.odom.get_current_yaw());
            change_state(HOVER);
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
            if (th.traj_idx_iter == th.traj_size) {
                traj_exec_cnt = 0;
                th.traj_idx_iter = 0;
                th.has_target = false;
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
                std::cout << "target pos is: " << th.pos_target[th.traj_idx_iter].data[0] << std::endl; 
            }
         
            pidProcess(th);
            break;

        case AUTO_LAND:
            OFFBOARD_SAFE_CHECK();
            pid.desire_position.z() = pid.desire_position.z() - 0.0005;
            pidProcess(th);

            if (th.odom.position.z() <= -0.2 && !th.rc.is_offboard && !th.rc.is_armed) {
                change_state(INIT);
                break;
            }
            break;

        // case MANUAL_CTRL:
        //     if (!th.rc.is_armed) {
        //         change_state(INIT);
        //         break;
        //     }
        //     pid.reset();
        //     ROS_INFO("is_armed = %d", th.rc.is_armed);
        //     ROS_INFO("is_offboard = %d", th.rc.is_offboard);
        //     if(th.rc.is_offboard){
        //         pid.reset();
        //         pid.desire_position = th.odom.position;
        //         pid.desire_position[2] += 0.4;
        //         pid.desire_yaw = th.odom.get_current_yaw();
        //         change_state(CMD_CTRL);
        //     }
        //     break;
        // case CMD_CTRL:
        //     if (!th.rc.is_armed) {
        //         change_state(INIT);
        //     }
        //     if(!(th.rc.is_offboard)){
        //         pid.reset();
        //         change_state(MANUAL_CTRL);
        //     }
        //     else{
        //         pid.outer_position_loop(th);
        //         pid.inner_velocity_loop(th);
        //         th.mav_cmd_pub.publish(pid.att_cmd_msg);
        //         printCmd();
        //     }
        //     break;
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