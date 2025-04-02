#include <ros/ros.h>
#include "BFcontrol_FSM.h"
#include "topic_handler.h"
#include <PID_controller.h>

Topic_handler th;       // 实时更新并存储话题数据

int main(int argc, char **argv){
    ros::init(argc, argv, "offboard_node");

    ros::NodeHandle nh("~");
    BFcontrol_FSM fsm;      // 用于执行程序主体
    th.init(nh);
    fsm.pid.init(nh);
    ros::Rate rate(CTRL_FREQUENCY);
    ROS_INFO("offboard_node is running");
    while (ros::ok()){
        ros::spinOnce();
        fsm.run(th);
        rate.sleep();
    }
    return 0;
}