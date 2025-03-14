
#include <ros/ros.h>

class Plan_manage {

public:
    ros::Timer FSM_task_timer;
    void FSM_task(const ros::TimerEvent &event);
    void init(ros::NodeHandle &nh);
};