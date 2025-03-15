#include <ros/ros.h>

void cbk(const ros::TimerEvent& e){
    ROS_INFO("use %.4f secs", (ros::Time::now()).toSec());
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "timer_test");
    ros::NodeHandle nh("~");

    ros::Timer vis_timer_ = nh.createTimer(ros::Duration(0.1), cbk);

    ros::spin();
    return 0;
}