
#pragma once
#include <ros/ros.h>
#include <Eigen/Dense>
#include <mavros_msgs/AttitudeTarget.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <mavros_msgs/RCIn.h>

#include "PID_controller.h"
#include "topic_handler.h"

enum State{
    INIT,
    MANUAL_CTRL, 
    HOVER,
    CMD_CTRL,	
    AUTO_TAKEOFF,
    AUTO_LAND
};

class BFcontrol_FSM {
public:
    std::string state_str[6] = {"INIT", "MANUAL_CTRL", "HOVER", "CMD_CTRL", "AUTO_TAKEOFF", "AUTO_LAND"};
    State state = INIT;
    PID_controller pid;

    void run(Topic_handler &th);
    void change_state(State new_state) ;
    void printCmd();
    inline void pidProcess(Topic_handler &th);
};
