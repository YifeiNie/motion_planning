#pragma once

// #include "lec5_hw/trajectory.hpp"

#include <iostream>
#include <memory>
#include <chrono>
#include <cmath>

#include <ros/ros.h>
#include <geometry_msgs/Point.h>

#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include "trajectory.h"
#include "quad_msgs/Target.h"

class Visualizer
{
private:
    ros::NodeHandle nh;

    ros::Publisher wayPointsPub;
    ros::Publisher trajectoryPub;
    ros::Publisher target_pub;
public:
    Visualizer(ros::NodeHandle &nh_)
        : nh(nh_)
    {
        wayPointsPub = nh.advertise<visualization_msgs::Marker>("/visualizer/waypoints", 10);
        trajectoryPub = nh.advertise<visualization_msgs::Marker>("/visualizer/trajectory", 10);
        target_pub = nh.advertise<quad_msgs::Target>("/target", 10);
    }

    template <int D>
    inline void visualize(const Trajectory<D> &traj,
                          const Eigen::Matrix3Xd &route)
    {
        visualization_msgs::Marker wayPointsMarker;
        visualization_msgs::Marker trajMarker;
        quad_msgs::Target target;

        target.header.stamp = ros::Time::now();
        target.header.frame_id = "world";
        
        // 路径点
        wayPointsMarker.id = 0;
        wayPointsMarker.type = visualization_msgs::Marker::SPHERE_LIST;
        wayPointsMarker.header.stamp = ros::Time::now();
        wayPointsMarker.header.frame_id = "world";
        wayPointsMarker.pose.orientation.w = 1.00;
        wayPointsMarker.action = visualization_msgs::Marker::ADD;
        wayPointsMarker.ns = "waypoints";
        wayPointsMarker.color.r = 1.00;
        wayPointsMarker.color.g = 0.00;
        wayPointsMarker.color.b = 0.00;
        wayPointsMarker.color.a = 1.00;
        wayPointsMarker.scale.x = 0.25;
        wayPointsMarker.scale.y = 0.25;
        wayPointsMarker.scale.z = 0.25;

        // 轨迹
        trajMarker = wayPointsMarker;
        trajMarker.type = visualization_msgs::Marker::LINE_LIST;
        trajMarker.header.frame_id = "world";
        trajMarker.id = 0;
        trajMarker.ns = "trajectory";
        trajMarker.color.r = 0.00;
        trajMarker.color.g = 0.50;
        trajMarker.color.b = 1.00;
        trajMarker.scale.x = 0.10;

        for (int i = 0; i < route.cols(); ++i)
        {
            geometry_msgs::Point point;
            point.x = route.col(i)(0);
            point.y = route.col(i)(1);
            point.z = route.col(i)(2);
            wayPointsMarker.points.push_back(point);
        }
        wayPointsPub.publish(wayPointsMarker);

        if (traj.getPieceNum() > 0)
        {
            const double T = std::min(0.01, traj.getTotalDuration() / 1000);
            Eigen::Vector3d lastPos = traj.getPos(0.0);
            for (double t = T; t < traj.getTotalDuration(); t += T)
            {
                geometry_msgs::Point point;
                Eigen::Vector3d pos = traj.getPos(t);
                Eigen::Vector3d vel = traj.getVel(t);
                Eigen::Vector3d acc = traj.getAcc(t);
                point.x = lastPos(0);
                point.y = lastPos(1);
                point.z = lastPos(2);
                trajMarker.points.push_back(point);
                point.x = pos(0);
                point.y = pos(1);
                point.z = pos(2);
                trajMarker.points.push_back(point);
                lastPos = pos;

                std_msgs::Float64MultiArray p, v, a, yaw;

                p.data.resize(3);
                v.data.resize(3);
                a.data.resize(3);
                yaw.data.resize(1);

                p.data[0] = pos.x();
                p.data[1] = pos.y();
                p.data[2] = pos.z();
                v.data[0] = vel.x();
                v.data[1] = vel.y();
                v.data[2] = vel.z();
                a.data[0] = acc.x();
                a.data[1] = acc.y();
                a.data[2] = acc.z();

                yaw.data[0] = atan2(vel.y(), vel.x());;

                // push_back 到消息中
                target.pos.push_back(p);
                target.vel.push_back(v);
                target.acc.push_back(a);
                target.yaw.push_back(yaw);
            }
            trajectoryPub.publish(trajMarker);
            target_pub.publish(target);
        }
        else
        {
            trajMarker.action = visualization_msgs::Marker::DELETEALL;
            trajectoryPub.publish(trajMarker);
        }
    }
};
