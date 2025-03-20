#include <iostream>
#include <fstream>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <ros/ros.h>
#include <ros/console.h>
#include <sensor_msgs/PointCloud2.h>

#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>

// #include "backward.hpp"
#include "A_star.h"
#include "trajectory_optimization.h"
#include "grid_map.h"
#include "plan_manage.h"

using namespace std;
using namespace Eigen;

// namespace backward {
// backward::SignalHandling sh;
// }


// ros::Subscriber _map_sub, _pts_sub;
// ros::Publisher  _grid_path_vis_pub, _debug_nodes_vis_pub, _closed_nodes_vis_pub, _open_nodes_vis_pub, _close_nodes_sequence_vis_pub, _grid_map_vis_pub;

AStarManager * Astar_path_finder  = new AStarManager();
Traj_opt * traj_opt = new Traj_opt();
Plan_manage * plan_manage = new Plan_manage();
GridMap::Ptr grid_map;

int main(int argc, char** argv)
{
    ros::init(argc, argv, "plan_manage");
    ros::NodeHandle nh("~");

    // _grid_map_vis_pub             = nh.advertise<sensor_msgs::PointCloud2>("grid_map_vis", 1);
    // _grid_path_vis_pub            = nh.advertise<visualization_msgs::Marker>("grid_path_vis", 1);
    // _debug_nodes_vis_pub          = nh.advertise<visualization_msgs::Marker>("debug_nodes_vis", 1);
    // _closed_nodes_vis_pub         = nh.advertise<visualization_msgs::Marker>("closed_nodes_vis",   1);
    // _open_nodes_vis_pub           = nh.advertise<visualization_msgs::Marker>("open_nodes_vis",     1);
    // _close_nodes_sequence_vis_pub = nh.advertise<visualization_msgs::Marker>("close_nodes_sequence_vis", 10);

    // 初始化A_star，后端优化和建图模块
    Astar_path_finder->init(nh);
    traj_opt->init(nh);
    plan_manage->init(nh);
    grid_map.reset(new GridMap);
    grid_map->initMap(nh);
    ros::spin();

    // ros::Rate rate(100);
    // while(ros::ok()) 
    // {
    //     ros::spinOnce();      
    //     rate.sleep();
    // }
    delete Astar_path_finder;
    delete traj_opt;
    return 0;
}
