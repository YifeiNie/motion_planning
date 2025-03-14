#include <iostream>
#include <fstream>
#include <math.h>
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

#include "backward.hpp"
#include "A_star.h"
#include "trajectory_optimization.h"
#include "grid_map.h"

using namespace std;
using namespace Eigen;

namespace backward {
backward::SignalHandling sh;
}

// simulation param from launch file
double _resolution, _inv_resolution, _cloud_margin;
double _x_size, _y_size, _z_size;    

// useful global variables
bool _has_map   = false;

Vector3d _start_pt;
Vector3d _map_lower, _map_upper;
int _max_x_id, _max_y_id, _max_z_id;

// ros related
ros::Subscriber _map_sub, _pts_sub;
ros::Publisher  _grid_path_vis_pub, _debug_nodes_vis_pub, _closed_nodes_vis_pub, _open_nodes_vis_pub, _close_nodes_sequence_vis_pub, _grid_map_vis_pub;

//gridPathFinder * Astar_path_finder = new gridPathFinder();
AStarManager * Astar_path_finder  = new AStarManager();
Traj_opt * traj_opt = new Traj_opt();
GridMap::Ptr grid_map;

void rcvWaypointsCallback(const nav_msgs::Path & wp);
void rcvPointCloudCallBack(const sensor_msgs::PointCloud2 & pointcloud_map);
void visGridPath( vector<Vector3d> nodes, bool is_use_jps );


void rcvWaypointsCallback(const nav_msgs::Path & wp)
{     
    if( wp.poses[0].pose.position.z < 0.0 ) {
        ROS_WARN("Coordinate Z smaller than zero!!");
        return;
    }
    if (_has_map == false) {
        ROS_WARN("No map!!");
        return;
    }


    Vector3d target_pt;
    target_pt << wp.poses[0].pose.position.x,
                 wp.poses[0].pose.position.y,
                 wp.poses[0].pose.position.z;

    ROS_INFO("[A_star_node] receive the way-points");

    // A_star寻路
    bool is_path_found = Astar_path_finder->A_star_search(traj_opt->odom_pos, target_pt);
    if (!is_path_found) {
        return;
    }
    std::vector<Eigen::Vector3d> grid_path = Astar_path_finder->get_path();
    std::vector<Eigen::Vector3d> path_main_point = Astar_path_finder->path_simplify(grid_path);
    visGridPath(path_main_point, false);

    // // 轨迹优化和碰撞检测
    traj_opt->optimize(path_main_point);
    int safecheck_iter = 0;
    int unsafe_segment = Astar_path_finder->safeCheck(*traj_opt);
    while (unsafe_segment != -1) {

        if (safecheck_iter >= Astar_path_finder->max_safecheck_iter) { // 说明插入了很多也无法避免碰撞，只能忽略此处碰撞防止无限循环
            break;  
        }
        Eigen::Vector3d insert_point = (path_main_point[unsafe_segment] + path_main_point[unsafe_segment + 1]) / 2;
        // 注意insert是在指定位置前插入，所以需要+1
        path_main_point.insert(path_main_point.begin() + unsafe_segment + 1, insert_point);
        ++ safecheck_iter;
        traj_opt->time = traj_opt->time_allocation(path_main_point);
        traj_opt->optimize(path_main_point);
        unsafe_segment = Astar_path_finder->safeCheck(*traj_opt);
    }
    traj_opt->Visualize(path_main_point);
    
}

// 接受random_complex_generator发出的原始点云并转化为栅格地图
void rcvPointCloudCallBack(const sensor_msgs::PointCloud2 & pointcloud_map)
{
    // if(_has_map ) return;

    pcl::PointCloud<pcl::PointXYZ> cloud;
    pcl::PointCloud<pcl::PointXYZ> cloud_vis;
    sensor_msgs::PointCloud2 map_vis;

    pcl::fromROSMsg(pointcloud_map, cloud);
    
    if ((int)cloud.points.size() == 0 ) 
    {
        std::cout << "Point cloud is enpty!" << std::endl;
        return;
    }
    // 遍历点云，并对每个点进行扩展操作，即对一个点的xyz都加减若干距离，然后根据分辨率确定扩展的点的个数
    pcl::PointXYZ pt, pt_inf;
    int inf_step   = round(_cloud_margin * _inv_resolution);
    int inf_step_z = max(1, inf_step / 2);
    for (int idx = 0; idx < (int)cloud.points.size(); idx++)
    {    
        pt = cloud.points[idx];        
        for(int x = -inf_step ; x <= inf_step; x ++ )
        {
            for(int y = -inf_step ; y <= inf_step; y ++ )
            {
                for(int z = -inf_step_z; z <= inf_step_z; z ++ )
                {
                    double inf_x = pt.x + x * _resolution;
                    double inf_y = pt.y + y * _resolution;
                    double inf_z = pt.z + z * _resolution;
                    Astar_path_finder->setObs(inf_x, inf_y, inf_z);

                    Vector3d cor_inf = Astar_path_finder->coordRounding(Vector3d(inf_x, inf_y, inf_z));

                    pt_inf.x = cor_inf(0);
                    pt_inf.y = cor_inf(1);
                    pt_inf.z = cor_inf(2);
                    cloud_vis.points.push_back(pt_inf);

                }
            }
        }
    }

    cloud_vis.width    = cloud_vis.points.size();
    cloud_vis.height   = 1;
    cloud_vis.is_dense = true;

    pcl::toROSMsg(cloud_vis, map_vis);

    map_vis.header.frame_id = "world";
    _grid_map_vis_pub.publish(map_vis);

    _has_map = true;
}
// void cbk(const ros::TimerEvent& e){
//     ROS_INFO("use %.4f secs", (ros::Time::now()).toSec());
// }
int main(int argc, char** argv)
{
    ros::init(argc, argv, "plan_manage");
    ros::NodeHandle nh("~");
    _map_sub  = nh.subscribe( "/plan_manage/grid_map/occupancy_inflate",       1, rcvPointCloudCallBack );
    _pts_sub  = nh.subscribe( "waypoints", 1, rcvWaypointsCallback );

    _grid_map_vis_pub             = nh.advertise<sensor_msgs::PointCloud2>("grid_map_vis", 1);
    _grid_path_vis_pub            = nh.advertise<visualization_msgs::Marker>("grid_path_vis", 1);
    _debug_nodes_vis_pub          = nh.advertise<visualization_msgs::Marker>("debug_nodes_vis", 1);
    _closed_nodes_vis_pub         = nh.advertise<visualization_msgs::Marker>("closed_nodes_vis",   1);
    _open_nodes_vis_pub           = nh.advertise<visualization_msgs::Marker>("open_nodes_vis",     1);
    _close_nodes_sequence_vis_pub = nh.advertise<visualization_msgs::Marker>("close_nodes_sequence_vis", 10);

    nh.param("map/cloud_margin",  _cloud_margin, 0.0);
    nh.param("map/resolution",    _resolution,   0.2);
    
    nh.param("map/x_size",        _x_size, 50.0);
    nh.param("map/y_size",        _y_size, 50.0);
    nh.param("map/z_size",        _z_size, 5.0 );
    
    nh.param("planning/start_x",  _start_pt(0),  0.0);
    nh.param("planning/start_y",  _start_pt(1),  0.0);
    nh.param("planning/start_z",  _start_pt(2),  0.0);

    _map_lower << - _x_size/2.0, - _y_size/2.0,     0.0;
    _map_upper << + _x_size/2.0, + _y_size/2.0, _z_size;
    
    _inv_resolution = 1.0 / _resolution;
    
    _max_x_id = (int)(_x_size * _inv_resolution);
    _max_y_id = (int)(_y_size * _inv_resolution);
    _max_z_id = (int)(_z_size * _inv_resolution);

    Astar_path_finder -> init(nh, _resolution, _map_lower, _map_upper, _max_x_id, _max_y_id, _max_z_id);
    traj_opt->init(nh);
    // ros::Timer time_test = nh.createTimer(ros::Duration(0.1), cbk);
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
    return 0;
}

void visGridPath( vector<Vector3d> nodes, bool is_use_jps )
{   
    visualization_msgs::Marker node_vis; 
    node_vis.header.frame_id = "world";
    node_vis.header.stamp = ros::Time::now();
    
    if(is_use_jps)
        node_vis.ns = "plan_manage/jps_path";
    else
        node_vis.ns = "plan_manage/astar_path";

    node_vis.type = visualization_msgs::Marker::CUBE_LIST;
    node_vis.action = visualization_msgs::Marker::ADD;
    node_vis.id = 0;

    node_vis.pose.orientation.x = 0.0;
    node_vis.pose.orientation.y = 0.0;
    node_vis.pose.orientation.z = 0.0;
    node_vis.pose.orientation.w = 1.0;

    if (is_use_jps){
        node_vis.color.a = 1.0;
        node_vis.color.r = 1.0;
        node_vis.color.g = 0.0;
        node_vis.color.b = 0.0;
    }
    else{
        node_vis.color.a = 1.0;
        node_vis.color.r = 0.0;
        node_vis.color.g = 1.0;
        node_vis.color.b = 0.0;
    }


    node_vis.scale.x = _resolution;
    node_vis.scale.y = _resolution;
    node_vis.scale.z = _resolution;

    geometry_msgs::Point pt;
    for(int i = 0; i < int(nodes.size()); i++)
    {
        Vector3d coord = nodes[i];
        pt.x = coord(0);
        pt.y = coord(1);
        pt.z = coord(2);

        node_vis.points.push_back(pt);
    }

    _grid_path_vis_pub.publish(node_vis);
}