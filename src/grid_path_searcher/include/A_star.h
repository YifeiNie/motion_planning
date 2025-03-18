#pragma once
#include <Eigen/Eigen>
#include <tuple>
#include "trajectory_optimization.h"
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#define USE_A_STAR

#ifdef USE_A_STAR

#define inf 1e20


struct GridNode
{     
    uint8_t is_close = 0;     
	uint8_t is_path;   
	double g_score, f_score;

    Eigen::Vector3d coord; 
    Eigen::Vector3i dir;   
    Eigen::Vector3i idx;
	
    GridNode* father;

    GridNode(Eigen::Vector3i idx, Eigen::Vector3d coord);

    GridNode(){};
    ~GridNode(){};
};

class AStarManager
{
private: 
    double path_resolution;
    double delta_t;
    double cloud_margin;

    GridNode ****GridNodeMap;
    uint8_t *obstacle_map;     // 一个三维映射到一维的数组，存储的0或1表示当前索引的栅格是否有障碍物
    std::multimap<double, GridNode*> open_list;

    std::vector<GridNode*> neighbor_ptr_set;
    std::vector<double> edge_cost_set;
    GridNode *final_node_ptr;
    double x_size, y_size, z_size;
    float max_x_coord, max_y_coord, max_z_coord;   // 坐标值，
    float min_x_coord, min_y_coord, min_z_coord;

    // 由于访问数组的索引是从0开始，所以后面进行遍历时候是"< max_x_idx"而不是"<= max_x_idx"
    int max_x_idx, max_y_idx, max_z_idx;                //索引值，(min_x_coord, min_x_coord, min_z_coord)被认为是(0, 0, 0)索引
    int min_x_idx, min_y_idx, min_z_idx = 0;
    int max_yz_idx;           // 用于三维数组的一维映射时，减少重复计算
    int grid_num = max_x_idx * max_x_idx * max_z_idx;   // 整个地图总共的栅格数量，也即映射到一维时的数组大小

    Eigen::Vector3i goal_idx;   // 目标点的索引

    double resolution, inv_resolution;  // 栅格地图分辨率

    inline bool is_obstacle(Eigen::Vector3i idx);
    inline double heuristics(Eigen::Vector3d start_coord, Eigen::Vector3d goal_coord);

public:
    int max_safecheck_iter;
    ros::Publisher grid_path_vis_pub;
    ros::Publisher grid_map_vis_pub;
    ros::Subscriber pointcloud_sub;

    void init(ros::NodeHandle &nh);
    inline Eigen::Vector3i coord2idx(Eigen::Vector3d coord);
    inline Eigen::Vector3d idx2coord(Eigen::Vector3i idx);
    void set_obstacle(Eigen::Vector3i idx);
    void A_star_expand_neighbors(GridNode* GridNodePtr);
    bool A_star_search(Eigen::Vector3d start_coord, Eigen::Vector3d goal_coord);
    std::vector<Eigen::Vector3d> get_path();
    Eigen::Vector3d coordRounding(const Eigen::Vector3d &coord);

    void setObs(const double coord_x, const double coord_y, const double coord_z);
    inline double calculate_d(const Eigen::Vector3d point_insert,const Eigen:: Vector3d point_st,const Eigen::Vector3d point_end);
    std::vector<Eigen::Vector3d> path_simplify(const std::vector<Eigen::Vector3d> &path);
    int safeCheck(Traj_opt &traj_opt, int skip_seg_num);
    int safeCheckforFSM(Traj_opt &traj_opt);
    void reset_grid();
    void visGridPath(std::vector<Eigen::Vector3d> nodes, bool is_use_jps );
    void rcvPointCloudCallBack(sensor_msgs::PointCloud2ConstPtr pointcloud_map);
};

#endif