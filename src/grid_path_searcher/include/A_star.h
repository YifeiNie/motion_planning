#pragma once
#include <Eigen/Eigen>

#define USE_A_STAR

#ifdef USE_A_STAR

#define inf 1>>20


struct GridNode
{     
    uint8_t is_close;     
	uint8_t is_path;   
	double gScore, fScore;

    Eigen::Vector3d coord; 
    Eigen::Vector3i dir;   // direction of expanding
    Eigen::Vector3i idx;
	
    GridNode* father;
    std::multimap<double, GridNode*>::iterator nodeMapIt;

    GridNode(Eigen::Vector3i idx, Eigen::Vector3d coord);

    GridNode(){};
    ~GridNode(){};
};

class AStarManager
{
private: 
    
    GridNode ****GridNodeMap;
    uint8_t *obstacle_map;     // 一个三维映射到一维的数组，存储的0或1表示当前索引的栅格是否有障碍物\

    std::vector<GridNode*> neighbor_ptr_set;
    std::vector<double> edge_cost_set;

    float rezolution;
    float max_x_coord, max_y_coord, max_z_coord;   // 坐标值，
    float min_x_coord, min_y_coord, min_z_coord;

    // 由于访问数组的索引是从0开始，所以后面进行遍历时候是"< max_x_idx"而不是"<= max_x_idx"
    int max_x_idx, max_y_idx, max_z_idx;                //索引值，(min_x_coord, min_x_coord, min_z_coord)被认为是(0, 0, 0)索引
    int min_x_idx, min_y_idx, min_z_idx = 0;
    int max_yz_idx = max_y_idx * max_z_idx;           // 用于三维数组的一维映射时，减少重复计算
    int grid_num = max_x_idx * max_x_idx * max_z_idx;   // 整个地图总共的栅格数量，也即映射到一维时的数组大小

    Eigen::Vector3i goal_idx;   // 目标点的索引

    double resolution, inv_resolution;  // 栅格地图分辨率
	std::vector<GridNode*> expanded_nodes;
	std::vector<GridNode*> grid_path;
	std::vector<GridNode*> endPtrList;

    bool is_obstacle(Eigen::Vector3i idx);

public:
    void gridmap_init(double resolution, Eigen::Vector3d max_coord, Eigen::Vector3d min_coord, int max_x_idx, int max_y_idx, int max_z_idx);
    inline Eigen::Vector3i coord2idx(int x, int y, int z);
    inline Eigen::Vector3d idx2coord(Eigen::Vector3i tem_idx);
    void set_obstacle(Eigen::Vector3i idx);
    void A_star_expand_neighbors(GridNode* GridNodePtr);
    void A_star_search(Eigen::Vector3d start_coord, Eigen::Vector3d goal_coord);
    
};

#endif