#include <ros/ros.h>
#include <ros/console.h>
#include <cmath>

#include "backward.hpp"
#include "A_star.h"

#ifdef USE_A_STAR

GridNode::GridNode(Eigen::Vector3i idx, Eigen::Vector3d coord)
{
    is_close = 0;
    is_path = 0;
    this->idx = idx;
    this->coord = coord;
    this->dir = Eigen::Vector3i::Zero();

    gScore = inf;
    fScore = inf;
    father = NULL;
}

// 初始化
void AStarManager::gridmap_init(double _resolution, Eigen::Vector3d max_coord, Eigen::Vector3d min_coord, int max_x_idx, int max_y_idx, int max_z_idx)
{
    this->resolution = resolution;
    neighbor_ptr_set.clear();
    edge_cost_set.clear();

    max_x_coord = max_coord(0);
    max_y_coord = max_coord(1);
    max_z_coord = max_coord(2);

    min_x_coord = min_coord(0);
    min_y_coord = min_coord(1);
    min_z_coord = min_coord(2);

    // 由于访问数组的索引是从0开始，所以后面进行遍历时候是"< max_x_idx"而不是"<= max_x_idx"
    this->max_x_idx = max_x_idx;
    this->max_y_idx = max_y_idx;
    this->max_z_idx = max_z_idx;

    // 计算栅格总数
    grid_num = max_x_idx * max_y_idx * max_z_idx;

    obstacle_map = new uint8_t[grid_num];
    memset(obstacle_map, 0, grid_num * sizeof(uint8_t));

    GridNodeMap = new GridNode ***[max_x_idx];
    for (int i = 0; i < max_x_idx; i++)
    {
        GridNodeMap[i] = new GridNode **[max_y_idx];
        for (int j = 0; j < max_y_idx; j++)
        {
            GridNodeMap[i][j] = new GridNode *[max_z_idx];
            for (int k = 0; k < max_z_idx; k++)
            {
                Eigen::Vector3i temp_idx(i, j, k);
                Eigen::Vector3d temp_coord = idx2coord(temp_idx);
                GridNodeMap[i][j][k] = new GridNode(temp_idx ,temp_coord);
            }
        }
    }
}

// 返回某个栅格是否为障碍物栅格
bool AStarManager::is_obstacle(Eigen::Vector3i idx)
{
    uint8_t temp_is_out_of_range = idx(0)>=0 && idx(0)<max_x_idx && idx(1)>=0 && idx(1)<max_y_idx && idx(2)>=0 && idx(2)<max_z_idx;
    return temp_is_out_of_range && obstacle_map[idx(0)*max_yz_idx + idx(1)* max_z_idx + idx(2)];
}

void AStarManager::set_obstacle(Eigen::Vector3i idx)
{
    obstacle_map[idx(0)*max_yz_idx + idx(1)* max_z_idx + idx(2)] = 1;
}

// 查找当前节点的邻居，进行处理后按规则push进优先级队列
void AStarManager::A_star_expand_neighbors(GridNode* GridNodePtr)
{
    for (int i=-1; i<=1; i++)
    {
        for (int j=-1; j<=1; j++)
        {
            for (int k=-1; k <= 1; k++)
            {
                if (i==0 && j==0 && k==0)
                    continue;
                Eigen::Vector3i temp_idx = GridNodePtr->idx + Eigen::Vector3i(i, j, k);
                GridNode* temp_ptr = GridNodeMap[temp_idx(0)][temp_idx(1)][temp_idx(2)];
               
                if (is_obstacle(GridNodePtr->idx) || temp_idx(0)<0 || temp_idx(0)>=max_x_idx \
                                                  || temp_idx(1)<0 || temp_idx(1)>=max_y_idx \
                                                  || temp_idx(2)<0 || temp_idx(2)>=max_z_idx \
                                                  || temp_ptr->is_close)
                    continue;    // 当检查到的邻居栅格：1.是障碍物 2.超出界限 3.在close_list时，跳过
                neighbor_ptr_set.emplace_back(temp_ptr);
                edge_cost_set.emplace_back((GridNodePtr->coord - temp_ptr->coord).norm());
                
            }
        }
    }

}










#endif
