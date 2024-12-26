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

    grid_num = max_x_idx * max_y_idx * max_z_idx;

    data = new uint8_t[grid_num];
    memset(data, 0, grid_num * sizeof(uint8_t));

    GridNodeMap = new GridNode ***[max_x_idx];
    for (int i = 0; i < max_x_idx; i++)
    {
        GridNodeMap[i] = new GridNode **[max_y_idx];
        for (int j = 0; j < max_y_idx; j++)
        {
            GridNodeMap[i][j] = new GridNode *[max_z_idx];
            for (int k = 0; k < max_z_idx; k++)
            {
                Eigen::Vector3i tem_idx(i, j, k);
                Eigen::Vector3d tem_coord = idx2coord(tem_idx);
                GridNodeMap[i][j][k] = new GridNode(tem_idx ,tem_coord);
            }
        }
    }
}

// 返回某个栅格是否为障碍物栅格
bool AStarManager::is_obstacle(Eigen::vector3i idx)
{
    uint8_t = idx(0)>=0 && idx(0)<max_x_idx && idx(1)>=0 && idx(1)<max_y_idx && idx(2)>=0 && idx(2)<max_z_idx;
    return uint8_t && data[idx(0)*max_yz_idx + idx(1)* max_z_idx + idx(2)];
}

void AStarManager::set_obstacle(Eigen::vector3i idx)
{
    data[idx(0)*max_yz_idx + idx(1)* max_z_idx + idx(2)] = 1
}

//











#endif
