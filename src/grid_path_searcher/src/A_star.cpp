#include <ros/ros.h>
#include <ros/console.h>
#include <cmath>

// #include "backward.hpp"
#include "A_star.h"

#ifdef USE_A_STAR

GridNode::GridNode(Eigen::Vector3i idx, Eigen::Vector3d coord)
{
    is_close = 0;
    is_path = 0;
    this->idx = idx;
    this->coord = coord;
    this->dir = Eigen::Vector3i::Zero();

    g_score = inf;
    f_score = inf;
    father = NULL;
}

// 初始化
void AStarManager::init(ros::NodeHandle &nh, double resolution, Eigen::Vector3d min_coord, Eigen::Vector3d max_coord, int max_x_idx, int max_y_idx, int max_z_idx)
{
    nh.param("path/resolution", path_resolution, 0.1);
    nh.param("path/delta_t", delta_t, path_resolution / 2.0);
    nh.param("path/delta_t", max_safecheck_iter, 4);

    this->resolution = resolution;
    inv_resolution = 1.0 / resolution; 
    
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
    max_yz_idx = max_y_idx * max_z_idx;
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

void AStarManager::reset_grid()
{
    for (int i = 0; i < max_x_idx; i++) {
        for (int j = 0; j < max_y_idx; j++) {
            for (int k = 0; k < max_z_idx; k++) {
                GridNode* node = GridNodeMap[i][j][k];
                node->g_score = inf;             // 重置为无穷大
                node->f_score = inf;             // 重置为无穷大
                node->father = nullptr;          // 清空父节点
                node->is_close = 0;              // 重置关闭状态
            }
        }
    }
}


// 返回某个栅格是否为障碍物栅格
inline bool AStarManager::is_obstacle(Eigen::Vector3i idx)
{
    uint8_t temp_is_out_of_range = idx(0)>=0 && idx(0)<max_x_idx && idx(1)>=0 && idx(1)<max_y_idx && idx(2)>=0 && idx(2)<max_z_idx;
    return temp_is_out_of_range && obstacle_map[idx(0)*max_yz_idx + idx(1)* max_z_idx + idx(2)];
}

// 下面两个函数功能一样的，只是命名和输入不一样，一个输入是一维的索引，另一个是三维的索引，目的是适配rviz插件的接口
void AStarManager::set_obstacle(Eigen::Vector3i idx)
{
    obstacle_map[idx(0)*max_yz_idx + idx(1)* max_z_idx + idx(2)] = 1;
}

void AStarManager::setObs(const double coord_x, const double coord_y, const double coord_z)
{
    if (coord_x < min_x_coord || coord_y < min_y_coord || coord_z < min_z_coord ||
        coord_x >= max_x_coord || coord_y >= max_y_coord || coord_z >= max_z_coord)
        return;

    int idx_x = static_cast<int>((coord_x - min_x_coord) * inv_resolution);
    int idx_y = static_cast<int>((coord_y - min_y_coord) * inv_resolution);
    int idx_z = static_cast<int>((coord_z - min_z_coord) * inv_resolution);

    obstacle_map[idx_x * max_yz_idx + idx_y * max_z_idx + idx_z] = 1;
}

inline Eigen::Vector3i AStarManager::coord2idx(Eigen::Vector3d coord)
{
    Eigen::Vector3i idx;
    idx <<  std::min(std::max(int((coord(0) - min_x_coord) * inv_resolution), 0), max_x_idx - 1),
            std::min(std::max(int((coord(1) - min_y_coord) * inv_resolution), 0), max_y_idx - 1),
            std::min(std::max(int((coord(2) - min_z_coord) * inv_resolution), 0), max_z_idx - 1);                  
    return idx;
}

inline Eigen::Vector3d AStarManager::idx2coord(Eigen::Vector3i idx)
{
    Eigen::Vector3d coord;
    coord(0) = ((double)idx(0) + 0.5) * resolution + min_x_coord;
    coord(1) = ((double)idx(1) + 0.5) * resolution + min_y_coord;
    coord(2) = ((double)idx(2) + 0.5) * resolution + min_z_coord;
    return coord;
}

// 启发式函数
inline double AStarManager::heuristics(Eigen::Vector3d start_coord, Eigen::Vector3d goal_coord)
{
    return (start_coord - goal_coord).norm();
}

// 查找当前节点的邻居，进行处理后按规则push进优先级队列
void AStarManager::A_star_expand_neighbors(GridNode* GridNodePtr)
{
    for (int i=-1; i<=1; i++){
        for (int j=-1; j<=1; j++){
            for (int k=-1; k <= 1; k++){
                if (i==0 && j==0 && k==0) {
                    continue;
                }
                    
                Eigen::Vector3i temp_idx = GridNodePtr->idx + Eigen::Vector3i(i, j, k);
                GridNode* temp_ptr = GridNodeMap[temp_idx(0)][temp_idx(1)][temp_idx(2)];
               
                if (is_obstacle(temp_idx) || temp_idx(0)<0 || temp_idx(0)>=max_x_idx \
                                          || temp_idx(1)<0 || temp_idx(1)>=max_y_idx \
                                          || temp_idx(2)<0 || temp_idx(2)>=max_z_idx \
                                          || temp_ptr->is_close) {
                    continue;    // 当检查到的邻居栅格：1.是障碍物 2.超出界限 3.在close_list时，跳过
                }
                    
                neighbor_ptr_set.emplace_back(temp_ptr);
                edge_cost_set.emplace_back((GridNodePtr->coord - temp_ptr->coord).norm());
                
            }
        }
    }

}

// A*搜索
bool AStarManager::A_star_search(Eigen::Vector3d start_coord, Eigen::Vector3d goal_coord)
{
    reset_grid();
    ros::Time start_time = ros::Time::now();
    Eigen::Vector3i start_idx = coord2idx(start_coord);
    Eigen::Vector3i end_idx = coord2idx(goal_coord);

    if (goal_coord.x() < min_x_coord  || goal_coord.y() < min_y_coord  || goal_coord.z() < min_z_coord  ||
        goal_coord.x() >= max_x_coord || goal_coord.y() >= max_y_coord || goal_coord.z() >= max_z_coord) {

        std::cout << "[A_star]: Target point is out of range" << std::endl;
        return false;
    }
    if (is_obstacle(end_idx)) {
        std::cout << "[A_star]: Target point is obstacle" << std::endl;
        return false;
    }

    GridNode* start_ptr = GridNodeMap[start_idx[0]][start_idx[1]][start_idx[2]];
    GridNode* end_ptr = GridNodeMap[end_idx[0]][end_idx[1]][end_idx[2]];
    GridNode* current_node_ptr = NULL;
    open_list.clear();

    start_ptr->coord = start_coord;
    start_ptr->g_score = 0;
    start_ptr->f_score = start_ptr->g_score + heuristics(start_ptr->coord, goal_coord);

    GridNodeMap[start_idx[0]][start_idx[1]][start_idx[2]]->is_close = 1;

    open_list.insert(std::make_pair(start_ptr->f_score, start_ptr));

    while (!open_list.empty()) {

        // 注意要清除用于临时存储的容器
        neighbor_ptr_set.clear();
        edge_cost_set.clear();
        // 弹出当前节点并准备扩展
        current_node_ptr = open_list.begin()->second;
        open_list.erase(open_list.begin());
        current_node_ptr->is_close = 1;

        if (current_node_ptr->idx == end_idx){
            final_node_ptr = current_node_ptr;
            ros::Time end_time = ros::Time::now();
            ROS_WARN("[A*]{sucess}  Time in A*  is %f ms, path cost if %f m", (end_time - start_time).toSec() * 1000.0, current_node_ptr->g_score);
            return true;
        }
        A_star_expand_neighbors(current_node_ptr);
        for (int i = 0; i < neighbor_ptr_set.size(); i++)
        {
            if (neighbor_ptr_set[i]->g_score <= current_node_ptr->g_score + edge_cost_set[i])
                continue;
            else
            {
                if (neighbor_ptr_set[i]->g_score == inf)
                {
                    neighbor_ptr_set[i]->father = current_node_ptr;
                    neighbor_ptr_set[i]->g_score = current_node_ptr->g_score + edge_cost_set[i];
                    neighbor_ptr_set[i]->f_score = neighbor_ptr_set[i]->g_score + heuristics(neighbor_ptr_set[i]->coord, goal_coord);
                    open_list.insert(std::make_pair(neighbor_ptr_set[i]->f_score, neighbor_ptr_set[i]));
                }
                else if (neighbor_ptr_set[i]->g_score > current_node_ptr->g_score + edge_cost_set[i])
                {
                    neighbor_ptr_set[i]->father = current_node_ptr;
                    neighbor_ptr_set[i]->g_score = current_node_ptr->g_score + edge_cost_set[i];
                    neighbor_ptr_set[i]->f_score = neighbor_ptr_set[i]->g_score + heuristics(neighbor_ptr_set[i]->coord, goal_coord);
                }               
                
            }
        }
    }
    std::cout << "[A_star]: The current path is a dead end" << std::endl;
    return false;
}

// 路径回溯
std::vector<Eigen::Vector3d> AStarManager::get_path()
{
    std::vector<Eigen::Vector3d> path;
    std::vector<GridNode*> grid_path;
    GridNode* ptr = final_node_ptr;
    // 从终点向前回溯
    while (ptr->father != NULL)
    {
        grid_path.push_back(ptr);
        path.push_back(ptr->coord);
        ptr = ptr->father;
    }  
    std::reverse(path.begin(), path.end());
    return path;
}

// 简化路径
std::vector<Eigen::Vector3d> AStarManager::path_simplify(const std::vector<Eigen::Vector3d> &path)
{
    // 计算全部路径点到路径首尾连线的距离最大值
    double dmax = 0;
    double d;
    int index = 0;
    int end = path.size();
    for (int i = 1; i < end - 1; i++) {
        d = calculate_d(path[i], path[0], path[end - 1]);
        if (d > dmax) {
            index = i;
            dmax = d;
        }
    }
    // 拆为两段
    std::vector<Eigen::Vector3d> subPath1;
    int j = 0;
    while(j<index+1){
        subPath1.push_back(path[j]);
        j++;
    }
    std::vector<Eigen::Vector3d> subPath2;
    while(j < int(path.size())){
        subPath2.push_back(path[j]);
        j++;
    }
    // 递归调用直到满足要求
    std::vector<Eigen::Vector3d> recPath1;
    std::vector<Eigen::Vector3d> recPath2;
    std::vector<Eigen::Vector3d> resultPath;
    if(dmax>path_resolution)
    {
        recPath1 = path_simplify(subPath1);
        recPath2 = path_simplify(subPath2);
    for(int i = 0; i < int(recPath1.size()); i++) {
        resultPath.push_back(recPath1[i]);
    }
        for(int i = 0; i < int(recPath2.size()); i++) {
        resultPath.push_back(recPath2[i]);
    }
    }else {
        if(path.size() > 1){
            resultPath.push_back(path[0]);
            resultPath.push_back(path[end-1]);
        }
        else {
            resultPath.push_back(path[0]);
        }       
    }
    return resultPath;
}

inline double AStarManager::calculate_d(const Eigen::Vector3d insert, const Eigen::Vector3d start, const Eigen::Vector3d end)
{
    Eigen::Vector3d line1 = end - start;
    Eigen::Vector3d line2 = insert - start;
    return double(line2.cross(line1).norm()/line1.norm());
}

int AStarManager::safeCheck(Traj_opt &traj_opt) {
    int unsafe_segment = -1;        // -1表示多项式轨迹安全无碰撞
    std::vector<Eigen::VectorXd> P_coef_vec = traj_opt.P_coef_vec;
    Eigen::MatrixX3d polyCoeff = traj_opt.resize_coeff(P_coef_vec);
    double delta_t = resolution / 1.5;
    double t = delta_t;
    Eigen::Vector3d advancePos;
    for(int i = 0; i < polyCoeff.rows() / traj_opt.p_num; i++)
    {
        while (t < traj_opt.time(i)) {
            advancePos = traj_opt.getPos(polyCoeff, i, t);
            if (is_obstacle(coord2idx(advancePos))) {
                unsafe_segment = i;
                break;
            }   
            t += delta_t;
        }
        if (unsafe_segment != -1) {
            std::cout << "segment " << i << " unsafe" << std::endl;
            break;
        } else {
            t = delta_t;
        }
    }
    return unsafe_segment;
}


// 测试
Eigen::Vector3d AStarManager::coordRounding(const Eigen::Vector3d &coord)
{
    return idx2coord(coord2idx(coord));
}

#endif
