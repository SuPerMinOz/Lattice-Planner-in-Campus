#include <chrono>
#include "Astar_searcher.h"

using namespace std;
using namespace Eigen;

// 栅格地图初始化(设定栅格索引值与世界坐标系对应关系)
void AstarPathFinder::initGridMap(double _resolution, Eigen::Vector2d global_xy_l, Eigen::Vector2d global_xy_u, int max_x_id, int max_y_id)
{
    gl_xl = global_xy_l(0);
    gl_yl = global_xy_l(1);
    gl_xu = global_xy_u(0);
    gl_yu = global_xy_u(1);

    GLX_SIZE = max_x_id;
    GLY_SIZE = max_y_id;
    GLXY_SIZE = GLX_SIZE * GLY_SIZE;

    resolution = _resolution;
    inv_resolution = 1.0 / _resolution;

    data.resize(GLXY_SIZE, 0);

    GridNodeMap.resize(GLX_SIZE);
    for (int i = 0; i < GLX_SIZE; i++)
    {
        GridNodeMap[i].resize(GLY_SIZE);
        for (int j = 0; j < GLY_SIZE; j++)
        {
            Vector2i tmpIdx(i, j);
            Vector2d pos = gridIndex2coord(tmpIdx);
            GridNodeMap[i][j] = new GridNode(tmpIdx, pos);
        }
    }
}

void AstarPathFinder::resetGrid(GridNodePtr ptr)
{
    ptr->id = 0;
    ptr->cameFrom = nullptr;
    ptr->gScore = inf;
    ptr->fScore = inf;
}

void AstarPathFinder::resetUsedGrids()
{
    for (int i = 0; i < GLX_SIZE; i++)
    {
        for (int j = 0; j < GLY_SIZE; j++)
        {
            resetGrid(GridNodeMap[i][j]);
        }
    }
}

void AstarPathFinder::setObs(const double coord_x, const double coord_y)
{
    if (coord_x < gl_xl || coord_y < gl_yl || coord_x >= gl_xu || coord_y >= gl_yu)
    {
        return;
    }
    int idx_x = static_cast<int>((coord_x - gl_xl) * inv_resolution);
    int idx_y = static_cast<int>((coord_y - gl_yl) * inv_resolution);
    data[idx_x * GLY_SIZE + idx_y] = 1;
}

std::vector<Eigen::Vector2d> AstarPathFinder::getVisitedNodes()
{
    vector<Vector2d> visited_nodes;
    for (int i = 0; i < GLX_SIZE; i++)
    {
        for (int j = 0; j < GLY_SIZE; j++)
        {
            // if(GridNodeMap[i][j][k]->id != 0) // visualize all nodes in open and close list
            if (GridNodeMap[i][j]->id == -1) // visualize nodes in close list only
            {
                visited_nodes.push_back(GridNodeMap[i][j]->coord);
            }
        }
    }

    std::cout << "This search visited_nodes size : " << visited_nodes.size() << std::endl;
    return visited_nodes;
}

Vector2d AstarPathFinder::gridIndex2coord(const Vector2i &index)
{
    Vector2d pt;
    pt(0) = ((double)index(0) + 0.5) * resolution + gl_xl;
    pt(1) = ((double)index(1) + 0.5) * resolution + gl_yl;
    return pt;
}

Vector2i AstarPathFinder::coord2gridIndex(const Vector2d &pt)
{
    Vector2i idx;
    idx << min(max(int((pt(0) - gl_xl) * inv_resolution), 0), GLX_SIZE - 1),
        min(max(int((pt(1) - gl_yl) * inv_resolution), 0), GLY_SIZE - 1);

    return idx;
}

Eigen::Vector2d AstarPathFinder::coordRounding(const Eigen::Vector2d &coord)
{
    return gridIndex2coord(coord2gridIndex(coord));
}

inline bool AstarPathFinder::isOccupied(const Eigen::Vector2i &index) const
{
    return isOccupied(index(0), index(1));
}

inline bool AstarPathFinder::isFree(const Eigen::Vector2i &index) const
{
    return isFree(index(0), index(1));
}

inline bool AstarPathFinder::isOccupied(const int &idx_x, const int &idx_y) const
{
    return (idx_x >= 0 && idx_x < GLX_SIZE && idx_y >= 0 && idx_y < GLY_SIZE &&
            (data[idx_x * GLY_SIZE + idx_y] == 1));
}

inline bool AstarPathFinder::isFree(const int &idx_x, const int &idx_y) const
{
    return (idx_x >= 0 && idx_x < GLX_SIZE && idx_y >= 0 && idx_y < GLY_SIZE &&
            (data[idx_x * GLY_SIZE + idx_y] < 1));
}

// 扩展节点
inline void AstarPathFinder::AstarGetSucc(GridNodePtr currentPtr, vector<GridNodePtr> &neighborPtrSets, vector<double> &edgeCostSets)
{
    neighborPtrSets.clear();
    edgeCostSets.clear();

    Eigen::Vector2i current_index = currentPtr->index;
    int current_x = current_index[0];
    int current_y = current_index[1];

    int n_x, n_y, n_z;
    GridNodePtr current_neighbor_ptr = NULL;
    double edge_cost = 0.0;

    for (int i = -1; i <= 1; ++i)
    {
        for (int j = -1; j <= 1; ++j)
        {
            if (i == 0 && j == 0)
            {
                continue;
            }

            n_x = current_x + i;
            n_y = current_y + j;

            if ((n_x < 0) || (n_y < 0) || (n_z < 0) || (n_x > GLX_SIZE - 1) || (n_y > GLY_SIZE - 1))
            {
                continue;
            }

            if (isOccupied(n_x, n_y))
            {
                continue;
            }

            current_neighbor_ptr = GridNodeMap[n_x][n_y];
            edge_cost = getHeu(currentPtr, current_neighbor_ptr);
            neighborPtrSets.emplace_back(current_neighbor_ptr);
            edgeCostSets.emplace_back(edge_cost);
        }
    }
}

// 启发函数计算
double AstarPathFinder::getHeu(GridNodePtr node1, GridNodePtr node2)
{
    // #define sqrt_3 1.7320508
    // #define sqrt_2 1.4142136
    double h = 0.0;
    Eigen::Vector2i start_index = node1->index;
    Eigen::Vector2i end_index = node2->index;

    if (heuristic_function_type == HeuristicFunctionType::Dijkstra)
    {
        h = 0.0;
    }
    else if (heuristic_function_type == HeuristicFunctionType::Manhattan)
    {
        // h = (node2->index.cast<double>() - node1->index.cast<double>()).lpNorm<1>();
        double dx = abs((double)(start_index(0) - end_index(0)));
        double dy = abs((double)(start_index(1) - end_index(1)));
        h = dx + dy;
    }
    else if (heuristic_function_type == HeuristicFunctionType::Euclidean)
    {
        // h = (node2->index.cast<double>() - node1->index.cast<double>()).norm();
        double dx = abs((double)(start_index(0) - end_index(0)));
        double dy = abs((double)(start_index(1) - end_index(1)));
        h = std::sqrt((std::pow(dx, 2.0) + std::pow(dy, 2.0)));
    }
    else if (heuristic_function_type == HeuristicFunctionType::Diagonal)
    {
        // double dx = std::abs(node1->index.x() - node2->index.x());
        // double dy = std::abs(node1->index.y() - node2->index.y());
        // double min_xy = std::min({dx, dy});
        // h = (dx + dy) + (sqrt_2 - 2) * min_xy;
        double distance[2];
        distance[0] = abs((double)(start_index(0) - end_index(0)));
        distance[1] = abs((double)(start_index(1) - end_index(1)));
        std::sort(distance, distance + 2);
        h = distance[0] + distance[1] + (std::sqrt(2.0) - 2) * distance[0];
    }
    else if (heuristic_function_type == HeuristicFunctionType::L_infty)
    {
        double dx = abs((double)(start_index(0) - end_index(0)));
        double dy = abs((double)(start_index(1) - end_index(1)));
        h = std::max({dx, dy});
    }

    if (use_Tie_breaker)
    {
        h *= 1.01; // deterministic random number
    }

    return h;
}

void AstarPathFinder::AstarGraphSearch(Vector2d start_pt, Vector2d end_pt)
{
    auto T_start = std::chrono::high_resolution_clock::now();
    terminatePtr = nullptr;

    double dx = start_pt(0) - end_pt(0);
    double dy = start_pt(1) - end_pt(1);
    double distance = std::sqrt(std::pow(dx, 2) + std::pow(dy, 2));

    if (distance <= resolution)
    {
        std::cerr << "\033[1;31m[A*] failed, the distance between the setup and the goal is lees than grid length. \033[0m" << std::endl;
        return;
    }

    // index of start_point and end_point
    Vector2i start_idx = coord2gridIndex(start_pt);
    Vector2i end_idx = coord2gridIndex(end_pt);
    if (isOccupied(start_idx))
    {
        std::cerr << "\033[1;31m[A*] failed, the setup is Occupied, please choose a new setup. \033[0m" << std::endl;
        return;
    }
    goalIdx = end_idx;

    if (isOccupied(goalIdx))
    {
        std::cerr << "\033[1;31m[A*] failed, the goal is Occupied, please choose a new goal. \033[0m" << std::endl;
        return;
    }

    start_pt = gridIndex2coord(start_idx);
    end_pt = gridIndex2coord(end_idx);
    // Initialize the pointers of struct GridNode which represent start node and goal node
    GridNodePtr startPtr = GridNodeMap[start_idx(0)][start_idx(1)];
    GridNodePtr endPtr = GridNodeMap[end_idx(0)][end_idx(1)];
    std::cout << "\033[1;32mStart Node position: (" << startPtr->coord(0) << " " << startPtr->coord(1) << ") \033[0m" << std::endl;
    std::cout << "\033[1;32mGoal Node position: (" << endPtr->coord(0) << " " << endPtr->coord(1) << ")\033[0m" << std::endl;
    openSet.clear();

    GridNodePtr currentPtr = nullptr;
    GridNodePtr neighborPtr = nullptr;

    startPtr->gScore = 0;
    startPtr->fScore = getHeu(startPtr, endPtr);
    startPtr->id = 1;
    startPtr->coord = start_pt;
    startPtr->nodeMapIt = openSet.insert(make_pair(startPtr->fScore, startPtr));

    vector<GridNodePtr> neighborPtrSets;
    vector<double> edgeCostSets;

    while (!openSet.empty())
    {
        currentPtr = openSet.begin()->second;
        currentPtr->id = -1;
        openSet.erase(openSet.begin());

        AstarGetSucc(currentPtr, neighborPtrSets, edgeCostSets);

        for (int i = 0; i < (int)neighborPtrSets.size(); i++)
        {
            neighborPtr = neighborPtrSets[i];
            double gh = currentPtr->gScore + edgeCostSets[i];
            double fh = gh + getHeu(neighborPtr, endPtr);

            if (neighborPtr->id == 0)
            {
                neighborPtr->gScore = gh;
                neighborPtr->fScore = fh;
                neighborPtr->cameFrom = currentPtr;
                neighborPtr->nodeMapIt = openSet.insert(make_pair(neighborPtr->fScore, neighborPtr)); // 此处注意一定要先计算和赋值完f再加入(按照fscore升序排列)

                if (neighborPtr->index == goalIdx)
                {
                    auto T_end = std::chrono::high_resolution_clock::now();
                    auto T_duration = std::chrono::duration_cast<std::chrono::microseconds>(T_end - T_start);
                    terminatePtr = neighborPtr;
                    std::cout << "\033[1;32m[A*] succeeded, Time in A star is "
                              << (double)T_duration.count() / 1e3 << "ms, path cost " << currentPtr->gScore * resolution << "m. \033[0m" << std::endl;
                    return;
                }
                else
                {
                    neighborPtr->id = 1;
                    continue;
                }
            }
            else if (neighborPtr->id == 1)
            {
                if (neighborPtr->gScore > gh)
                {
                    neighborPtr->gScore = gh;
                    neighborPtr->fScore = fh;
                    neighborPtr->cameFrom = currentPtr;
                    openSet.erase(neighborPtr->nodeMapIt);
                    neighborPtr->nodeMapIt = openSet.insert(make_pair(neighborPtr->fScore, neighborPtr));
                }
            }
            else
            {
                continue;
            }
        }
    }
    auto T_end = std::chrono::high_resolution_clock::now();
    auto T_duration = std::chrono::duration_cast<std::chrono::microseconds>(T_end - T_start);
    if ((double)T_duration.count() > 1e6)
    {
        std::cout << "\033[1;32m[A*] failed, Time consume in Astar path finding is " << (double)T_duration.count() / 1e3 << "ms. \033[0m" << std::endl;
    }
}

vector<Vector2d> AstarPathFinder::getPath()
{
    vector<Vector2d> path;
    vector<Vector2i> path_idx;
    vector<GridNodePtr> gridPath;

    GridNodePtr grid_node_ptr = terminatePtr;
    if (grid_node_ptr == nullptr)
    {
        return path;
    }

    while (grid_node_ptr->cameFrom != nullptr)
    {
        gridPath.emplace_back(grid_node_ptr);
        grid_node_ptr = grid_node_ptr->cameFrom;
    }

    std::cout << "This search path size : " << gridPath.size() << std::endl;

    if (gridPath.size() <= 1)
    {
        return path;
    }

    for (auto ptr : gridPath)
    {
        // path.push_back(ptr->coord);
        path_idx.push_back(ptr->index);
    }
    reverse(path_idx.begin(), path_idx.end());

    int n = path_idx.size(), j = 0, k = 0;
    double move_step = resolution / 50;
    for (int i = 0; i < n; i++)
    {
        for (j = n - 1; j > i; j--)
        {
            double dy = path_idx[j](1) - path_idx[i](1);
            double dx = path_idx[j](0) - path_idx[i](0);
            double yaw = std::atan2(dy, dx);
            double distance = std::sqrt(std::pow(dx, 2) + std::pow(dy, 2));
            int counter = distance / move_step;
            double distance_sum = 0;
            for (k = 0; k <= counter; k++)
            {
                double path_x = k * move_step * cos(yaw);
                double path_y = k * move_step * sin(yaw);
                int idx_x = path_x < 0 ? path_x - 1 : path_x;
                int idx_y = path_y < 0 ? path_y - 1 : path_y;
                distance_sum += move_step;

                if (data[(path_idx[i](0) + idx_x) * GLY_SIZE + path_idx[i](1) + idx_y] == 1)
                {
                    break;
                }
            }

            if (k > counter)
            {
                break;
            }
        }

        if (i == 0)
        {
            path.push_back(gridIndex2coord(path_idx[i]));
        }
        if (j == i)
        {
            break;
        }
        path.push_back(gridIndex2coord(path_idx[j]));
        i = j - 1;
    }
    while (j < n - 1)
    {
        path.push_back(gridIndex2coord(path_idx[j]));
        j += 2;
    }

    return path;
}