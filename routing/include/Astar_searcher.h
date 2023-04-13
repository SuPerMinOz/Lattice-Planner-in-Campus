#ifndef __ASTART_SEARCHER_H__
#define __ASTART_SEARCHER_H__

#include <iostream>
#include <algorithm>
#include "Node.h"

// Astar路径规划类
class AstarPathFinder
{
private:
protected:
    enum class HeuristicFunctionType
    {
        Dijkstra = 0,
        Manhattan = 1,
        Euclidean = 2,
        Diagonal = 3,
        L_infty = 4
    };
    HeuristicFunctionType heuristic_function_type = HeuristicFunctionType::Diagonal;
    bool use_Tie_breaker = true;
    std::vector<std::vector<GridNodePtr>> GridNodeMap;
    Eigen::Vector2i goalIdx;
    int GLX_SIZE, GLY_SIZE, GLXY_SIZE;

    double resolution, inv_resolution;
    double gl_xl, gl_yl;
    double gl_xu, gl_yu;

    GridNodePtr terminatePtr;
    std::multimap<double, GridNodePtr> openSet;

    double getHeu(GridNodePtr node1, GridNodePtr node2);
    void AstarGetSucc(GridNodePtr currentPtr, std::vector<GridNodePtr> &neighborPtrSets, std::vector<double> &edgeCostSets);

    bool isOccupied(const int &idx_x, const int &idx_y) const;
    bool isOccupied(const Eigen::Vector2i &index) const;
    bool isFree(const int &idx_x, const int &idx_y) const;
    bool isFree(const Eigen::Vector2i &index) const;

    Eigen::Vector2d gridIndex2coord(const Eigen::Vector2i &index);
    Eigen::Vector2i coord2gridIndex(const Eigen::Vector2d &pt);

public:
    AstarPathFinder(){};
    ~AstarPathFinder(){};
    void AstarGraphSearch(Eigen::Vector2d start_pt, Eigen::Vector2d end_pt);
    void resetGrid(GridNodePtr ptr);
    void resetUsedGrids();

    void initGridMap(double _resolution, Eigen::Vector2d global_xy_l, Eigen::Vector2d global_xy_u, int max_x_id, int max_y_id);
    void setObs(const double coord_x, const double coord_y);
    std::vector<size_t> data;

    Eigen::Vector2d coordRounding(const Eigen::Vector2d &coord);
    std::vector<Eigen::Vector2d> getPath();
    std::vector<Eigen::Vector2d> getVisitedNodes();
};

#endif