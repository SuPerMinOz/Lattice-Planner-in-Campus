#ifndef __NODE_H__
#define __NODE_H__

#include <Eigen/Eigen>

#define inf (1 >> 20)

struct GridNode;
typedef GridNode *GridNodePtr;

struct GridNode
{
    int id; // 1--> open set, -1 --> closed set
    Eigen::Vector2i index;
    Eigen::Vector2d coord;
    Eigen::Vector2i dir; // direction of expanding

    double gScore, fScore;
    GridNodePtr cameFrom;

    std::multimap<double, GridNodePtr>::iterator nodeMapIt;

    GridNode(Eigen::Vector2i _index, Eigen::Vector2d _coord)
        : id(0), index(_index), coord(_coord), dir(Eigen::Vector2i::Zero()), gScore(inf), fScore(inf), cameFrom(nullptr){};

    GridNode(){};
    ~GridNode(){};
};

#endif
