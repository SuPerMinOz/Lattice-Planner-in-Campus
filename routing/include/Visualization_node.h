#ifndef __VISUALIZATION_NODE_H__
#define __VISUALIZATION_NODE_H__

#include "Basic_configuration.h"

struct callback_args
{
    // structure used to pass arguments to the callback function
    pcl::PointCloud<PointType>::Ptr clicked_points_2d;
    pcl::visualization::PCLVisualizer::Ptr viewerPtr;
};

#endif