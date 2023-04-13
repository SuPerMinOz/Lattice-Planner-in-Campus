#ifndef __GRID_MAP_GENERATION__
#define __GRID_MAP_GENERATION__

#include "Basic_configuration.h"

class GridmapGenerator
{
private:
    double sensor_height;
    double cluster_distance;
    int MIN_CLUSTER_SIZE;
    int MAX_CLUSTER_SIZE;
    double expansion_size;
    double boundingbox_step;

protected:
    std::vector<PointType> boundingbox_min_points, boundingbox_max_points; // 障碍物聚类boundingbox尺寸信息
    std::vector<pcl::PointIndices> clusters_indices;                       // 聚类结果

public:
    GridmapGenerator() : sensor_height(0.9), cluster_distance(0.36), MIN_CLUSTER_SIZE(1), MAX_CLUSTER_SIZE(250), expansion_size(0.3), boundingbox_step(0.2), grid_resolution(0.4), grid_inv_resolution(1 / grid_resolution), GLX_SIZE(0), GLY_SIZE(0){};
    ~GridmapGenerator(){};
    void global_map_segmentation(pcl::PointCloud<PointType>::Ptr global_map);
    void global_map_cluster(const pcl::PointCloud<PointType>::Ptr global_map);
    void boundingbox_generation(PointType &min_pt, PointType &max_pt, pcl::PointCloud<PointType>::Ptr bounding_box);
    void boundingbox_sum_generation(const pcl::PointCloud<PointType>::Ptr global_map, pcl::PointCloud<PointType>::Ptr boundingbox_sum);
    void set_obstacles(pcl::PointCloud<PointType>::Ptr grid_map);

    double grid_resolution, grid_inv_resolution;
    size_t GLX_SIZE, GLY_SIZE;
    PointType global_map_min_pt, global_map_max_pt; // 全局地图尺寸信息
    std::vector<size_t> obstacles_data;             // 障碍物位置信息
};

void ground_generation(const pcl::PointCloud<PointType>::Ptr input_map, pcl::PointCloud<PointType>::Ptr output_ground);

#endif