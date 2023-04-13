#include "Grid_map_generator.h"

void GridmapGenerator::global_map_segmentation(pcl::PointCloud<PointType>::Ptr global_map)
{
    pcl::ExtractIndices<PointType> extractor;
    extractor.setInputCloud(global_map);
    pcl::PointIndices indices;
    for (size_t i = 0; i < global_map->points.size(); i++)
    {
        if (global_map->points[i].z > sensor_height)
        {
            indices.indices.emplace_back(i);
        }
        global_map->points[i].z = 0;
    }
    extractor.setIndices(boost::make_shared<pcl::PointIndices>(indices));
    extractor.setNegative(true); // true to remove the indices
    extractor.filter(*global_map);
    pcl::getMinMax3D(*global_map, global_map_min_pt, global_map_max_pt);
    GLX_SIZE = static_cast<size_t>((global_map_max_pt.x - global_map_min_pt.x) * grid_inv_resolution);
    GLY_SIZE = static_cast<size_t>((global_map_max_pt.y - global_map_min_pt.y) * grid_inv_resolution);
}

void GridmapGenerator::global_map_cluster(const pcl::PointCloud<PointType>::Ptr global_map)
{
    pcl::search::KdTree<PointType>::Ptr tree(new pcl::search::KdTree<PointType>);
    tree->setInputCloud(global_map);
    pcl::EuclideanClusterExtraction<PointType> euclidean_cluster_extractor;
    euclidean_cluster_extractor.setInputCloud(global_map);
    euclidean_cluster_extractor.setSearchMethod(tree);
    euclidean_cluster_extractor.setClusterTolerance(cluster_distance);
    euclidean_cluster_extractor.setMinClusterSize(MIN_CLUSTER_SIZE);
    euclidean_cluster_extractor.setMaxClusterSize(MAX_CLUSTER_SIZE);
    euclidean_cluster_extractor.extract(clusters_indices);
}

void GridmapGenerator::boundingbox_generation(PointType &min_pt, PointType &max_pt, pcl::PointCloud<PointType>::Ptr boundingbox)
{
    boundingbox->points.clear();
    PointType point;
    for (float i = min_pt.x; i <= max_pt.x; i += boundingbox_step)
    {
        point.x = i;
        point.y = min_pt.y;
        boundingbox->points.emplace_back(point);
        point.y = max_pt.y;
        boundingbox->points.emplace_back(point);
    }
    for (float i = min_pt.y; i <= max_pt.y; i += boundingbox_step)
    {
        point.y = i;
        point.x = min_pt.x;
        boundingbox->points.emplace_back(point);
        point.x = max_pt.x;
        boundingbox->points.emplace_back(point);
    }
}

void GridmapGenerator::boundingbox_sum_generation(const pcl::PointCloud<PointType>::Ptr global_map, pcl::PointCloud<PointType>::Ptr boundingbox_sum)
{
    size_t n = clusters_indices.size();
    boundingbox_min_points.resize(n);
    boundingbox_max_points.resize(n);
    for (size_t i = 0; i < n; i++)
    {
        pcl::PointCloud<PointType>::Ptr cloud_cluster(new pcl::PointCloud<PointType>());
        pcl::PointCloud<PointType>::Ptr bounding_box(new pcl::PointCloud<PointType>());
        PointType point;
        for (auto pit = clusters_indices[i].indices.begin(); pit != clusters_indices[i].indices.end(); ++pit)
        {
            point.x = global_map->points[*pit].x;
            point.y = global_map->points[*pit].y;
            point.z = global_map->points[*pit].z;
            cloud_cluster->points.emplace_back(point);
        }
        Eigen::Vector4f centroid; // 质心
        pcl::compute3DCentroid(*cloud_cluster, centroid);
        PointType min_pt, max_pt;
        pcl::getMinMax3D(*cloud_cluster, min_pt, max_pt);
        min_pt.x -= expansion_size;
        min_pt.y -= expansion_size;
        max_pt.x += expansion_size;
        max_pt.y += expansion_size;
        boundingbox_min_points[i] = min_pt;
        boundingbox_max_points[i] = max_pt;
        boundingbox_generation(min_pt, max_pt, bounding_box);
        *boundingbox_sum += *bounding_box;
    }
}

void GridmapGenerator::set_obstacles(pcl::PointCloud<PointType>::Ptr grid_map)
{
    obstacles_data.resize(GLX_SIZE * GLY_SIZE);
    for (size_t i = 0; i < GLX_SIZE; i++)
    {
        for (size_t j = 0; j < GLY_SIZE; j++)
        {
            float grid_min_x = global_map_min_pt.x + i * grid_resolution;
            float grid_min_y = global_map_min_pt.y + j * grid_resolution;
            float grid_max_x = global_map_min_pt.x + (i + 1) * grid_resolution;
            float grid_max_y = global_map_min_pt.y + (j + 1) * grid_resolution;
            size_t k = 0;
            for (; k < boundingbox_min_points.size(); k++)
            {
                if (grid_min_x > boundingbox_max_points[k].x || grid_min_y > boundingbox_max_points[k].y ||
                    grid_max_x < boundingbox_min_points[k].x || grid_max_y < boundingbox_min_points[k].y)
                {
                    continue;
                }
                else
                {
                    obstacles_data[i * GLY_SIZE + j] = 1;
                    PointType point;
                    point.x = i;
                    point.y = j;
                    grid_map->points.emplace_back(point);
                    break;
                }
            }
        }
    }
}

void ground_generation(const pcl::PointCloud<PointType>::Ptr input_map, pcl::PointCloud<PointType>::Ptr output_ground)
{
    double map_range_min_x = std::numeric_limits<double>::max();
    double map_range_max_x = -std::numeric_limits<double>::max();
    double map_range_min_y = std::numeric_limits<double>::max();
    double map_range_max_y = -std::numeric_limits<double>::max();
    int input_map_size = input_map->points.size();
    for (int i = 0; i < input_map_size; i++)
    {
        if (input_map->points[i].x < map_range_min_x)
        {
            map_range_min_x = input_map->points[i].x;
        }
        if (input_map->points[i].y < map_range_min_y)
        {
            map_range_min_y = input_map->points[i].y;
        }
        if (input_map->points[i].x > map_range_max_x)
        {
            map_range_max_x = input_map->points[i].x;
        }
        if (input_map->points[i].y > map_range_max_y)
        {
            map_range_max_y = input_map->points[i].y;
        }
    }
    PointType map_point;
    for (double i = map_range_min_x; i <= map_range_max_x; i += 0.4)
    {
        for (double j = map_range_min_y; j <= map_range_max_y; j += 0.4)
        {
            map_point.x = i;
            map_point.y = j;
            map_point.z = 2;
            output_ground->points.emplace_back(map_point);
        }
    }
    for (double i = map_range_min_x + 0.2; i <= map_range_max_x - 0.2; i += 0.4)
    {
        for (double j = map_range_min_y + 0.2; j <= map_range_max_y - 0.2; j += 0.4)
        {
            map_point.x = i;
            map_point.y = j;
            map_point.z = 2;
            output_ground->points.emplace_back(map_point);
        }
    }
}