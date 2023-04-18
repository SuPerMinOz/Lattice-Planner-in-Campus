#include <mutex>
#include <chrono>
#include "Grid_map_generator.h"
#include "Astar_searcher.h"
#include "Spline.h"
#include "Reference_line.h"
#include "Visualization_node.h"

std::mutex cloud_mutex;

auto grid_map_generator = std::make_shared<GridmapGenerator>();
auto global_path_searcher = std::make_shared<AstarPathFinder>();
auto reference_line = std::make_shared<ReferenceLine>();
boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("viewer"));

double peak = 0, foot = 0;

pcl::PointCloud<PointType>::Ptr global_map(new pcl::PointCloud<PointType>());
pcl::PointCloud<PointType>::Ptr ground_map(new pcl::PointCloud<PointType>());
pcl::PointCloud<PointType>::Ptr boundingbox_sum(new pcl::PointCloud<PointType>());
pcl::PointCloud<PointType>::Ptr grid_map(new pcl::PointCloud<PointType>());
pcl::PointCloud<PointType>::Ptr astar_path(new pcl::PointCloud<PointType>());
pcl::PointCloud<PointType>::Ptr global_path(new pcl::PointCloud<PointType>());
pcl::PointCloud<PointType>::Ptr virtual_barrier(new pcl::PointCloud<PointType>());
pcl::PointCloud<PointType>::Ptr virtual_barrier_sum(new pcl::PointCloud<PointType>());

void gridmap_generation(pcl::PointCloud<PointType>::Ptr global_map,
                        pcl::PointCloud<PointType>::Ptr boundingbox_sum,
                        pcl::PointCloud<PointType>::Ptr grid_map)
{
    grid_map_generator->global_map_segmentation(global_map);
    grid_map_generator->global_map_cluster(global_map);
    grid_map_generator->boundingbox_sum_generation(global_map, boundingbox_sum);
    grid_map_generator->set_obstacles(grid_map);

    double resolution = grid_map_generator->grid_resolution;
    Eigen::Vector2d global_xy_l(grid_map_generator->global_map_min_pt.x, grid_map_generator->global_map_min_pt.y);
    Eigen::Vector2d global_xy_u(grid_map_generator->global_map_max_pt.x, grid_map_generator->global_map_max_pt.y);
    peak = grid_map_generator->global_map_max_pt.z, foot = grid_map_generator->global_map_min_pt.z;
    int max_x_id = grid_map_generator->GLX_SIZE, max_y_id = grid_map_generator->GLY_SIZE;

    global_path_searcher->initGridMap(resolution, global_xy_l, global_xy_u, max_x_id, max_y_id);
    global_path_searcher->data.assign(grid_map_generator->obstacles_data.begin(), grid_map_generator->obstacles_data.end());
}

double NormalizeAngle(const double angle)
{
    double a = std::fmod(angle + M_PI, 2.0 * M_PI);
    if (a < 0.0)
    {
        a += (2.0 * M_PI);
    }
    return a - M_PI;
}

int sum_of_modes = 3, current_mode = 0;
void astarpath_searching(const Eigen::Vector2d start_pt, const Eigen::Vector2d target_pt,
                         pcl::PointCloud<PointType>::Ptr astar_path)
{
    if (current_mode == 0)
    {
        // Call A* to search for a path
        global_path_searcher->AstarGraphSearch(start_pt, target_pt);
        // Retrieve the path
        auto astar_result = global_path_searcher->getPath();
        // auto visited_nodes = global_path_searcher->getVisitedNodes();
        // Reset map for next call
        global_path_searcher->resetUsedGrids();

        astar_path->points.clear();
        for (auto pt : astar_result)
        {
            PointType pathpoint;
            pathpoint.x = pt(0);
            pathpoint.y = pt(1);
            pathpoint.z = 0;
            astar_path->points.emplace_back(pathpoint);
        }
    }
    else if (current_mode == 1)
    {
        current_mode = 0;
        astar_path->points.clear();
        PointType pathpoint;
        pathpoint.z = 0;
        pathpoint.x = start_pt(0);
        pathpoint.y = start_pt(1);
        astar_path->points.emplace_back(pathpoint);
        pathpoint.x = target_pt(0);
        pathpoint.y = target_pt(1);
        astar_path->points.emplace_back(pathpoint);
    }
}

double spiral_step = 0.8, curve_angle = 30;
void globalpath_generating(const pcl::PointCloud<PointType>::Ptr astar_path, pcl::PointCloud<PointType>::Ptr global_path,
                           VEC_1D &referenceline_x, VEC_1D &referenceline_y)
{
    int path_length = astar_path->points.size();
    if (path_length <= 1)
    {
        std::cerr << "\033[1;31m The astar result has less than two points. \033[0m" << std::endl;
        return;
    }

    std::cout << "[Astar  Result]"
              << "Start: (" << astar_path->points.front().x << ", " << astar_path->points.front().y << ") "
              << "End: (" << astar_path->points.back().x << ", " << astar_path->points.back().y << ") " << std::endl;

    pcl::PointCloud<PointType>::Ptr init_nodes(new pcl::PointCloud<PointType>());
    pcl::PointCloud<PointType>::Ptr final_nodes(new pcl::PointCloud<PointType>());

    for (int i = 1; i < path_length; i++)
    {
        double dy = astar_path->points[i].y - astar_path->points[i - 1].y;
        double dx = astar_path->points[i].x - astar_path->points[i - 1].x;
        double yaw = atan2(dy, dx);
        double distance = sqrt(pow(dx, 2) + pow(dy, 2));
        for (double offset = 0; offset < distance; offset += spiral_step)
        {
            PointType line_point;
            double path_temp_x = offset * cos(yaw);
            double path_temp_y = offset * sin(yaw);
            line_point.x = astar_path->points[i - 1].x + path_temp_x;
            line_point.y = astar_path->points[i - 1].y + path_temp_y;
            init_nodes->points.push_back(line_point);
        }
        if (i == path_length - 1)
        {
            init_nodes->points.push_back(astar_path->points[path_length - 1]);
        }
    }
    int ind_turn = 0;
    path_length = init_nodes->points.size();
    for (int i = 0; i < path_length; i++)
    {
        if (i < path_length - 2 && i > 1)
        {
            double dy_back = init_nodes->points[i].y - init_nodes->points[i - 1].y;
            double dx_back = init_nodes->points[i].x - init_nodes->points[i - 1].x;
            double yaw_back = atan2(dy_back, dx_back);

            double dy_front = init_nodes->points[i + 1].y - init_nodes->points[i].y;
            double dx_front = init_nodes->points[i + 1].x - init_nodes->points[i].x;
            double yaw_front = atan2(dy_front, dx_front);
            if (fabs(yaw_front - yaw_back) > (curve_angle * M_PI / 180))
            {
                PointType point_front;
                point_front.x = (init_nodes->points[i].x + init_nodes->points[i - 1].x) / 2.0;
                point_front.y = (init_nodes->points[i].y + init_nodes->points[i - 1].y) / 2.0;
                PointType point_back;
                point_back.x = (init_nodes->points[i].x + init_nodes->points[i + 1].x) / 2.0;
                point_back.y = (init_nodes->points[i].y + init_nodes->points[i + 1].y) / 2.0;
                PointType front_point_back;
                front_point_back.x = (point_front.x + point_back.x) / 2.0;
                front_point_back.y = (point_front.y + point_back.y) / 2.0;

                PointType center_point_front;
                center_point_front.x = (init_nodes->points[i - 1].x + point_front.x) / 2.0;
                center_point_front.y = (init_nodes->points[i - 1].y + point_front.y) / 2.0;
                PointType center_point_mid_front;
                center_point_mid_front.x = (front_point_back.x + point_front.x) / 2.0;
                center_point_mid_front.y = (front_point_back.y + point_front.y) / 2.0;
                PointType center_point_mid_back;
                center_point_mid_back.x = (front_point_back.x + point_back.x) / 2.0;
                center_point_mid_back.y = (front_point_back.y + point_back.y) / 2.0;
                PointType center_point_back;
                center_point_back.x = (init_nodes->points[i + 1].x + point_back.x) / 2.0;
                center_point_back.y = (init_nodes->points[i + 1].y + point_back.y) / 2.0;

                double dy_front_front = init_nodes->points[i + 1].y - init_nodes->points[i + 2].y;
                double dx_front_front = init_nodes->points[i + 2].x - init_nodes->points[i + 1].x;
                double yaw_front_front = atan2(-dy_front_front, dx_front_front);
                if (ind_turn == 1)
                {
                    if (fabs(yaw_front - yaw_front_front) > (curve_angle * M_PI / 180))
                    {
                        final_nodes->points.push_back(center_point_mid_back);
                    }
                    else
                    {
                        final_nodes->points.push_back(center_point_mid_back);
                        final_nodes->points.push_back(center_point_back);
                    }
                }
                else
                {
                    if (fabs(yaw_front - yaw_front_front) > (30 * M_PI / 180))
                    {
                        final_nodes->points.push_back(center_point_front);
                        final_nodes->points.push_back(center_point_mid_front);
                        final_nodes->points.push_back(center_point_mid_back);
                    }
                    else
                    {
                        final_nodes->points.push_back(center_point_front);
                        final_nodes->points.push_back(center_point_mid_front);
                        final_nodes->points.push_back(center_point_mid_back);
                        final_nodes->points.push_back(center_point_back);
                    }
                }
                ind_turn = 1;
            }
            else
            {
                ind_turn = 0;
                final_nodes->points.push_back(init_nodes->points[i]);
            }
        }
        else
        {
            final_nodes->points.push_back(init_nodes->points[i]);
        }
    }
    ind_turn = 0;

    global_path->points.clear();
    global_path->points.push_back(final_nodes->points.front());

    path_length = final_nodes->points.size();
    for (int j = 0; j < path_length; j++)
    {
        double plot_x = final_nodes->points[j].x;
        double plot_y = final_nodes->points[j].y;
        referenceline_x.push_back(plot_x);
        referenceline_y.push_back(plot_y);
    }
}

void generate_reference_line(Spline2D &Trajectory, std::shared_ptr<ReferenceLine> reference_line, pcl::PointCloud<PointType>::Ptr global_path)
{
    reference_line->trajectory_points.clear();
    auto T_start = std::chrono::high_resolution_clock::now();
    for (double i = 0.05; i <= Trajectory.s.back(); i += 0.05)
    {
        POINT_2D point = Trajectory.calc_postion(i);
        PointType pt;
        pt.x = point[0];
        pt.y = point[1];
        ReferencePoint plot;
        plot.x = point[0];
        plot.y = point[1];
        plot.s = i;
        plot.theta = Trajectory.calc_yaw(i);
        std::cout << "s " << i << " x " << point[0] << " y " << point[1] << " yaw " << plot.theta << std::endl;

        reference_line->trajectory_points.emplace_back(plot);

        if (isnan(pt.x) || isnan(pt.y))
        {
            std::cerr << "\033[1;31m The Spline path exist nan point. \033[0m" << std::endl;
        }
        global_path->points.push_back(pt);
    }
    std::cout << "The reference line has " << reference_line->trajectory_points.size() << " trajectory points." << std::endl;
    std::cout << "[Spline Result]"
              << " Start: (" << global_path->points.front().x << ", " << global_path->points.front().y << ") "
              << " End: (" << global_path->points.back().x << ", " << global_path->points.back().y << ") " << std::endl;

    auto T_end = std::chrono::high_resolution_clock::now();
    auto T_duration = std::chrono::duration_cast<std::chrono::microseconds>(T_end - T_start);
    std::cout << "\033[1;32m[Spline] succeeded, Time in Spline is "
              << (double)T_duration.count() / 1e3 << "ms. \033[0m" << std::endl;
}

void update_global_map(pcl::PointCloud<PointType>::Ptr global_map,
                       pcl::PointCloud<PointType>::Ptr virtual_barrier,
                       pcl::PointCloud<PointType>::Ptr virtual_barrier_sum)
{
    *global_map += *virtual_barrier;
    global_map->height = 1;
    global_map->width = global_map->points.size();
    pcl::io::savePCDFileASCII("./global_map.pcd", *global_map);

    *virtual_barrier_sum += *virtual_barrier;
    virtual_barrier_sum->height = 1;
    virtual_barrier_sum->width = virtual_barrier_sum->points.size();
    pcl::io::savePCDFileASCII("./virtual_barrier.pcd", *virtual_barrier_sum);
}

void virtual_barrier_generation(PointType &start_point, PointType &end_point, pcl::PointCloud<PointType>::Ptr virtual_barrier)
{
    double dy = end_point.y - start_point.y;
    double dx = end_point.x - start_point.x;
    double yaw = std::atan2(dy, dx);
    double distance = std::sqrt(std::pow(dx, 2) + std::pow(dy, 2));
    double level_step = std::max(0.1, (peak - foot) / 2.0);
    for (double i = foot; i <= peak; i += level_step)
    {
        PointType barrier_point;
        barrier_point.z = i;
        for (double j = 0; j <= distance; j += 0.4)
        {
            barrier_point.x = start_point.x + j * cos(yaw);
            barrier_point.y = start_point.y + j * sin(yaw);
            virtual_barrier->points.emplace_back(barrier_point);
        }
    }
}

void mode_select_callback(const pcl::visualization::KeyboardEvent &event, void *args)
{
    // struct callback_args *data = (struct callback_args *)args;
    if (event.getKeySym() == "p" && event.keyDown())
    {
        current_mode += 1;
        current_mode %= sum_of_modes;
        switch (current_mode)
        {
        case 0:
            std::cout << "[Mode Switch] --> Pick Point with Astar !" << std::endl;
            break;
        case 1:
            std::cout << "[Mode Switch] --> Pick Point directly !" << std::endl;
            break;
        case 2:
            std::cout << "[Mode Switch] --> Generate Virtual Barriers !" << std::endl;
            break;
        default:
            std::cout << "[Mode Switch] --> Invalid Mode !" << std::endl;
            break;
        }
    }
}

int last_pick_point_idx = -1;
void picking_point_callback(const pcl::visualization::PointPickingEvent &event, void *args)
{
    struct callback_args *data = (struct callback_args *)args;
    if (event.getPointIndex() == -1 || event.getPointIndex() == last_pick_point_idx)
    {
        std::cerr << "\033[1;31m 本次点云拾取失败 请重新选取(无效点) \033[0m" << std::endl;
        return;
    }
    last_pick_point_idx = event.getPointIndex();
    PointType current_point;
    event.getPoint(current_point.x, current_point.y, current_point.z);
    if (current_point.z != 2)
    {
        std::cerr << "\033[1;31m 本次点云拾取失败 请重新选取(地图点) \033[0m" << std::endl;
        return;
    }
    current_point.z = 0;
    std::cout << " 本次点云拾取成功 坐标信息 " << current_point.x << " " << current_point.y << std::endl;
    data->clicked_points_2d->points.emplace_back(current_point);

    int n = data->clicked_points_2d->points.size();
    if (n >= 2)
    {
        PointType start_point = data->clicked_points_2d->points[n - 2];
        PointType end_point = data->clicked_points_2d->points.back();
        if (current_mode == 0 || current_mode == 1)
        {
            Eigen::Vector2d start_pt(start_point.x, start_point.y);
            Eigen::Vector2d end_pt(end_point.x, end_point.y);
            astarpath_searching(start_pt, end_pt, astar_path);
            VEC_1D referenceline_x, referenceline_y;
            globalpath_generating(astar_path, global_path, referenceline_x, referenceline_y);
            if (referenceline_x.size() == referenceline_y.size() && referenceline_x.size() >= 2)
            {
                Spline2D Trajectory(referenceline_x, referenceline_y);
                generate_reference_line(Trajectory, reference_line, global_path);
            }
            else
            {
                global_path->points.clear();
                std::cerr << "\033[1;31m The global path has less than two points. \033[0m" << std::endl;
            }
        }
        else if (current_mode == 2)
        {
            virtual_barrier_generation(start_point, end_point, virtual_barrier);
        }
    }

    pcl::visualization::PointCloudColorHandlerCustom<PointType> white(astar_path, 255, 255, 255);
    pcl::visualization::PointCloudColorHandlerCustom<PointType> green(global_path, 0, 255, 0);
    pcl::visualization::PointCloudColorHandlerCustom<PointType> blue(virtual_barrier, 0, 0, 255);

    // Draw clicked points in red:
    pcl::visualization::PointCloudColorHandlerCustom<PointType> red(data->clicked_points_2d, 255, 0, 0);
    data->viewerPtr->removePointCloud("clicked_points");
    data->viewerPtr->addPointCloud(data->clicked_points_2d, red, "clicked_points");
    data->viewerPtr->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 10, "clicked_points");

    const std::lock_guard<std::mutex> lock(cloud_mutex);
    {
        data->viewerPtr->updatePointCloud<PointType>(astar_path, white, "astar_path");
        data->viewerPtr->updatePointCloud<PointType>(global_path, green, "global_path");
        data->viewerPtr->updatePointCloud<PointType>(virtual_barrier, blue, "virtual_barrier");
    }
}

void remove_points_callback(const pcl::visualization::MouseEvent &event, void *args)
{
    struct callback_args *data = (struct callback_args *)args;
    if (event.getButton() == pcl::visualization::MouseEvent::RightButton &&
        event.getType() == pcl::visualization::MouseEvent::MouseButtonRelease)
    {
        astar_path->points.clear();
        pcl::visualization::PointCloudColorHandlerCustom<PointType> white(astar_path, 255, 255, 255);

        global_path->points.clear();
        pcl::visualization::PointCloudColorHandlerCustom<PointType> green(global_path, 0, 255, 0);

        virtual_barrier->points.clear();
        pcl::visualization::PointCloudColorHandlerCustom<PointType> blue(virtual_barrier, 0, 0, 255);

        data->clicked_points_2d->points.clear();
        data->viewerPtr->removePointCloud("clicked_points");
        std::cout << " 清空所有历史拾取点云 " << std::endl;
        const std::lock_guard<std::mutex> lock(cloud_mutex);
        {
            data->viewerPtr->updatePointCloud<PointType>(astar_path, white, "astar_path");
            data->viewerPtr->updatePointCloud<PointType>(global_path, green, "global_path");
            data->viewerPtr->updatePointCloud<PointType>(virtual_barrier, blue, "virtual_barrier");
        }
    }
}

void map_save_callback(const pcl::visualization::KeyboardEvent &event, void *args)
{
    struct callback_args *data = (struct callback_args *)args;

    if (event.getKeySym() == "s" && event.keyDown())
    {
        update_global_map(global_map, virtual_barrier, virtual_barrier_sum);
        virtual_barrier->points.clear();
        boundingbox_sum->points.clear();
        grid_map->points.clear();
        pcl::io::loadPCDFile("./global_map.pcd", *global_map);
        gridmap_generation(global_map, boundingbox_sum, grid_map);

        pcl::visualization::PointCloudColorHandlerCustom<PointType> green(global_map, 127, 255, 0);
        pcl::visualization::PointCloudColorHandlerCustom<PointType> white(boundingbox_sum, 255, 255, 255);
        pcl::visualization::PointCloudColorHandlerCustom<PointType> blue(virtual_barrier, 0, 0, 255);

        const std::lock_guard<std::mutex> lock(cloud_mutex);
        {
            data->viewerPtr->updatePointCloud<PointType>(global_map, green, "global_map");
            data->viewerPtr->updatePointCloud<PointType>(boundingbox_sum, white, "boundingbox_sum");
            data->viewerPtr->updatePointCloud<PointType>(virtual_barrier, blue, "virtual_barrier");
        }
    }
}

int main(int argc, char *argv[])
{
    // generate grid map based on global map (one-time task)
    pcl::io::loadPCDFile(argv[1], *global_map);
    gridmap_generation(global_map, boundingbox_sum, grid_map);
    ground_generation(global_map, ground_map);

    if (argv[2])
    {
        grid_map->height = 1;
        grid_map->width = grid_map->points.size();
        pcl::io::savePCDFileASCII("grid.pcd", *grid_map);
    }

    viewer->setBackgroundColor(0, 0, 0);
    viewer->addCoordinateSystem(1.0);

    pcl::visualization::PointCloudColorHandlerCustom<PointType> handler_global_map(global_map, 127, 255, 0); // 全局地图
    viewer->addPointCloud(global_map, handler_global_map, "global_map");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "global_map");

    pcl::visualization::PointCloudColorHandlerCustom<PointType> handler_boundingbox_sum(boundingbox_sum, 255, 255, 255); // 聚类boundingbox集合
    viewer->addPointCloud(boundingbox_sum, handler_boundingbox_sum, "boundingbox_sum");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "boundingbox_sum");

    // pcl::visualization::PointCloudColorHandlerCustom<PointType> handler_grid_map(grid_map, 255, 255, 255); // 障碍物栅格地图
    // viewer->addPointCloud(grid_map, handler_grid_map, "grid_map");
    // viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "grid_map");

    pcl::visualization::PointCloudColorHandlerCustom<PointType> handler_ground_map(ground_map, 0, 0, 0); // 全局地面地图
    viewer->addPointCloud(ground_map, handler_ground_map, "ground_map");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "ground_map");

    pcl::visualization::PointCloudColorHandlerCustom<PointType> handler_virtual_barrier(virtual_barrier, 0, 0, 255); // 全局地面地图
    viewer->addPointCloud(virtual_barrier, handler_virtual_barrier, "virtual_barrier");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "virtual_barrier");

    pcl::visualization::PointCloudColorHandlerCustom<PointType> handler_astar_path(astar_path, 255, 255, 255); // 全局路径(astar)
    viewer->addPointCloud(astar_path, handler_astar_path, "astar_path");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "astar_path");

    pcl::visualization::PointCloudColorHandlerCustom<PointType> handler_global_path(global_path, 0, 255, 0); // 全局路径(spline)
    viewer->addPointCloud(global_path, handler_global_path, "global_path");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "global_path");

    struct callback_args cb_args;
    pcl::PointCloud<PointType>::Ptr clicked_points_2d(new pcl::PointCloud<PointType>());
    cb_args.clicked_points_2d = clicked_points_2d;
    cb_args.viewerPtr = pcl::visualization::PCLVisualizer::Ptr(viewer);
    viewer->registerKeyboardCallback(mode_select_callback, (void *)&cb_args);
    viewer->registerPointPickingCallback(picking_point_callback, (void *)&cb_args);
    viewer->registerMouseCallback(remove_points_callback, (void *)&cb_args);
    viewer->registerKeyboardCallback(map_save_callback, (void *)&cb_args);

    viewer->spin();

    while (!viewer->wasStopped())
    {
        viewer->spinOnce(100);
        boost::this_thread::sleep(boost::posix_time::microseconds(100000));
    }

    return 0;
}