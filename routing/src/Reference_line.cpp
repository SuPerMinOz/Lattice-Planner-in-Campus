#include <cmath>
#include "Reference_line.h"

double ReferenceLine::PointDistanceSquare(const ReferencePoint &point, const double x, const double y)
{
    const double dx = point.x - x;
    const double dy = point.y - y;
    return dx * dx + dy * dy;
}

ReferencePoint ReferenceLine::QueryNearestPointByPosition(const double x, const double y)
{
    double d_min = PointDistanceSquare(trajectory_points.front(), x, y);
    size_t index_min = 0;

    for (size_t i = 1; i < trajectory_points.size(); ++i)
    {
        double d_temp = PointDistanceSquare(trajectory_points[i], x, y);
        if (d_temp < d_min)
        {
            d_min = d_temp;
            index_min = i;
        }
    }
    return trajectory_points[index_min];
}

void ReferenceLine::cartesian_to_frenet(const ReferencePoint &point, const double x, const double y, double &ptr_s, double &ptr_d)
{
    double rx = point.x, ry = point.y, rtheta = point.theta, rs = point.s;    
    const double dx = x - rx;
    const double dy = y - ry;
    const double cos_theta_r = std::cos(rtheta);
    const double sin_theta_r = std::sin(rtheta);
    const double cross_rd_nd = cos_theta_r * dy - sin_theta_r * dx;
    ptr_d = std::copysign(std::sqrt(dx * dx + dy * dy), cross_rd_nd);
    ptr_s = rs;
}