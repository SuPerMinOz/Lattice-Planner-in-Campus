#ifndef __REFERENCE_LINE_H__
#define __REFERENCE_LINE_H__

#include <vector>

struct ReferencePoint
{
    double x;
    double y;
    double s;
    double theta;
};

class ReferenceLine
{
public:
    std::vector<ReferencePoint> trajectory_points;
    ReferencePoint QueryNearestPointByPosition(const double x, const double y);
    void cartesian_to_frenet(const ReferencePoint &point, const double x, const double y, double &ptr_s, double &ptr_d);

private:
    double PointDistanceSquare(const ReferencePoint &point, const double x, const double y);
};

#endif