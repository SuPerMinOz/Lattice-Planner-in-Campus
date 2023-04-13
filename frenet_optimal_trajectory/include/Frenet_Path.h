#ifndef __FRENET_PATH__
#define __FRENET_PATH__

#include "basic_configuration.h"

class FrenetPath
{
public:
    VEC_1D t;
    VEC_1D d, d_d, d_dd, d_ddd;
    VEC_1D s, s_d, s_dd, s_ddd;
    VEC_1D x, y, yaw, ds, curvature;

    double cd, cv, cf;
    double max_speed, max_accel, max_curvature;
};

using VEC_FrenetPath = std::vector<FrenetPath>;

#endif