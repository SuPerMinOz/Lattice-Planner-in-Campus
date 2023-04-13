#ifndef __CARMODEL__
#define __CARMODEL__

#include "basic_configuration.h"

class CarModel
{
public:
    VEC_1D car_x, car_y;

    void update(double x, double y, double yaw);

private:
    double RF = 4.5, RB = 1.0, W = 3.0;
};

#endif