#ifndef __ENVIRONMENT__
#define __ENVIRONMENT__

#include "basic_configuration.h"

class Environment
{
public:
    VEC_1D reference_line_x, reference_line_y;
    VEC_1D upper_boundary_x, upper_boundary_y;
    VEC_1D lower_boundary_x, lower_boundary_y;

    void generate_global_map();

private:
    double road_width = 8.0;
};

#endif