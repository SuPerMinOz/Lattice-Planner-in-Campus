#ifndef __Spline1D_1D__
#define __Spline1D_1D__

#include "basic_configuration.h"

VEC_1D calc_diff_vec(VEC_1D &input);
VEC_1D calc_sum_vec(VEC_1D &input);

class Spline1D
{
public:
    // x coordinates for data points. This x coordinates must be sorted in ascending order.
    // y coordinates for data points. y = a + b * x + c * x ** 2 + d * x ** 3.
    // h x1-x0, x2-x1, x3-x2, x4-x3, ..., xn-xn-1
    VEC_1D x, y, h;
    VEC_1D a, b, c, d;
    size_t line_length;

    Spline1D() = default;
    // y = a + b * x + c * x ** 2 + d * x ** 3
    Spline1D(VEC_1D x_, VEC_1D y_);

    double calc(double t);
    double calc_d(double t);
    double calc_dd(double t);

private:
    Eigen::MatrixXd calc_A();
    Eigen::VectorXd calc_B();
    int bisect(double t, int start, int end);
};

class Spline2D
{
public:
    VEC_1D s;
    Spline1D sx;
    Spline1D sy;

    Spline2D(VEC_1D x, VEC_1D y);

    POINT_2D calc_postion(double t);
    double calc_curvature(double t);
    double calc_yaw(double t);

private:
    VEC_1D calc_s(VEC_1D &x, VEC_1D &y);
};

#endif