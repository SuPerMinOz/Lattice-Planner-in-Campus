#include "Spline.h"

VEC_1D calc_diff_vec(VEC_1D &input)
{
    VEC_1D output;
    for (size_t i = 1; i < input.size(); i++)
    {
        output.emplace_back(input[i] - input[i - 1]);
    }
    return output;
};

VEC_1D calc_sum_vec(VEC_1D &input)
{
    VEC_1D output;
    double sum = 0;
    for (size_t i = 0; i < input.size(); i++)
    {
        sum += input[i];
        output.emplace_back(sum);
    }
    return output;
};

Spline1D::Spline1D(VEC_1D x_, VEC_1D y_) : x(x_), y(y_), a(y_)
{
    h = calc_diff_vec(x_);
    line_length = x_.size();
    Eigen::MatrixXd A = calc_A();
    Eigen::VectorXd B = calc_B();
    Eigen::VectorXd c_eigen = A.colPivHouseholderQr().solve(B);
    double *c_pointer = c_eigen.data();
    // Eigen::Map<Eigen::VectorXf>(c, c_eigen.rows(), 1) = c_eigen;
    c.assign(c_pointer, c_pointer + c_eigen.rows());

    for (size_t i = 0; i < line_length - 1; i++)
    {
        d.emplace_back((c[i + 1] - c[i]) / (3.0 * h[i]));
        b.emplace_back((a[i + 1] - a[i]) / h[i] - h[i] * (c[i + 1] + 2.0 * c[i]) / 3.0);
    }
};

// Calc y position for given x
double Spline1D::calc(double t)
{
    if (t < x.front() || t > x.back())
    {
        throw std::invalid_argument("received value out of the pre-defined range");
    }
    int seg_id = bisect(t, 0, line_length);
    double dx = t - x[seg_id];
    return a[seg_id] + b[seg_id] * dx + c[seg_id] * dx * dx + d[seg_id] * dx * dx * dx;
};

// Calc first derivative at given x
double Spline1D::calc_d(double t)
{
    if (t < x.front() || t > x.back())
    {
        throw std::invalid_argument("received value out of the pre-defined range");
    }
    int seg_id = bisect(t, 0, line_length);
    double dx = t - x[seg_id];
    return b[seg_id] + 2 * c[seg_id] * dx + 3 * d[seg_id] * dx * dx;
}

// Calc second derivative at given x
double Spline1D::calc_dd(double t)
{
    if (t < x.front() || t > x.back())
    {
        throw std::invalid_argument("received value out of the pre-defined range");
    }
    int seg_id = bisect(t, 0, line_length);
    double dx = t - x[seg_id];
    return 2 * c[seg_id] + 6 * d[seg_id] * dx;
}

// calc matrix A for spline coefficient c
Eigen::MatrixXd Spline1D::calc_A()
{
    Eigen::MatrixXd A = Eigen::MatrixXd::Zero(line_length, line_length);
    A(0, 0) = 1;
    for (size_t i = 0; i < line_length - 1; i++)
    {
        if (i != line_length - 2)
        {
            A(i + 1, i + 1) = 2.0 * (h[i] + h[i + 1]);
        }
        A(i + 1, i) = h[i];
        A(i, i + 1) = h[i];
    }
    A(0, 1) = 0.0;
    A(line_length - 1, line_length - 2) = 0.0;
    A(line_length - 1, line_length - 1) = 1.0;
    return A;
};

// calc matrix B for spline coefficient c
Eigen::VectorXd Spline1D::calc_B()
{
    Eigen::VectorXd B = Eigen::VectorXd::Zero(line_length);
    for (size_t i = 0; i < line_length - 2; i++)
    {
        B(i + 1) = 3.0 * (a[i + 2] - a[i + 1]) / h[i + 1] - 3.0 * (a[i + 1] - a[i]) / h[i];
    }
    return B;
};

int Spline1D::bisect(double t, int start, int end)
{
    int mid = start + (end - start) / 2;
    if (t == x[mid] || end - start <= 1)
    {
        return mid;
    }
    else if (t > x[mid])
    {
        return bisect(t, mid, end);
    }
    else
    {
        return bisect(t, start, mid);
    }
}

Spline2D::Spline2D(VEC_1D x, VEC_1D y)
{
    s = calc_s(x, y);
    sx = Spline1D(s, x);
    sy = Spline1D(s, y);
}

// calc position (distance from the start point)
POINT_2D Spline2D::calc_postion(double t)
{
    double x = sx.calc(t);
    double y = sy.calc(t);
    return {{x, y}};
};

double Spline2D::calc_curvature(double t)
{
    double dx = sx.calc_d(t);
    double ddx = sx.calc_dd(t);
    double dy = sy.calc_d(t);
    double ddy = sy.calc_dd(t);
    return (ddy * dx - ddx * dy) / std::pow((dx * dx + dy * dy), 3 / 2);
};

double Spline2D::calc_yaw(double t)
{
    double dx = sx.calc_d(t);
    double dy = sy.calc_d(t);
    return std::atan2(dy, dx);
};

VEC_1D Spline2D::calc_s(VEC_1D &x, VEC_1D &y)
{
    VEC_1D ds{0};
    VEC_1D dx = calc_diff_vec(x);
    VEC_1D dy = calc_diff_vec(y);

    for (size_t i = 0; i < dx.size(); i++)
    {
        ds.emplace_back(std::sqrt(dx[i] * dx[i] + dy[i] * dy[i]));
    }

    return calc_sum_vec(ds);
};
