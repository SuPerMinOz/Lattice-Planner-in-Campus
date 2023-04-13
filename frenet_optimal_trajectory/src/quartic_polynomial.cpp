#include "quartic_polynomial.h"

QuarticPolynomial::QuarticPolynomial(double x0, double dx0, double ddx0, double dx1, double ddx1, double T)
    : xs(x0), vxs(dx0), axs(ddx0), vxe(dx1), axe(ddx1)
{
    coef_[0] = x0;
    coef_[1] = dx0;
    coef_[2] = 0.5 * ddx0;

    double b0 = dx1 - ddx0 * T - dx0;
    double b1 = ddx1 - ddx0;

    double t2 = T * T;
    double t3 = t2 * T;

    coef_[3] = (3 * b0 - b1 * T) / (3 * t2);
    coef_[4] = (-2 * b0 + b1 * T) / (4 * t3);
};

double QuarticPolynomial::Evaluate(const size_t order, const double T) const
{
    switch (order)
    {
    case 0:
    {
        return (((coef_[4] * T + coef_[3]) * T + coef_[2]) * T + coef_[1]) * T + coef_[0];
    }
    case 1:
    {
        return ((4.0 * coef_[4] * T + 3.0 * coef_[3]) * T + 2.0 * coef_[2]) * T + coef_[1];
    }
    case 2:
    {
        return (12.0 * coef_[4] * T + 6.0 * coef_[3]) * T + 2.0 * coef_[2];
    }
    case 3:
    {
        return 24.0 * coef_[4] * T + 6.0 * coef_[3];
    }
    case 4:
    {
        return 24.0 * coef_[4];
    }
    default:
        return 0.0;
    }
}