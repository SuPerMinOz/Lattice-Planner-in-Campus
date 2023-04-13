#include "quintic_polynomial.h"

QuinticPolynomial::QuinticPolynomial(double x0, double dx0, double ddx0, double x1, double dx1, double ddx1, double T)
    : xs(x0), vxs(dx0), axs(ddx0), xe(x1), vxe(dx1), axe(ddx1)
{
    coef_[0] = x0;
    coef_[1] = dx0;
    coef_[2] = ddx0 / 2.0;

    const double t2 = T * T;
    const double t3 = T * t2;

    // the direct analytical method is at least 6 times faster than using matrix inversion.
    const double c0 = (x1 - 0.5 * t2 * ddx0 - dx0 * T - x0) / t3;
    const double c1 = (dx1 - ddx0 * T - dx0) / t2;
    const double c2 = (ddx1 - ddx0) / T;

    coef_[3] = 0.5 * (20.0 * c0 - 8.0 * c1 + c2);
    coef_[4] = (-15.0 * c0 + 7.0 * c1 - c2) / T;
    coef_[5] = (6.0 * c0 - 3.0 * c1 + 0.5 * c2) / t2;
}

double QuinticPolynomial::Evaluate(const uint32_t order, const double T) const
{
    switch (order)
    {
    case 0:
    {
        return ((((coef_[5] * T + coef_[4]) * T + coef_[3]) * T + coef_[2]) * T + coef_[1]) * T + coef_[0];
    }
    case 1:
    {
        return (((5.0 * coef_[5] * T + 4.0 * coef_[4]) * T + 3.0 * coef_[3]) * T + 2.0 * coef_[2]) * T + coef_[1];
    }
    case 2:
    {
        return (((20.0 * coef_[5] * T + 12.0 * coef_[4]) * T) + 6.0 * coef_[3]) * T + 2.0 * coef_[2];
    }
    case 3:
    {
        return (60.0 * coef_[5] * T + 24.0 * coef_[4]) * T + 6.0 * coef_[3];
    }
    case 4:
    {
        return 120.0 * coef_[5] * T + 24.0 * coef_[4];
    }
    case 5:
    {
        return 120.0 * coef_[5];
    }
    default:
        return 0.0;
    }
}