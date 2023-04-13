#ifndef _QUINTIC_POLYNOMIAL__
#define _QUINTIC_POLYNOMIAL__

#include <array>

class QuinticPolynomial
{
public:
    QuinticPolynomial(double x0, double dx0, double ddx0, double x1, double dx1, double ddx1, double T);
    double Evaluate(const std::uint32_t order, const double p) const;

private:
    double xs, vxs, axs, xe, vxe, axe;
    std::array<double, 6> coef_{{0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};
};

#endif