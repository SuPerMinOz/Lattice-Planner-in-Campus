#ifndef _QUARTIC_POLYNOMIAL__
#define _QUARTIC_POLYNOMIAL__

#include <array>

class QuarticPolynomial
{
public:
    QuarticPolynomial(double x0, double dx0, double ddx0, double dx1, double ddx1, double T);
    double Evaluate(const size_t order, const double T) const;

private:
    double xs, vxs, axs, vxe, axe;
    std::array<double, 5> coef_ = {{0.0, 0.0, 0.0, 0.0, 0.0}};
};

#endif