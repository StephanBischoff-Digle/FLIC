#include "CubicInterpolator.h"

#include <Eigen/Dense>
#include <fantom/dataset.hpp>

namespace LIC {

using Eigen::Vector2d;
using Eigen::Scaling;

CubicInterpolator::BaseFunctions CubicInterpolator::calc_base(double t) const {
    BaseFunctions base;
    double tt = t * t;
    double ttt = tt * t;
    base.h00 = 2 * ttt - 3 * tt + 1;
    base.h10 = ttt - 2 * tt + t;
    base.h01 = -2 * ttt + 3 * tt;
    base.h11 = ttt - tt;
    return base;
}

Vector2 CubicInterpolator::interpolate_interval(const CubicInterpolator::InterpolatorInterval &iv, double t) const {
    auto base = this->calc_base(t);
    Vector2 _delta = iv.p1.p - iv.p0.p;
    auto delta = Scaling(_delta[0], _delta[1]);

    Vector2d p0(iv.p0.p[0], iv.p0.p[1]);
    Vector2d p1(iv.p1.p[0], iv.p1.p[1]);
    Vector2d m0(iv.p0.m[0], iv.p0.m[1]);
    Vector2d m1(iv.p1.m[0], iv.p1.m[1]);

    auto pk = base.h00 * p0;
    auto mk = base.h10 * delta * m0;
    auto pk1 = base.h01 * p1;
    auto mk1 = base.h11 * delta * m1;

    auto p = pk + mk + pk1 + mk1;

    return Vector2(p.x(), p.y());
}

std::vector<Vector2> CubicInterpolator::interpolate(const std::vector<DiffPoint> &line, const fantom::Vector2 dpi) const {
    // TODO: line interpolation
    std::vector<fantom::Vector2> r;
    return r;
}
}  // namespace LIC
