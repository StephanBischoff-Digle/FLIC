#include "CubicInterpolator.h"

#include <Eigen/Dense>
#include <cmath>
#include <fantom/math.hpp>
#include <iterator>

namespace LIC {

using Eigen::Vector2d;

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

Vector2d CubicInterpolator::interpolate_interval(const CubicInterpolator::InterpolatorInterval &iv, double t) const {
    auto base = this->calc_base(t);

    Vector2d p0(iv.p0.p[0], iv.p0.p[1]);
    Vector2d p1(iv.p1.p[0], iv.p1.p[1]);
    Vector2d m0(iv.p0.m[0], iv.p0.m[1]);
    Vector2d m1(iv.p1.m[0], iv.p1.m[1]);

    auto pk = base.h00 * p0;
    auto mk = base.h10 * m0;
    auto pk1 = base.h01 * p1;
    auto mk1 = base.h11 * m1;

    auto p = pk + mk + pk1 + mk1;

    return p;
}

std::vector<double> CubicInterpolator::map_ts(const std::vector<double> &dss,
                                              const CubicInterpolator::AccDistF &adf) const {
    // NOTE: we assume `dss` to be sorted
    std::vector<double> ts;
    size_t i = 0;
    for (auto current_d : dss) {
        // find first distance that exceeds the searched distance value
        while (adf.d[i] < current_d) {
            i++;
            if (i == adf.d.size()) return ts;
        }

        double t0 = adf.t[i - 1];
        double t1 = adf.t[i];
        double d_t = (current_d - adf.d[i - 1]) / (adf.d[i] - adf.d[i - 1]);  // inverse lerp
        double t = t0 + d_t * (t1 - t0);                                      // lerp
        ts.push_back(t);
    }
    return ts;
}

std::vector<fantom::Vector2> CubicInterpolator::interpolate(const std::vector<DiffPoint> &line,
                                                            const double step_size) const {
    std::vector<Vector2d> r;
    r.emplace_back(line[0].p[0], line[0].p[1]);

    // NOTE: Presampling of the curve, such that we can map equally spaced distances into the t-space
    double dt = 0.025;       //!< This is arbitrary
    std::vector<double> ts;  //!< We collect the t-values corresponding to each sampled point
    for (size_t i = 1; i < line.size(); ++i) {
        CubicInterpolator::InterpolatorInterval iv{line[i - 1], line[i]};
        for (double t = dt; t < 1. + dt; t += dt) {
            r.push_back(this->interpolate_interval(iv, t));
            ts.push_back(i - 1 + t);
        }
    }

    // calculate accumulative distance function (ADF)
    std::vector<double> ds;
    double last_d = 0.;
    for (size_t i = 1; i < r.size(); ++i) {
        double dist = (r[i - 1] - r[i]).norm();
        last_d += dist;
        ds.push_back(last_d);
    }

    CubicInterpolator::AccDistF dst{ts, ds};
    std::vector<double> dss;  //!< Will contain the wanted distances that need map-back into t-space
    double v = 0.;
    while (v < last_d) {
        dss.push_back(v);
        v += step_size;
    }
    // TODO: Evaluate if this is necessary
    dss.push_back(last_d);

    // normalize ts
    auto norm_ts = this->map_ts(dss, dst);

    // Use normalized t-values to calculate the interpolation points again
    std::vector<Vector2d> normalized_r;
    for (double t : norm_ts) {
        size_t idx = std::floor(t);
        if (idx < line.size()) {
            CubicInterpolator::InterpolatorInterval iv{line[idx], line[idx + 1]};
            normalized_r.push_back(CubicInterpolator::interpolate_interval(iv, t - idx));
        } else {
            break;
        }
    }

    std::vector<fantom::Vector2> ret;
    // Remap from Eigen::Vector2d to fantom::Vector2
    std::transform(normalized_r.begin(), normalized_r.end(), std::back_inserter(ret), [&](Vector2d v) {
        return fantom::Vector2{v[0], v[1]};
    });
    // We missed the last point during interpolation, so we add it here
    ret.push_back(line.back().p);
    return ret;
}
}  // namespace LIC
