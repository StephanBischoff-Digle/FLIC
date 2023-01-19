#pragma once

#include <fantom/math.hpp>
#include <vector>

namespace LIC {

/// Structure to handle a point and its derivatives together
struct DiffPoint {
    fantom::Vector2 p;  //!< The point
    fantom::Vector2 m;  //!< The derivative

    /// Creates a new DiffPoint that structures a point and its derivative.
    /// @param p The point
    /// @param m The derivative
    explicit DiffPoint(fantom::Vector2 p, fantom::Vector2 m) : p{p}, m{m} {}
};

class CubicInterpolator {
   private:
    struct BaseFunctions {
        double h00;
        double h10;
        double h01;
        double h11;
    };

    struct InterpolatorInterval {
        DiffPoint p0;
        DiffPoint p1;
    };

    /// Creates the evaluated hermite base function coefficients.
    BaseFunctions calc_base(double t) const;

    /// Evaluates the interpolation of the points given in @param iv at location @param t.
    /// @param iv The two points with their derivatives we want to interpolate between.
    /// @param t The location in the interval [0, 1] the interpolation is evaluated at.
    fantom::Vector2 interpolate_interval(const InterpolatorInterval &iv, double t) const;

   public:
    /// TODO: Documentation
    std::vector<fantom::Vector2> interpolate(const std::vector<DiffPoint> &samples, const fantom::Vector2 dpi) const;
};
}  // namespace LIC
