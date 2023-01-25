#pragma once

#include <Eigen/Dense>
#include <fantom/math.hpp>
#include <vector>

namespace LIC {
using Eigen::Vector2d;

/// Structure to handle a point and its derivatives together (or a point and its corresponding vector)
struct DiffPoint {
    fantom::Vector2 p;  //!< The point
    fantom::Vector2 m;  //!< The derivative

    /// Creates a new DiffPoint that structures a point and its derivative.
    /// @param p The point
    /// @param m The derivative
    explicit DiffPoint(fantom::Vector2 p, fantom::Vector2 m) : p{p}, m{m} {}
};

/// Class to interpolate between a list of points with associated derivatives. The resulting interpolation is a list of
/// equidistant points along a hermite spline.
class CubicInterpolator {
   private:
    /// Structure to hold the evaluated hermite base functions
    struct BaseFunctions {
        double h00;
        double h10;
        double h01;
        double h11;
    };

    /// Structure to couple two adjacent DiffPoints during interpolation
    struct InterpolatorInterval {
        DiffPoint p0;
        DiffPoint p1;
    };

    /// The Accumulated Distance Function with respect to the corresponding t-values
    struct AccDistF {
        std::vector<double> t;  //!< The list of t-values corresponding to the accumulated distances
        std::vector<double> d;  //!< The list of accumulated distance values
    };

    /// Creates the evaluated hermite base function coefficients.
    /// @param t The value for which the coefficients are calculated.
    /// @returns The evaluated hermite base function coefficients.
    BaseFunctions calc_base(double t) const;

    // Maps the distances listed in `dss` from the Accumulated Distance into the corresponding t-values.
    // @attention Both `dss` and the lists in the ADF are required to be sorted.
    // @param dss The list of distances with respect to the start of the interpolation that need mapping into t-space.
    // @param adf The Accumulated Distance Function which is used to do the mapping.
    // @returns The list of t-values corresponding to the distance values in `dss`.
    std::vector<double> map_ts(const std::vector<double> &dss, const AccDistF &adf) const;

    /// Evaluates the interpolation of the points given in @param iv at location @param t.
    /// @param iv The two points with their derivatives we want to interpolate between.
    /// @param t The location in the interval [0, 1] the interpolation is evaluated at.
    /// @returns The interpolated point.
    Vector2d interpolate_interval(const InterpolatorInterval &iv, double t) const;

   public:
    /// Calculates the interpolation points between the DiffPoint vectors. Each calculated point is `step_size` appart
    /// from the next one.
    /// @param samples The list of DiffPoint (a point paired with its derivative) that needs to be interpolated.
    /// @param step_size The distance between two interpolated points.
    /// @returns List of interpolated points on the streamline
    std::vector<fantom::Vector2> interpolate(const std::vector<DiffPoint> &samples, const double step_size) const;
};
}  // namespace LIC
