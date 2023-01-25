#pragma once

#include <fantom/math.hpp>
#include <memory>
#include <vector>

namespace LIC {

struct LinePoint {
    fantom::Vector2 point;
    size_t streamline_id;
    // TODO: accumulator for the convolution
    // TODO: accumulator of the hits
};

class Streamline {
   private:
    std::vector<LinePoint> points;  //!< Points on the streamline.
    size_t id;

   public:
    /// Constructs a streamline from a list of fantom::Vector2.
    /// @param points Sorted list of points on the streamline.
    /// @param id The streamline id for acessing the streamline store
    explicit Streamline(std::vector<fantom::Vector2> &points, size_t id) : id{id} {
        std::transform(points.begin(), points.end(), std::back_inserter(this->points), [&](fantom::Vector2 p) {
            return LinePoint{p, id};
        });
    }

    /// Queries if the `query` is on the Streamline given the precision `eps`.
    /// @param query The point we want to check the Streamline for.
    /// @param eps Precision we care about.
    /// @return Shared pointer to the corresponding LinePoint if found, nullptr otherwise.
    std::shared_ptr<LinePoint> contains(const fantom::Vector2 &query, double eps = 0.000001) const;

    /// Constructs an list of points from the streamline in the interval given by `size`.
    /// The queried point is in the center of the interval.
    /// @param query The point around which to construct the list.
    /// @param size The size of the interval along the streamline (not global distance).
    /// @param eps Precision we care about.
    /// @return The sorted list of points in the interval.
    std::vector<std::shared_ptr<LinePoint>> request_range(const fantom::Vector2 &query, double size,
                                                          double eps = 0.000001) const;
};
}  // namespace LIC
