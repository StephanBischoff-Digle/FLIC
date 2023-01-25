#pragma once

#include <fantom/math.hpp>
#include <memory>
#include <vector>

namespace LIC {
class Streamline {
   private:
    // TODO: We probably want more than just the raw points here. We need to keep track of the convolution!
    std::vector<fantom::Vector2> points;  //!< Points on the streamline.

   public:
    /// Constructs a streamline from a list of fantom::Vector2.
    /// @param points Sorted list of points on the streamline.
    explicit Streamline(std::vector<fantom::Vector2> &&points) : points{points} {}

    /// Queries if the `query` is on the Streamline given the precision `eps`.
    /// @param query The point we want to check the Streamline for.
    /// @param eps Precision we care about.
    /// @return `true` if the point is on the streamline, `false` otherwise.
    bool contains(const fantom::Vector2 &query, double eps = 0.000001) const;

    /// Constructs an list of points from the streamline in the interval given by `size`.
    /// The queried point is in the center of the interval.
    /// @param query The point around which to construct the list.
    /// @param size The size of the interval along the streamline (not global distance).
    /// @param eps Precision we care about.
    /// @return The sorted list of points in the interval.
    std::vector<std::shared_ptr<fantom::Vector2>> request_range(const fantom::Vector2 &query, double size,
                                                                double eps = 0.000001) const;
};
}  // namespace LIC
