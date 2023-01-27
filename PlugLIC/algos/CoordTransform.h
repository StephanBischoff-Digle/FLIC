#pragma once

#include <cmath>
#include <fantom/datastructures/domains/Grid.hpp>

#include "DPoint2.h"

namespace LIC {

/// Performs coordinate transformation between world- and pixel-space.
class CoordTransform {
   private:
    double dpi;                               //!< Number of dots per unit
    fantom::PointSetBase::BoundingBox b_box;  //!< Bounding box of the data.

   public:
    /// Constructs a CoordTransform.
    /// @param dpi Number of pixel per world unit.
    /// @param b_box The bounding box of the data.
    explicit CoordTransform(double dpi, fantom::PointSetBase::BoundingBox b_box) : dpi{dpi}, b_box{b_box} {}

    /// Maps world coordinates into pixel coordinates
    /// @param p Point in world space
    /// @return Point in pixel space
    DPoint2 world2px(fantom::Point2 p) {
        return {
            (size_t)std::floor((*p.begin() - b_box[0].first) * dpi),
            (size_t)std::floor((*(p.begin() + 1) - b_box[1].first) * dpi),
        };
    }

    /// Maps pixel coordinates into world coordinates
    /// @param p Point in pixel space
    /// @return Point in world space
    fantom::Point2 px2world(DPoint2 p) {
        return {
            b_box[0].first + p.x / dpi,
            b_box[1].first + p.y / dpi,
        };
    }
};
}  // namespace LIC
