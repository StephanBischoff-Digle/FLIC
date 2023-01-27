#pragma once

#include <cstdint>

namespace LIC {

/// Discrete point
struct DPoint2 {
    std::size_t x;
    std::size_t y;
};

/// Pixel metadata for FLIC
struct PixelMeta {
    DPoint2 pix_coord;  //!< The pixel's coordinate
    std::size_t hits;   //!< The counter of how often this pixel was updated
    double intensity;   //!< The accumulated intensity during FLIC
};

}  // namespace LIC
