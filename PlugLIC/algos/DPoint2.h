#pragma once

#include <cstdint>

namespace LIC {

struct DPoint2 {
    std::size_t x;
    std::size_t y;
};

struct PixelMeta {
    DPoint2 pix_coord;
    std::size_t hits;
    double intensity;
};

}
