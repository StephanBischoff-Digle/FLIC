#pragma once

#include "DPoint2.h"

#include <vector>
#include <algorithm>
#include <memory>

namespace LIC {

/// Class that provides access to the texture metadate during fast line integral convolution.
class TexMeta {
private:
    std::size_t width; //!< Width of the texture. Needed for performing index calculation.

    std::vector<std::shared_ptr<PixelMeta>> tex;    //!< The actual pixel metadata.

    /// Performs point to index conversion.
    /// @param p The point we want to convert into the tex index-space.
    /// @return The index in tex of the point `p`.
    std::size_t point2idx(DPoint2& p) {
        return p.x + p.y * width;
    }

public:
    /// Creates a texture metadata object.
    /// @param width Texture width.
    /// @param height Texture height.
    explicit TexMeta(std::size_t width, std::size_t height) : width{width}, tex(width * height) {
        for (std::size_t y = 0; y < height; ++y) {
            for (std::size_t x = 0; x < width; ++x) {
                DPoint2 p {x, y};
                tex[point2idx(p)] = std::make_shared<PixelMeta>(PixelMeta{p, 0, 0});
            }
        }
    }

    /// Provides read/write access to a pixels metadata.
    /// @param p The pixel coordinate of the pixel in question.
    /// @return Read/write access to the pixels metadata.
    std::shared_ptr<PixelMeta> pxl(DPoint2 p) {return tex[point2idx(p)];}

    /// Calculates the intensitiy normalization according to the pixels `hit` field.
    /// @return List of all normalized pixel intensities.
    std::vector<double> normalized_intensities() const {
        std::vector<double> intensities(tex.size());
        for (std::size_t i = 0; i < tex.size(); ++i) {
            intensities[i] = std::min(1., std::max(0., tex[i]->intensity / tex[i]->hits));
        }
        return intensities;
    }

};

}
