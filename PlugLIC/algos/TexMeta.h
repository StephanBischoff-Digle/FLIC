#pragma once

#include "DPoint2.h"

#include <vector>
#include <memory>

namespace LIC {

class TexMeta {
private:
    std::size_t width;

    std::vector<std::shared_ptr<PixelMeta>> tex;
    std::size_t point2idx(DPoint2& p) {
        return p.x + p.y * width;
    }

public:
    explicit TexMeta(std::size_t width, std::size_t height) : width{width}, tex(width * height) {
        for (std::size_t y = 0; y < height; ++y) {
            for (std::size_t x = 0; x < width; ++x) {
                DPoint2 p {x, y};
                tex[point2idx(p)] = std::make_shared<PixelMeta>(PixelMeta{p, 0, 0});
            }
        }
    }

    std::shared_ptr<PixelMeta> pxl(DPoint2 p) {return tex[point2idx(p)];}
    std::vector<double> normalized_intensities() const {
        std::vector<double> intensities(tex.size());
        for (std::size_t i = 0; i < tex.size(); ++i) {
            intensities[i] = tex[i]->intensity / tex[i]->hits;
        }
        return intensities;
    }

};

}
