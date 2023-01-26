#pragma once

#include "CoordTransform.h"
#include "DPoint2.h"

namespace LIC {

struct LinePoint {
    DPoint2 coords;
    size_t streamline_id;
    // TODO: accumulator for the convolution
    // TODO: accumulator of the hits
};

class StreamlineStore {
   private:
    std::vector<std::vector<size_t>> streamlines;
    std::vector<LinePoint> points;
    CoordTransform ctf;
    size_t width;
    size_t height;

    /// Transforms coords into an index
    size_t coord2idx(size_t x, size_t y) const {
        return y * width + x;
    }

   public:
    explicit StreamlineStore(size_t width, size_t height, CoordTransform ctf)
        : streamlines{}, points(width * height), ctf{ctf}, width{width}, height{height} {
        for (size_t y = 0; y < height; ++y) {
            for (size_t x = 0; x < width; ++x) {
                points[coord2idx(x, y)] = LinePoint {DPoint2{x, y}, 0};
            }
        }
    }

    /// Adds a new streamline by providing its points.
    /// @param points The points of the streamline
    void add(std::vector<fantom::Vector2> points);

    /// Querys the store by streamline id.
    /// @throws out_of_range exception if the id does not correspond to a streamline in the store.
    /// @param id The id of the streamline.
    /// @return A shared pointer to the streamline.
    std::shared_ptr<std::vector<LinePoint>> get(size_t id);

    /// Querys the store by a point.
    /// @param point The point we want the streamline for.
    /// @param eps The precision we care for.
    /// @returns The streamline containing the point, or `nullptr` if there is no such streamline.
    std::shared_ptr<std::vector<LinePoint>> get(DPoint2 point);

    std::shared_ptr<LinePoint> getLinePoint(DPoint2 point) const {
        auto idx = this->coord2idx(point.x, point.y);
        return std::make_shared<LinePoint>(this->points[idx]);
    }


    // TODO: get values from LinePoints
};
}  // namespace LIC
