#pragma once

#include "Streamline.h"

namespace LIC {
class StreamlineStore {
   private:
    std::vector<Streamline> streamlines;

   public:
    explicit StreamlineStore() : streamlines{} {}

    /// Adds a new streamline by providing its points.
    /// @param points The points of the streamline
    void add(std::vector<fantom::Vector2> points);

    /// Querys the store by streamline id.
    /// @throws out_of_range exception if the id does not correspond to a streamline in the store.
    /// @param id The id of the streamline.
    /// @return A shared pointer to the streamline.
    std::shared_ptr<Streamline> get(size_t id);

    /// Querys the store by a point.
    /// @param point The point we want the streamline for.
    /// @param eps The precision we care for.
    /// @returns The streamline containing the point, or `nullptr` if there is no such streamline.
    std::shared_ptr<Streamline> get(fantom::Vector2& point, double eps = 0.000001);
};
}  // namespace LIC
