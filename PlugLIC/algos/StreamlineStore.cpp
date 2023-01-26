#include "StreamlineStore.h"

#include <algorithm>
#include <stdexcept>

namespace LIC {
void StreamlineStore::add(std::vector<fantom::Vector2> points) {
    std::vector<DPoint2> dpoints;
    std::transform(points.begin(), points.end(), std::back_inserter(dpoints),
                   [&](fantom::Vector2 v) {
                       return this->ctf.world2px(v);
                   });

    std::vector<size_t> streamline;
    size_t s_id = this->streamlines.size() + 1;
    for (auto &p : points) {
        auto pix = this->ctf.world2px(p);
        auto pix_id = this->coord2idx(pix.x, pix.y);
        streamline.push_back(pix_id);
        if (this->points[pix_id].streamline_id == 0) {
            this->points[pix_id].streamline_id = s_id;
        }
    }

    this->streamlines.push_back(streamline);
}

std::shared_ptr<std::vector<LinePoint>> StreamlineStore::get(size_t id) {
    if (id == 0 || id - 1 >= this->streamlines.size()) return nullptr;
    return std::make_shared<std::vector<LinePoint>>(this->streamlines[id-1]);
}
std::shared_ptr<std::vector<LinePoint>> StreamlineStore::get(DPoint2 p) {
    auto idx = this->coord2idx(p.x, p.y);
    if (this->points[idx].streamline_id == 0) return nullptr;
    return this->get(idx);
}
}  // namespace LIC
