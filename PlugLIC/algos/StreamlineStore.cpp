#include "StreamlineStore.h"

#include <algorithm>
#include <stdexcept>

namespace LIC {
void StreamlineStore::add(std::vector<fantom::Vector2>& points) {
    auto id = this->streamlines.size();
    this->streamlines.emplace_back(points, id);
}

std::shared_ptr<Streamline> StreamlineStore::get(size_t id) {
    if (id >= this->streamlines.size()) throw std::out_of_range("ID not in store!");
    return std::make_shared<Streamline>(this->streamlines[id]);
}
std::shared_ptr<Streamline> StreamlineStore::get(fantom::Vector2& point, double eps) {
    auto f = std::find_if(this->streamlines.begin(), this->streamlines.end(),
                          [&](Streamline s) { return s.contains(point, eps); });
    if (f == this->streamlines.end()) return nullptr;
    return std::make_shared<Streamline>(*f);
}
}  // namespace LIC
