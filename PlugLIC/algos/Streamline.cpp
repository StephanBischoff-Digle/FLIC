#include "Streamline.h"

#include <algorithm>
#include <fantom/math.hpp>

namespace LIC {
std::shared_ptr<LinePoint> Streamline::contains(const fantom::Vector2& query, double eps) const {
    double eps2 = eps * eps;
    auto f = std::find_if(this->points.begin(), this->points.end(),
                          [&](LinePoint p) { return fantom::norm2(query - p.point) < eps2; });
    if (f == this->points.end()) return nullptr;
    return std::make_shared<LinePoint>(*f);
}

std::vector<std::shared_ptr<LinePoint>> Streamline::request_range(const fantom::Vector2& query, double size,
                                                                  double eps) const {
    double eps2 = eps * eps;
    double size2h = (size * size) * .5;

    // Find our point
    auto pos = std::find_if(this->points.begin(), this->points.end(),
                            [&](LinePoint p) { return fantom::norm2(query - p.point) < eps2; });

    std::vector<std::shared_ptr<LinePoint>> ps;

    // backward until size is exceeded
    auto last = pos;
    double dist = fantom::norm2(pos->point - (pos - 1)->point);
    for (auto b = pos - 1; b != this->points.begin() && dist < size2h; b--) {
        ps.push_back(std::make_shared<LinePoint>(*b));
        dist += fantom::norm2(b->point - last->point);
        last = b;
    }

    // reverse such that the list starts with the point farthest away in the negative direction
    std::reverse(ps.begin(), ps.end());
    // add the queried point
    ps.push_back(std::make_shared<LinePoint>(*pos));

    // forward until size is exceeded
    last = pos;
    dist = fantom::norm2(pos->point - (pos + 1)->point);
    for (auto b = pos + 1; b != this->points.begin() && dist < size2h; b++) {
        ps.push_back(std::make_shared<LinePoint>(*b));
        dist += fantom::norm2(b->point - last->point);
        last = b;
    }

    return ps;
}
}  // namespace LIC
