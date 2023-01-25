#include "Streamline.h"

#include <algorithm>
#include <fantom/math.hpp>

namespace LIC {
bool Streamline::contains(const fantom::Vector2& query, double eps) const {
    double eps2 = eps * eps;
    return std::any_of(this->points.begin(), this->points.end(),
                       [&](fantom::Vector2 p) { return fantom::norm2(query - p) < eps2; });
}

std::vector<std::shared_ptr<fantom::Vector2>> Streamline::request_range(const fantom::Vector2& query, double size,
                                                                        double eps) const {
    double eps2 = eps * eps;
    double size2h = (size * size) * .5;

    // Find our point
    auto pos = std::find_if(this->points.begin(), this->points.end(),
                            [&](fantom::Vector2 p) { return fantom::norm2(query - p) < eps2; });

    std::vector<std::shared_ptr<fantom::Vector2>> ps;

    // backward
    auto last = pos;
    double dist = fantom::norm2(*pos - *(pos - 1));
    for (auto b = pos - 1; b != this->points.begin() && dist < size2h; b--) {
        ps.push_back(std::make_shared<fantom::Vector2>(*b));
        dist += fantom::norm2(*b - *last);
        last = b;
    }

    // reverse such that the list starts with the point farthest away in the negative direction
    std::reverse(ps.begin(), ps.end());
    // add the queried result
    ps.push_back(std::make_shared<fantom::Vector2>(*pos));

    // forward
    last = pos;
    dist = fantom::norm2(*pos - *(pos + 1));
    for (auto b = pos + 1; b != this->points.begin() && dist < size2h; b++) {
        ps.push_back(std::make_shared<fantom::Vector2>(*b));
        dist += fantom::norm2(*b - *last);
        last = b;
    }

    return ps;
}
}  // namespace LIC
