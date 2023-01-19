#pragma once

#include <fantom/dataset.hpp>

namespace LIC {

using fantom::Vector2;

struct DiffPoint {
  Vector2 *p;
  Vector2 *m;
};

struct InterpolatorInterval {
  DiffPoint p0;
  DiffPoint p1;
};

class CubicInterpolator {
 public:
  Vector2 interpolate(const InterpolatorInterval &iv, double t) const;
};
}  // namespace LIC
