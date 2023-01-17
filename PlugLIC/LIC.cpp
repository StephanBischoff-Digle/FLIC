#include <algorithm>
#include <cstdint>
#include <fantom/algorithm.hpp>
#include <fantom/datastructures/DomainFactory.hpp>
#include <fantom/datastructures/Function.hpp>
#include <fantom/datastructures/domains/Grid.hpp>
#include <fantom/datastructures/domains/LineSet.hpp>
#include <fantom/datastructures/interfaces/Field.hpp>
#include <fantom/registry/algorithm.hpp>

#include "SimplexNoise.h"

namespace {
using Grid2 = fantom::Grid<2>;
using fantom::Function;
using fantom::Vector2;

using Evaluator = std::unique_ptr<FieldEvaluator<2UL, Tensor<double, 2>>>;

class Integrator : public fantom::DataAlgorithm {
 public:
  struct Options : public DataAlgorithm::Options {
    explicit Options(fantom::Options::Control& control)
        : fantom::DataAlgorithm::Options{control} {
      add<Field<2, Vector2>>("Field", "A 2D vector field",
                             definedOn<Grid2>(Grid2::Points));
    }
  };

  struct DataOutputs : public DataAlgorithm::DataOutputs {
    explicit DataOutputs(DataAlgorithm::DataOutputs::Control& control)
        : DataAlgorithm::DataOutputs{control} {}
  };

  explicit Integrator(InitData& data) : DataAlgorithm{data} {}

  void execute(const Algorithm::Options& options,
               const volatile bool&) override {
    std::shared_ptr<const Field<2, Vector2>> field =
        options.get<Field<2, Vector2>>("Field");
    std::shared_ptr<const Function<Vector2>> function =
        options.get<Function<Vector2>>("Field");

    // if there is no input, do nothing
    if (!field) {
      debugLog() << "Input Field not set." << std::endl;
      return;
    }

    // sanity check that interpolated fields really use the correct grid type.
    // This should never fail
    std::shared_ptr<const Grid2> grid =
        std::dynamic_pointer_cast<const Grid<2>>(function->domain());
    if (!grid) {
      throw std::logic_error("Wrong type of grid!");
    }

    // TODO: Implement stuff
    SimplexNoise noise{};
    for (float x = 0.; x < 10.; x += 0.5) {
      debugLog() << noise.noise(x, 0.) << std::endl;
    }
  }

 private:
  std::vector<Point2> runge_kutta(const Point2& start, double h,
                                  const Evaluator& eval,
                                  size_t max_iter = 1000) {
    std::vector<Point2> points{};
    Point2 current = start;
    size_t iterations = 0;

    while (iterations < max_iter) {
      iterations++;
      eval->reset(current);

      if (!*eval) break;
      auto k1 = eval->value();

      eval->reset(current + h * k1 / 2., 0);
      if (!*eval) break;
      auto k2 = eval->value();

      eval->reset(current + h * k2 / 2., 0);
      if (!*eval) break;
      auto k3 = eval->value();

      eval->reset(current + h * k3, 0);
      if (!*eval) break;
      auto k4 = eval->value();

      auto update = 1 / 6. * (k1 + 2 * k2 + 2 * k3 + k4) * h;
      auto point = current + update;

      eval->reset(point);
      if (!*eval) break;
      points.push_back(point);
      current = point;
    }

    return points;
  }
};

AlgorithmRegister<Integrator> dummy("A/LIC", "Line Integral Convolution");

}  // namespace
