#include <algorithm>
#include <cstdint>

#include <fantom/algorithm.hpp>
#include <fantom/datastructures/DomainFactory.hpp>
#include <fantom/datastructures/Function.hpp>
#include <fantom/datastructures/domains/Grid.hpp>
#include <fantom/datastructures/domains/LineSet.hpp>
#include <fantom/datastructures/interfaces/Field.hpp>
#include <fantom/registry/algorithm.hpp>

namespace {
using Grid3 = fantom::Grid<3>;
using fantom::Function;
using fantom::Vector3;

using Evaluator = std::unique_ptr<FieldEvaluator<3UL, Tensor<double, 3>>>;

class Integrator : public fantom::DataAlgorithm {
 public:
  struct Options : public DataAlgorithm::Options {
    explicit Options(fantom::Options::Control& control)
        : fantom::DataAlgorithm::Options{control} {
      add<Field<3, Vector3>>("Field", "A 3D vector field",
                             definedOn<Grid3>(Grid3::Points));
    }
  };

  struct DataOutputs : public DataAlgorithm::DataOutputs {
    explicit DataOutputs(DataAlgorithm::DataOutputs::Control& control)
        : DataAlgorithm::DataOutputs{control} {
    }
  };

  explicit Integrator(InitData& data) : DataAlgorithm{data} {}

  void execute(const Algorithm::Options& options,
               const volatile bool&) override {
    std::shared_ptr<const Field<3, Vector3>> field =
        options.get<Field<3, Vector3>>("Field");
    std::shared_ptr<const Function<Vector3>> function =
        options.get<Function<Vector3>>("Field");

    // if there is no input, do nothing
    if (!field) {
      debugLog() << "Input Field not set." << std::endl;
      return;
    }

    // sanity check that interpolated fields really use the correct grid type.
    // This should never fail
    std::shared_ptr<const Grid3> grid =
        std::dynamic_pointer_cast<const Grid<3>>(function->domain());
    if (!grid) {
      throw std::logic_error("Wrong type of grid!");
    }

    // TODO: Implement stuff

  }

 private:
  enum IntegrationMethod { Euler, RungeKutta };
  static std::unordered_map<std::string, Integrator::IntegrationMethod>
      integration_method_options;


  std::vector<Point3> runge_kutta(const Point3& start, double h,
                                  const Evaluator& eval,
                                  size_t max_iter = 1000) {
    std::vector<Point3> points{};
    Point3 current = start;
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

}
