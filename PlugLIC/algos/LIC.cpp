#include <algorithm>
#include <cstdint>
#include <fantom/algorithm.hpp>
#include <fantom/datastructures/DomainFactory.hpp>
#include <fantom/datastructures/Function.hpp>
#include <fantom/datastructures/domains/Grid.hpp>
#include <fantom/datastructures/domains/LineSet.hpp>
#include <fantom/datastructures/interfaces/Field.hpp>
#include <fantom/registry/algorithm.hpp>

#include "CubicInterpolator.h"
#include "SimplexNoise.h"
#include "StreamlineStore.h"

namespace {
using Grid2 = fantom::Grid<2>;
using fantom::Function;
using fantom::Vector2;

using Evaluator = std::unique_ptr<FieldEvaluator<2UL, Tensor<double, 2>>>;

class Integrator : public fantom::DataAlgorithm {
   public:
    struct Options : public DataAlgorithm::Options {
        explicit Options(fantom::Options::Control& control) : fantom::DataAlgorithm::Options{control} {
            add<Field<2, Vector2>>("Field", "A 2D vector field", definedOn<Grid2>(Grid2::Points));
        }
    };

    struct DataOutputs : public DataAlgorithm::DataOutputs {
        explicit DataOutputs(DataAlgorithm::DataOutputs::Control& control) : DataAlgorithm::DataOutputs{control} {}
    };

    explicit Integrator(InitData& data) : DataAlgorithm{data} {}

    void execute(const Algorithm::Options& options, const volatile bool&) override {
        std::shared_ptr<const Field<2, Vector2>> field = options.get<Field<2, Vector2>>("Field");
        std::shared_ptr<const Function<Vector2>> function = options.get<Function<Vector2>>("Field");

        // if there is no input, do nothing
        if (!field) {
            debugLog() << "Input Field not set." << std::endl;
            return;
        }

        // sanity check that interpolated fields really use the correct grid type.
        // This should never fail
        std::shared_ptr<const Grid2> grid = std::dynamic_pointer_cast<const Grid<2>>(function->domain());
        if (!grid) {
            throw std::logic_error("Wrong type of grid!");
        }

        // TODO: Implement stuff

        // std::vector<LIC::DiffPoint> samples;
        // samples.emplace_back(Vector2 {0., 0.}, Vector2 {1., 1.});
        // samples.emplace_back(Vector2 {1., 0.5}, Vector2 {1., 1.});
        // samples.emplace_back(Vector2 {1., 2.}, Vector2 {0., 1.});
        // samples.emplace_back(Vector2 {0., 1.}, Vector2 {1., 0.});
        //
        // LIC::CubicInterpolator ci;
        // auto ps = ci.interpolate(samples, .1);

        //////////////////////////////////////////////////
        //// Mock for Streamline Store
        //////////////////////////////////////////////////

        LIC::StreamlineStore sls;

        // construct streamline ps0 list
        std::vector<fantom::Vector2> ps0;
        ps0.emplace_back(0., 0.);
        ps0.emplace_back(1., 0.);
        ps0.emplace_back(1.5, 0.);
        ps0.emplace_back(2., 0.);
        ps0.emplace_back(3., 0.);
        ps0.emplace_back(4., 0.);
        ps0.emplace_back(5., 0.);

        // add to streamline store
        sls.add(ps0);

        // construct streamline ps1 list
        std::vector<fantom::Vector2> ps1;
        ps1.emplace_back(0., 1.);
        ps1.emplace_back(1., 1.);
        ps1.emplace_back(1.5, 1.);
        ps1.emplace_back(2., 1.);
        ps1.emplace_back(3., 1.);
        ps1.emplace_back(4., 1.);
        ps1.emplace_back(5., 1.);

        // add to streamline store
        sls.add(ps1);
        
        //////////////////////////////////////////////////
        //// debugging

        // recover streamline 0
        auto sl0 = sls.get(ps0[0]);
        // we need to check if we got a streamline
        if (sl0) {
            // print streamline id
            debugLog() << "id: " << sl0->id() << std::endl;

            // request all points that fall into the interval of size 2. centered around ps0[3]
            auto sl0_range = sl0->request_range(ps0[3], 2.);
            debugLog() << "\n";
            for (auto &p : sl0_range) {
                debugLog() << p->point << "\n";
            }
            debugLog() << std::endl;
        }
        // expected:
        // | id: 0
        // | [1, 0]
        // | [1.5, 0]
        // | [2, 0]         <- center ps0[3]
        // | [3, 0]

        // recover streamline 1
        auto sl1 = sls.get(ps1[3]);
        if (sl1) {
            // expected to be 1
            debugLog() << "id: " << sl1->id() << std::endl;

            // check and receive if a point is in the streamline
            auto p = sl1->contains(ps1[2]);
            if (p) {
                // expected to be 1 as well
                debugLog() << "point on streamline: " << p->streamline_id << std::endl;
            }
        }

        //////////////////////////////////////////////////
    }

   private:
    std::vector<Point2> runge_kutta(const Point2& start, double h, const Evaluator& eval, size_t max_iter = 1000) {
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
