#include <algorithm>
#include <cmath>
#include <cstddef>
#include <fantom/algorithm.hpp>
#include <fantom/datastructures/DomainFactory.hpp>
#include <fantom/datastructures/Function.hpp>
#include <fantom/datastructures/domains/Grid.hpp>
#include <fantom/datastructures/domains/LineSet.hpp>
#include <fantom/datastructures/interfaces/Field.hpp>
#include <fantom/graphics.hpp>
#include <fantom/math.hpp>
#include <fantom/register.hpp>
#include <fantom/registry/algorithm.hpp>
#include <numeric>

// needed for BoundinSphere-Calculation and normal calculation
#include <fantom-plugins/utils/Graphics/HelperFunctions.hpp>
#include <fantom-plugins/utils/Graphics/ObjectRenderer.hpp>

#include "CoordTransform.h"
#include "CubicInterpolator.h"
#include "SimplexNoise.h"
#include "TexMeta.h"

using namespace fantom;

namespace {

using Grid2 = fantom::Grid<2>;
using Evaluator = std::unique_ptr<FieldEvaluator<2UL, Tensor<double, 2>>>;

class GraphicsTutorialAlgorithm : public VisAlgorithm {
   public:
    struct Options : public VisAlgorithm::Options {
        explicit Options(fantom::Options::Control& control) : fantom::VisAlgorithm::Options{control} {
            add<Field<2, Vector2>>("Field", "A 2D vector field", definedOn<Grid<2>>(Grid<2>::Points));
            add<float>("DPI", "Dots per unit", 1.);
            add<size_t>("Arc Length", "Lenght of the convolution", 30);
            add<size_t>("MinNumHits", "Minimal Number of Hits", 30);
            add<size_t>("M", "M", 30);
            add<float>("Z-Offset", "Z-Offset to fix Z-fighting", 0.);
        }
    };
    struct VisOutputs : public VisAlgorithm::VisOutputs {
        VisOutputs(fantom::VisOutputs::Control& control) : VisAlgorithm::VisOutputs(control) { addGraphics("FLIC"); }
    };

    GraphicsTutorialAlgorithm(InitData& data) : VisAlgorithm(data) {}

    virtual void execute(const Algorithm::Options& options, const volatile bool& abortFlag) override {
        auto const& system = graphics::GraphicsSystem::instance();

        // Test read Field
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

        // Local Resource Path
        std::string resourcePathLocal = PluginRegistrationService::getInstance().getResourcePath("custom/PlugLIC");

        // get max coordinates
        PointSetBase::BoundingBox b_box = grid->getBoundingBox();

        double dpi = options.get<float>("DPI");
        size_t arc_length = options.get<size_t>("Arc Length");
        float z_off = options.get<float>("Z-Offset");

        size_t minNumHits = options.get<size_t>("MinNumHits");
        size_t M = options.get<size_t>("M");

        size_t t_width = (b_box[0].second - b_box[0].first) * dpi;
        size_t t_height = (b_box[1].second - b_box[1].first) * dpi;
        LIC::CoordTransform c_transform(dpi, b_box);

        auto evaluator = field->makeEvaluator();
        SimplexNoise noise{};

        //////////////////////////////////////////////////
        //// Generate noise
        std::vector<Color> noise_c(t_width * t_height);
        auto noise_idx = [&t_width](LIC::DPoint2 p) { return p.x + p.y * t_width; };

        Progress prog(*this, "Sampling Noise", t_width * t_height);
        for (size_t y = 0; y < t_height; ++y) {
            for (size_t x = 0; x < t_width; ++x) {
                size_t idx = y * t_width + x;
                float n = std::abs(noise.noise(x, y));
                noise_c[idx] = Color(n, n, n, 1.);
                prog++;
            }
        }
        prog.finalize();

        ///////////////////////////////////////////////////////
        debugLog() << "Start FLIC" << std::endl;
        LIC::TexMeta tex_meta(t_width, t_height);

        Progress prog2(*this, "Calculating Points", t_width * t_height);

        // FIX: partition into blocks instead of scanlining
        for (size_t y = 0; y < t_height; ++y) {
            for (size_t x = 0; x < t_width; ++x) {
                prog2++;
                if (abortFlag) return;
                auto px = tex_meta.pxl({x, y});
                if (px->hits < minNumHits) {
                    // streamline computaiton
                    auto ir = integrate(c_transform.px2world(px->pix_coord), evaluator, dpi, c_transform);

                    // convolution
                    double acc = 0;

                    size_t n_min = 0;
                    if (arc_length / 2 < ir.idx) n_min = ir.idx - arc_length / 2;
                    size_t n_max = std::min(ir.idx + arc_length / 2, ir.streamline.size());

                    for (size_t i = n_min; i < n_max; ++i) {
                        acc += noise_c[noise_idx(ir.streamline[i])].r();
                    }

                    // update pixel
                    px->intensity += acc;
                    px->hits += 1;

                    // start fast convolution calculation
                    // m positive and negative
                    double mp = acc;
                    double mn = acc;
                    for (size_t m = 1; m < M; ++m) {
                        if (n_max + m > ir.streamline.size() || n_min < m) break;

                        // collect noise values for the kernel borders
                        auto p_p = ir.streamline[n_max + m];       // x_m + 1 + L
                        auto p_n = ir.streamline[n_min - m];       // x_m - 1 - L
                        auto p_pn = ir.streamline[n_max + m - 1];  // x_m + L
                        auto p_nn = ir.streamline[n_min - m + 1];  // x_m - L
                        double i_p = noise_c[noise_idx(p_p)].r();
                        double i_n = noise_c[noise_idx(p_n)].r();
                        double i_pn = noise_c[noise_idx(p_pn)].r();
                        double i_nn = noise_c[noise_idx(p_nn)].r();

                        // normalization factor
                        double k = 1. / (n_max - n_min + 1);

                        mp = mp + k * (i_p - i_nn);
                        mn = mn + k * (i_n - i_pn);

                        // update pixels
                        auto p_pxl = tex_meta.pxl(p_p);
                        p_pxl->hits += 1;
                        p_pxl->intensity += mp;

                        auto n_pxl = tex_meta.pxl(p_n);
                        n_pxl->hits += 1;
                        n_pxl->intensity += mn;
                    }
                }
            }
        }
        prog2.finalize();

        ///////////////////////////////////////////////////////
        debugLog() << "Collect Intensities" << std::endl;

        std::vector<Color> colors(t_width * t_height);
        auto intensities = tex_meta.normalized_intensities();
        for (size_t i = 0; i < intensities.size(); ++i) {
            auto val = intensities[i];
            colors[i] = Color(val, val, val, 1.);
        }

        ///////////////////////////////////////////////////////
        // Generate 2D Texture
        Size2D texture_size{t_width, t_height};
        std::shared_ptr<graphics::Texture2D> tex_vec = system.makeTexture(texture_size, graphics::ColorChannel::RGBA);
        tex_vec->range(Pos2D(0, 0), texture_size, colors);

        // Set Quad vertecies.
        std::vector<PointF<3>> verticesTex(4);
        verticesTex[0] = PointF<3>(b_box[0].first, b_box[1].first, z_off);
        verticesTex[1] = PointF<3>(b_box[0].second, b_box[1].first, z_off);
        verticesTex[2] = PointF<3>(b_box[0].first, b_box[1].second, z_off);
        verticesTex[3] = PointF<3>(b_box[0].second, b_box[1].second, z_off);

        // These are the 3D-TextureCoordinates describing the borders of the
        // Texture.
        std::vector<PointF<2>> texCoords(4);
        texCoords[0] = PointF<2>(0.0, 0.0);
        texCoords[1] = PointF<2>(1.0, 0.0);
        texCoords[2] = PointF<2>(0.0, 1.0);
        texCoords[3] = PointF<2>(1.0, 1.0);

        // Because we cannot draw QUADS in FAnToM, we have to draw two triangles.
        std::vector<unsigned int> indicesTex(6);
        indicesTex[0] = 0;
        indicesTex[1] = 1;
        indicesTex[2] = 2;
        indicesTex[3] = 2;
        indicesTex[4] = 1;
        indicesTex[5] = 3;

        // Setup Rendering stuff
        auto bs = graphics::computeBoundingSphere(verticesTex);
        std::shared_ptr<graphics::Drawable> lic_tex =
            system.makePrimitive(graphics::PrimitiveConfig{graphics::RenderPrimitives::TRIANGLES}
                                     .vertexBuffer("position", system.makeBuffer(verticesTex))
                                     .vertexBuffer("texCoords", system.makeBuffer(texCoords))
                                     .indexBuffer(system.makeIndexBuffer(indicesTex))
                                     .texture("inTexture", tex_vec)
                                     .boundingSphere(bs),
                                 system.makeProgramFromFiles(resourcePathLocal + "shader/texture-vertex.glsl",
                                                             resourcePathLocal + "shader/texture-fragment.glsl"));
        setGraphics("FLIC", lic_tex);
    }

   private:
    /// Holds the result of the `integrate` method. We need the index of the start point in addition to the streamline.
    struct IntegrationResult {
        std::vector<LIC::DPoint2> streamline;  //!< The integrated points in pixel coordinates.
        size_t idx;                            //!< The index of the point that initiated the integration.
    };

    /// Performs the forward and backward integration of the streamline.
    /// Without timing it, this is probably extremely slow.
    /// @param p Center point of the streamline.
    /// @param eval The field evaluator.
    /// @param dpi Dots per unit, integration stepsize will be 3/dpi, final interpolation has 1/dpi distance between
    /// points.
    /// @param transform The transformation between world and pixel coordinates and the inverse.
    /// @return Integration result, which contains the points on the streamline and `p` 's index on it.
    IntegrationResult integrate(const Point2& p, const Evaluator& eval, double dpi, LIC::CoordTransform& transform) {
        // FIX: This whole thing is really slow ... :( Probably due to a lot of allocations.

        auto forward = runge_kutta(p, 3 / dpi, eval);
        auto backward = runge_kutta(p, -3 / dpi, eval);

        if (backward.size() > 0) {
            // reverse backward and drop `p`
            std::reverse(backward.begin(), backward.end());
            backward.pop_back();
        }

        size_t idx = backward.size();

        // concat
        backward.insert(backward.end(), forward.begin(), forward.end());

        // interpolate
        LIC::CubicInterpolator ci;

        std::vector<LIC::DiffPoint> sp;
        std::transform(backward.begin(), backward.end(), std::back_inserter(sp), [&eval](Point2 p) {
            eval->reset(p);
            return LIC::DiffPoint(p, eval->value());
        });
        auto interpolated = ci.interpolate(sp, 1 / dpi);

        // transform to px-space
        std::vector<LIC::DPoint2> ret(interpolated.size());
        for (size_t i = 0; i < interpolated.size(); ++i) {
            ret[i] = transform.world2px(interpolated[i]);
        }
        // retain only unique pixels
        auto last = std::unique(ret.begin(), ret.end(),
                                [](LIC::DPoint2 a, LIC::DPoint2 b) { return a.x == b.x && a.y == b.y; });
        ret.erase(last, ret.end());

        return {ret, idx};
    }

    /// Performs integration via the Runge-Kutta method.
    /// @param start The start point of the integration.
    /// @param h The step size of the integration.
    /// @param eval The field evaluator which provides the integration with values at points.
    /// @param max_iter Fail-save, we cannot produce more than this amount of points … or step on one spot more than
    /// this.
    /// @return A list of points on the integrated path.
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

AlgorithmRegister<GraphicsTutorialAlgorithm> dummy("A/2D-FLIC",
                                                   "Renders Fast Line Integral Convolution for 2D Vector Field");
}  // namespace
