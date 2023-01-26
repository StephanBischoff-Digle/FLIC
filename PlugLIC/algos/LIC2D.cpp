#include <cmath>
#include <cstddef>
#include <fantom/algorithm.hpp>
#include <fantom/datastructures/Function.hpp>
#include <fantom/datastructures/domains/Grid.hpp>
#include <fantom/datastructures/interfaces/Field.hpp>
#include <fantom/graphics.hpp>
#include <fantom/register.hpp>

// needed for BoundinSphere-Calculation and normal calculation
#include <fantom-plugins/utils/Graphics/HelperFunctions.hpp>
#include <fantom-plugins/utils/Graphics/ObjectRenderer.hpp>

#include "SimplexNoise.h"

using Grid2 = fantom::Grid<2>;
using fantom::Function;
using fantom::Vector2;

using namespace fantom;

namespace {

class GraphicsTutorialAlgorithm : public VisAlgorithm {
   public:
    struct Options : public VisAlgorithm::Options {
        explicit Options(fantom::Options::Control& control) : fantom::VisAlgorithm::Options{control} {
            add<Field<2, Vector2>>("Field", "A 2D vector field", definedOn<Grid<2>>(Grid<2>::Points));
            add<float>("DPI", "Dots per unit", 1.);
            add<float>("Arc Length", "Lenght of the convolution", 30.);
            add<float>("Z-Offset", "Z-Offset to fix Z-fighting", 0.);
        }
    };
    struct VisOutputs : public VisAlgorithm::VisOutputs {
        VisOutputs(fantom::VisOutputs::Control& control) : VisAlgorithm::VisOutputs(control) {
            addGraphics("LIC");
            addGraphics("Field");
            addGraphics("Noise");
        }
    };

    GraphicsTutorialAlgorithm(InitData& data) : VisAlgorithm(data) {}

    virtual void execute(const Algorithm::Options& options, const volatile bool& /*abortFlag*/) override {
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
        float v_scale = .5;  // TODO: get max from samples
        float arc_length = options.get<float>("Arc Length");
        float z_off = options.get<float>("Z-Offset");

        size_t t_width = (b_box[0].second - b_box[0].first) * dpi;
        size_t t_height = (b_box[1].second - b_box[1].first) * dpi;
        auto px2world = [&](size_t x, size_t y) {
            return Point2{
                b_box[0].first + x / dpi,
                b_box[1].first + y / dpi,
            };
        };

        auto evaluator = field->makeEvaluator();
        SimplexNoise noise{};

        // Generate textures
        // NOTE: we could use just one here, merging rg-for the field and b for the noise
        std::vector<Color> colors(t_width * t_height);
        std::vector<Color> noise_c(t_width * t_height);

        Progress prog(*this, "sampling texture and noise", t_width * t_height);

        // Set colors
        for (size_t y = 0; y < t_height; ++y) {
            for (size_t x = 0; x < t_width; ++x) {
                size_t idx = y * t_width + x;
                auto p = px2world(x, y);
                float n = std::abs(noise.noise(x, y));
                noise_c[idx] = Color(n, n, n, 1.);
                if (evaluator->reset(p, 0.)) {
                    auto val = evaluator->value();
                    double v_x = *val.begin();
                    double v_y = *(val.begin() + 1);

                    double r = 0.5 + v_x * v_scale;
                    double g = 0.5 + v_y * v_scale;
                    colors[idx] = Color(r, g, 0., 1.);
                } else {
                    colors[idx] = Color(.5, .5, 0., 1.);
                }
                prog++;
            }
        }

        // Generate 2D Texture
        Size2D texture_size{t_width, t_height};
        std::shared_ptr<graphics::Texture2D> tex_vec = system.makeTexture(texture_size, graphics::ColorChannel::RGBA);
        tex_vec->range(Pos2D(0, 0), texture_size, colors);

        std::shared_ptr<graphics::Texture2D> tex_noise = system.makeTexture(texture_size, graphics::ColorChannel::RGBA);
        tex_noise->range(Pos2D(0, 0), texture_size, noise_c);

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
                                     .uniform("arc_length", arc_length)
                                     .uniform("scale", 1 / v_scale)
                                     .texture("inTexVec", tex_vec)
                                     .texture("inTexNoise", tex_noise)
                                     .boundingSphere(bs),
                                 system.makeProgramFromFiles(resourcePathLocal + "shader/texture-vertex.glsl",
                                                             resourcePathLocal + "shader/lic-fragment.glsl"));

        std::shared_ptr<graphics::Drawable> field_tex =
            system.makePrimitive(graphics::PrimitiveConfig{graphics::RenderPrimitives::TRIANGLES}
                                     .vertexBuffer("position", system.makeBuffer(verticesTex))
                                     .vertexBuffer("texCoords", system.makeBuffer(texCoords))
                                     .indexBuffer(system.makeIndexBuffer(indicesTex))
                                     .texture("inTexture", tex_vec)
                                     .boundingSphere(bs),
                                 system.makeProgramFromFiles(resourcePathLocal + "shader/texture-vertex.glsl",
                                                             resourcePathLocal + "shader/texture-fragment.glsl"));

        std::shared_ptr<graphics::Drawable> noise_tex =
            system.makePrimitive(graphics::PrimitiveConfig{graphics::RenderPrimitives::TRIANGLES}
                                     .vertexBuffer("position", system.makeBuffer(verticesTex))
                                     .vertexBuffer("texCoords", system.makeBuffer(texCoords))
                                     .indexBuffer(system.makeIndexBuffer(indicesTex))
                                     .texture("inTexture", tex_noise)
                                     .boundingSphere(bs),
                                 system.makeProgramFromFiles(resourcePathLocal + "shader/texture-vertex.glsl",
                                                             resourcePathLocal + "shader/texture-fragment.glsl"));

        setGraphics("LIC", lic_tex);
        setGraphics("Field", field_tex);
        setGraphics("Noise", noise_tex);
    }
};

AlgorithmRegister<GraphicsTutorialAlgorithm> dummy("A/2D-LIC", "Renders Line Integral Convolution for 2D Vector Field");
}  // namespace
