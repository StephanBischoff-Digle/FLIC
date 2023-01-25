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
        }
    };
    struct VisOutputs : public VisAlgorithm::VisOutputs {
        VisOutputs(fantom::VisOutputs::Control& control) : VisAlgorithm::VisOutputs(control) {
            addGraphics("textureDrawable");
            addGraphics("noiseTextureDrawable");
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

        // get max coordinates
        PointSetBase::BoundingBox test = grid->getBoundingBox();

        // Local Resource Path
        std::string resourcePathLocal = PluginRegistrationService::getInstance().getResourcePath("custom/PlugLIC/");

        // Draw a circle at cx, cy with radius r and stroke width s
        double r = 50.;
        double rs = std::pow(r, 2.);
        double s = 3.;
        double rss = std::pow(r + s, 2.);
        double cx = 100;
        double cy = 150;
        size_t t_width = 250;
        size_t t_height = 250;

        // Generate color vector
        std::vector<Color> colors(t_width * t_height);

        // Set colors
        for (size_t y = 0; y < t_height; ++y) {
            for (size_t x = 0; x < t_width; ++x) {
                double dx = std::pow(cx - x, 2);
                double dy = std::pow(cy - y, 2);
                double dist2 = dx + dy;
                size_t idx = y * t_width + x;
                if (dist2 > rs && dist2 < rss) {
                    colors[idx] = Color(0., 0., 0., 1.);
                } else {
                    colors[idx] = Color(.1, 0.25, .8, 1.);
                }
            }
        }

        // Generate 2D Texture
        Size2D texture_size{t_width, t_height};
        std::shared_ptr<graphics::Texture2D> texture2D = 
            system.makeTexture(texture_size, graphics::ColorChannel::RGBA);
        texture2D->range(Pos2D(0, 0), texture_size, colors);

        SimplexNoise noise{};
        std::vector<Color> noiseColors;
        for (float x = 0.; x < t_width; x++) {
            for (float y = 0; y < t_height; y++) {
                r = std::abs(noise.noise(x, y));
                noiseColors.push_back(Color(r, r, r, 1.));
            }
        }

        Size2D noiseTexture_size{t_width, t_height};
        std::shared_ptr<graphics::Texture2D> noiseTexture2D =
            system.makeTexture(noiseTexture_size, graphics::ColorChannel::RGBA);

        std::vector<Color> finalColors;
        for (size_t x = 0; x < t_width * t_height; x++) {
            float r = (noiseColors[x].r() + colors[x].r()) / 2;
            float g = (noiseColors[x].g() + colors[x].g()) / 2;
            float b = (noiseColors[x].b() + colors[x].b()) / 2;
            finalColors.push_back(Color(r, g, b, 1.));
        }

        noiseTexture2D->range(Pos2D(0, 0), noiseTexture_size, finalColors);

        // Set Quad vertecies.
        std::vector<PointF<3>> verticesTex(4);
        verticesTex[0] = PointF<3>(test[0].first, test[1].first, 0.);
        verticesTex[1] = PointF<3>(test[0].second, test[1].first, 0.);
        verticesTex[2] = PointF<3>(test[0].first, test[1].second, -0.);
        verticesTex[3] = PointF<3>(test[0].second, test[1].second, -0.);

        // These are the 3D-TextureCoordinates describing the borders of the
        // Texture.
        std::vector<PointF<2> > texCoords(4);
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
        std::shared_ptr<graphics::Drawable> textureDrawable =
            system.makePrimitive(graphics::PrimitiveConfig{graphics::RenderPrimitives::TRIANGLES}
                                     .vertexBuffer("position", system.makeBuffer(verticesTex))
                                     .vertexBuffer("texCoords", system.makeBuffer(texCoords))
                                     .indexBuffer(system.makeIndexBuffer(indicesTex))
                                     .texture("inTexture", texture2D)
                                     .boundingSphere(bs),
                                 system.makeProgramFromFiles(resourcePathLocal + "shader/texture-vertex.glsl",
                                                             resourcePathLocal + "shader/texture-fragment.glsl"));

        std::shared_ptr<graphics::Drawable> noiseTextureDrawable =
            system.makePrimitive(graphics::PrimitiveConfig{graphics::RenderPrimitives::TRIANGLES}
                                     .vertexBuffer("position", system.makeBuffer(verticesTex))
                                     .vertexBuffer("texCoords", system.makeBuffer(texCoords))
                                     .indexBuffer(system.makeIndexBuffer(indicesTex))
                                     .texture("inTexture", noiseTexture2D)
                                     .boundingSphere(bs),
                                 system.makeProgramFromFiles(resourcePathLocal + "shader/texture-vertex.glsl",
                                                             resourcePathLocal + "shader/texture-fragment.glsl"));

        setGraphics("noiseTextureDrawable", noiseTextureDrawable);
        setGraphics("textureDrawable", textureDrawable);
    }
};

AlgorithmRegister<GraphicsTutorialAlgorithm> dummy("A/Texture", "Draws a Circle");
}  // namespace
