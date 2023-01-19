#include <cmath>
#include <cstddef>
#include <fantom/algorithm.hpp>
#include <fantom/graphics.hpp>
#include <fantom/register.hpp>

// needed for BoundinSphere-Calculation and normal calculation
#include <fantom-plugins/utils/Graphics/HelperFunctions.hpp>
#include <fantom-plugins/utils/Graphics/ObjectRenderer.hpp>

using namespace fantom;

namespace {

class GraphicsTutorialAlgorithm : public VisAlgorithm {
 public:
  struct VisOutputs : public VisAlgorithm::VisOutputs {
    // These are the graphic outputs which can be toggled on and off in the
    // interface.
    VisOutputs(fantom::VisOutputs::Control& control) : VisAlgorithm::VisOutputs(control) {
      addGraphics("textureDrawable");
    }
  };

  GraphicsTutorialAlgorithm(InitData& data) : VisAlgorithm(data) {}

  virtual void execute(const Algorithm::Options& /*options*/, const volatile bool& /*abortFlag*/) override {
    auto const& system = graphics::GraphicsSystem::instance();

    // Local Resource Path
    std::string resourcePathLocal = PluginRegistrationService::getInstance().getResourcePath("custom/PlugLIC");

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
    std::shared_ptr<graphics::Texture2D> texture2D = system.makeTexture(texture_size, graphics::ColorChannel::RGBA);
    texture2D->range(Pos2D(0, 0), texture_size, colors);

    // Set Quad vertecies.
    std::vector<PointF<3> > verticesTex(4);
    verticesTex[0] = PointF<3>(-0.5, -0.5, 0.);
    verticesTex[1] = PointF<3>(0.5, -0.5, 0.);
    verticesTex[2] = PointF<3>(-0.5, 0.5, -0.);
    verticesTex[3] = PointF<3>(0.5, 0.5, -0.);

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

    setGraphics("textureDrawable", textureDrawable);
  }
};

AlgorithmRegister<GraphicsTutorialAlgorithm> dummy("A/Texture", "Draws a Circle");
}  // namespace
