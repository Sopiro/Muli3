#include "demo.h"
#include "game.h"
#include "muli3/random.h"
#include "window.h"

namespace muli3
{

static int32 bodyCount = 64;
static int32 maxVertexCount = 16;

class ConvexShapeTest : public Demo
{
public:
    ConvexShapeTest(Game& game)
        : Demo(game)
    {
        world->CreateBox(28.0f, 0.5f, 28.0f, identity, Body::static_body);

        float r = 1.5f;
        float xzBound = std::min(10.0f, bodyCount * 0.1f);
        float yBound = std::max(8.0f, bodyCount * 0.1f);

        for (int32 i = 0; i < bodyCount; ++i)
        {
            int32 vertexCount = int32(Rand(4, maxVertexCount));
            std::vector<Vec3> vertices;
            vertices.reserve(vertexCount);
            for (int32 j = 0; j < vertexCount; ++j)
            {
                vertices.push_back(RandVec3() * r);
            }

            Vec3 p = RandVec3({ -xzBound, 1, -xzBound }, { xzBound, yBound, xzBound });

            world->CreateConvex(vertices, p);
        }

        camera.SetPosition(Vec3{ 0.0f, 6.0f, 20.0f });
        camera.SetRotation(-90.0f, -5.0f);
    }

    void UpdateUI() override
    {
        ImGui::SetNextWindowPos({ Window::Get()->GetWindowSize().x - 5.0f, 5.0f }, ImGuiCond_Once, { 1.0f, 0.0f });

        if (ImGui::Begin("Convex shape", NULL, ImGuiWindowFlags_AlwaysAutoResize))
        {
            if (ImGui::SliderInt("Body count", &bodyCount, 1, 1000))
            {
                game.RestartDemo();
            }
            if (ImGui::SliderInt("Max vertex count", &maxVertexCount, 4, 64))
            {
                game.RestartDemo();
            }
        }
        ImGui::End();
    }
};

static Demo* CreateConvexShapeTest(Game& game)
{
    return new ConvexShapeTest(game);
}

static int32 convex_shapes = register_demo("Shapes", "Convex shape", CreateConvexShapeTest, 1);

} // namespace muli3
