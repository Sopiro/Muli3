#include "demo.h"
#include "game.h"
#include "window.h"

namespace muli3
{

static int32 rows = 10;
static int32 xCount = 15;
static int32 zCount = 15;
static float spacing = 2.0f;

class ManyPyramid : public Demo
{
public:
    ManyPyramid(Game& game)
        : Demo(game)
    {
        float size = 1.0f;
        float radius = 0.0f;
        float gap = size * 0.01f;
        float xzStep = size + gap;
        float yStep = size + gap;
        float xStart = -(rows - 1.0f) * xzStep * 0.5f;
        float yStart = 0.25f + size * 0.5f;
        float pyramidSpan = (rows - 1.0f) * xzStep + size;
        float pitch = pyramidSpan + spacing;
        float centerX = -(xCount - 1.0f) * pitch * 0.5f;
        float centerZ = -(zCount - 1.0f) * pitch * 0.5f;

        for (int32 z = 0; z < zCount; ++z)
        {
            for (int32 x = 0; x < xCount; ++x)
            {
                Vec3 origin{ centerX + x * pitch, 0.0f, centerZ + z * pitch };

                for (int32 y = 0; y < rows; ++y)
                {
                    for (int32 i = 0; i < rows - y; ++i)
                    {
                        world->CreateBox(
                            size, Vec3{ origin.x + xStart + y * xzStep * 0.5f + i * xzStep, yStart + y * yStep, origin.z },
                            Body::dynamic_body, radius
                        );
                    }
                }
            }
        }

        float groundHalfX = 0.5f * ((xCount - 1.0f) * pitch + pyramidSpan) + 2.0f;
        float groundHalfZ = 0.5f * ((zCount - 1.0f) * pitch + pyramidSpan) + 2.0f;
        float h = Max(12.0f, (float)rows * yStep);
        float r = Max(groundHalfX, groundHalfZ);

        world->CreateBox(groundHalfX * 2.0f, 0.5f, groundHalfZ * 2.0f, identity, Body::static_body, 0.0f);

        camera.SetPosition(Vec3{ 0.0f, Max(h * 0.9f, r * 0.7f), r * 2.1f });
        camera.SetRotation(0.0f, -18.0f);
    }

    void UpdateUI() override
    {
        ImGui::SetNextWindowPos({ Window::Get()->GetWindowSize().x - 5.0f, 5.0f }, ImGuiCond_Always, { 1.0f, 0.0f });

        if (ImGui::Begin("Many pyramid", NULL, ImGuiWindowFlags_AlwaysAutoResize))
        {
            if (ImGui::SliderInt("Rows", &rows, 1, 20)) game.RestartDemo();
            if (ImGui::SliderInt("X count", &xCount, 1, 20)) game.RestartDemo();
            if (ImGui::SliderInt("Z count", &zCount, 1, 20)) game.RestartDemo();
            if (ImGui::SliderFloat("Spacing", &spacing, 0.5f, 10.0f, "%.1f")) game.RestartDemo();
        }
        ImGui::End();
    }
};

static Demo* CreateManyPyramid(Game& game)
{
    return new ManyPyramid(game);
}

static int32 many_pyramid = register_demo("Stacking", "Many pyramid", CreateManyPyramid, 4);

} // namespace muli3
