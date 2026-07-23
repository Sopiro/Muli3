#include "demo.h"
#include "game.h"
#include "window.h"

namespace muli3
{

static int32 rows = 10;

class Pyramid : public Demo
{
public:
    Pyramid(Game& game)
        : Demo(game)
    {
        float radius = 0.0f;
        float size = 1.0f;
        float gap = radius + size * 0.02f;
        float xStep = size + gap;
        float yStep = size + gap;
        float xStart = -(rows - 1.0f) * xStep * 0.5f;
        float yStart = (0.5f + default_radius + size) * 0.5f + gap;

        float density = 1;

        for (int32 y = 0; y < rows; ++y)
        {
            for (int32 x = 0; x < rows - y; ++x)
            {
                Body* b = world->CreateBox(
                    size,
                    Transform{
                        Vec3{ xStart + y * xStep * 0.5f + x * xStep, yStart + y * yStep, 0.0f },
                    },
                    Body::dynamic_body, radius, density
                );
            }
        }

        float h = Max(12.0f, (float)rows * yStep);
        world->CreateBox(h * 2, 0.5f, h * 2, identity, Body::static_body);
        camera.SetPosition(Vec3{ 0.0f, h * 0.7f, h * 1.5f });
        camera.SetRotation(0.0f, -18.0f);
    }

    void UpdateUI() override
    {
        ImGui::SetNextWindowPos({ Window::Get()->GetWindowSize().x - 5.0f, 5.0f }, ImGuiCond_Always, { 1.0f, 0.0f });

        if (ImGui::Begin("Pyramid", NULL, ImGuiWindowFlags_AlwaysAutoResize))
        {
            if (ImGui::SliderInt("Rows", &rows, 1, 100)) game.RestartDemo();
        }
        ImGui::End();
    }
};

static Demo* CreatePyramid(Game& game)
{
    return new Pyramid(game);
}

static int32 pyramid = register_demo("Stacking", "Pyramid", CreatePyramid, 3);

} // namespace muli3
