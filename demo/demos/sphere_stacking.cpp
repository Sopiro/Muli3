#include "demo.h"
#include "game.h"
#include "window.h"

namespace muli3
{

static int32 count = 10;
static float error = 0.0f;

class SphereStacking : public Demo
{
public:
    SphereStacking(Game& game)
        : Demo(game)
    {
        world->CreateBox(24.0f, 0.5f, 24.0f, identity, Body::static_body);

        float radius = 0.5f;
        float diameter = radius * 2.0f;
        float gap = 0.05f;
        float start = 0.25f + radius + gap;

        for (int32 i = 0; i < count; ++i)
        {
            float x = std::sin((float)i * 12.9898f) * error;
            float z = std::sin((float)i * 78.2330f) * error;

            world->CreateSphere(radius, Transform{ Vec3{ x, start + i * (diameter + gap), z } }, Body::dynamic_body);
        }

        float h = Max(12.0f, (float)count * (diameter + gap));
        camera.SetPosition(Vec3{ 0.0f, h * 0.7f, h * 1.5f });
        camera.SetRotation(-90.0f, -15.0f);
    }

    void UpdateUI() override
    {
        ImGui::SetNextWindowPos({ Window::Get()->GetWindowSize().x - 5.0f, 5.0f }, ImGuiCond_Always, { 1.0f, 0.0f });

        if (ImGui::Begin("Sphere stacking", NULL, ImGuiWindowFlags_AlwaysAutoResize))
        {
            if (ImGui::SliderInt("Count", &count, 1, 100)) game.RestartDemo();
            if (ImGui::SliderFloat("Error", &error, 0.0f, 0.1f, "%.2f")) game.RestartDemo();
        }
        ImGui::End();
    }
};

static Demo* CreateSphereStacking(Game& game)
{
    return new SphereStacking(game);
}

static int32 sphere_stacking = register_demo("Stacking", "Sphere stacking", CreateSphereStacking, 1);

} // namespace muli3
