#include "demo.h"
#include "window.h"

namespace muli3
{

static int32 count = 20;
static float error = 0.0f;

class BoxStacking final : public Demo
{
public:
    BoxStacking(Game& game)
        : Demo(game)
    {
        RigidBody* ground = world->CreateBox(24.0f, 0.5f, 24.0f, Transform{ Vec3{ 0.0f, -0.25f, 0.0f } }, RigidBody::static_body);

        float size = 1.0f;
        float gap = 0.05f;
        float start = 0.5f + size / 2.0f + gap;

        for (int32 i = 0; i < count; ++i)
        {
            float x = std::sin((float)i * 12.9898f) * error;
            float z = std::sin((float)i * 78.2330f) * error;

            RigidBody* b = world->CreateBox(size, Transform{ Vec3{ x, start + i * (size + gap), z } }, RigidBody::dynamic_body);
        }

        float h = Max(12.0f, (float)count * (size + gap));
        camera.SetPosition(Vec3{ 0.0f, h * 0.45f, h * 1.05f });
        camera.SetRotation(-90.0f, -15.0f);
    }

    void UpdateUI() override
    {
        ImGui::SetNextWindowPos({ Window::Get()->GetWindowSize().x - 5.0f, 5.0f }, ImGuiCond_Once, { 1.0f, 0.0f });

        if (ImGui::Begin("Box stacking", NULL, ImGuiWindowFlags_AlwaysAutoResize))
        {
            ImGui::SliderInt("Count", &count, 1, 100);
            ImGui::SliderFloat("Error", &error, 0.0f, 0.1f, "%.2f");
        }
        ImGui::End();
    }
};

static Demo* CreateBoxStacking(Game& game)
{
    return new BoxStacking(game);
}

static int32 box_stacking = register_demo("Box stacking", CreateBoxStacking, 2);

} // namespace muli3
