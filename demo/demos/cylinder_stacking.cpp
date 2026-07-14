#include "demo.h"
#include "game.h"
#include "window.h"

namespace muli3
{

static int32 segments = 16;
static float height = 1.0f;
static float topRadius = 0.45f;
static float bottomRadius = 0.45f;

class CylinderStacking : public Demo
{
public:
    CylinderStacking(Game& game)
        : Demo(game)
    {
        world->CreateBox(28.0f, 0.5f, 18.0f, identity, Body::static_body);

        int32 stackCount = 10;
        float gap = 0.05f;
        float convexRadius = default_radius;
        float groundTop = 0.25f + default_radius;

        for (int32 i = 0; i < stackCount; ++i)
        {
            float halfHeight = height * 0.5f + convexRadius;
            float y = groundTop + halfHeight + i * (halfHeight * 2.0f + gap);
            Body* body = world->CreateCylinder(height, topRadius, bottomRadius, segments, Transform{ Vec3{ -4.0f, y, 0.0f } });
            body->SetGyroscopicTorqueEnabled(true);
        }

        Quat horizontal{ -pi * 0.5f, z_axis };
        for (int32 i = 0; i < stackCount; ++i)
        {
            float outerRadius = Max(topRadius, bottomRadius) + convexRadius;
            float y = groundTop + outerRadius + i * (outerRadius * 2.0f + gap);
            Body* body =
                world->CreateCylinder(height, topRadius, bottomRadius, segments, Transform{ Vec3{ 4.0f, y, 0.0f }, horizontal });
            body->SetGyroscopicTorqueEnabled(true);
        }

        camera.SetPosition(Vec3{ 0.0f, 8.0f, 18.0f });
        camera.SetRotation(-90.0f, -10.0f);
    }

    void UpdateUI() override
    {
        ImGui::SetNextWindowPos({ Window::Get()->GetWindowSize().x - 5.0f, 5.0f }, ImGuiCond_Always, { 1.0f, 0.0f });

        if (ImGui::Begin("Cylinder stacking", NULL, ImGuiWindowFlags_AlwaysAutoResize))
        {
            if (ImGui::SliderInt("Segments", &segments, 3, 64))
            {
                game.RestartDemo();
            }
            if (ImGui::SliderFloat("Height", &height, 0.1f, 2.0f, "%.2f"))
            {
                game.RestartDemo();
            }
            if (ImGui::SliderFloat("Top radius", &topRadius, 0.0f, 1.0f, "%.2f"))
            {
                game.RestartDemo();
            }
            if (ImGui::SliderFloat("Bottom radius", &bottomRadius, 0.0f, 1.0f, "%.2f"))
            {
                game.RestartDemo();
            }
        }
        ImGui::End();
    }
};

static Demo* CreateCylinderStacking(Game& game)
{
    return new CylinderStacking(game);
}

static int32 cylinder_stacking = register_demo("Stacking", "Cylinder stacking", CreateCylinderStacking, 3);

} // namespace muli3
