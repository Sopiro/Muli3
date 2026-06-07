#include "demo.h"
#include "game.h"
#include "window.h"

namespace muli3
{

static int32 selection = 0;
static float threshold = 2.0f;
static const char* items[] = { "Sphere", "Box", "Capsule" };
static int32 selection2 = 0;
static const char* items2[] = { "Quadratic", "Linear" };

class RestitutionTest : public Demo
{
public:
    RestitutionTest(Game& game)
        : Demo(game)
    {
        Body* ground = world->CreateBox(50.0f, 0.5f, 50.0f, identity, Body::static_body);
        ground->SetRestitutionThreshold(threshold);

        int32 count = 11;
        float gap = 0.5f;
        float size = 0.3f;

        float xStart = -(count - 1) / 2.0f * gap;
        float yStart = 6.0f;

        Body* b;
        for (int32 i = 0; i < count; ++i)
        {
            switch (selection)
            {
            case 0:
                b = world->CreateSphere(size * 0.5f);
                break;
            case 1:
                b = world->CreateBox(size);
                break;
            case 2:
                b = world->CreateCapsule(size * 0.8f, size * 0.8f * 0.5f);
                break;
            default:
                MuliAssert(false);
                break;
            }

            b->SetPosition(xStart + gap * i, yStart, 0);
            float attenuation = (count - i) / (float)count;
            float restitution = 1.0f - (selection2 == 0 ? attenuation * attenuation : attenuation);
            b->SetRestitution(restitution);
            b->SetRestitutionThreshold(threshold);
        }

        camera.SetPosition(Vec3{ 0.0f, 4.0f, 8.0f });
        camera.SetRotation(-90.0f, -12.0f);
    }

    void UpdateUI() override
    {
        ImGui::SetNextWindowPos({ Window::Get()->GetWindowSize().x - 5, 5 }, ImGuiCond_Once, { 1.0f, 0.0f });

        if (ImGui::Begin("Resitution test", NULL, ImGuiWindowFlags_AlwaysAutoResize))
        {
            ImGui::Text("Shape");
            ImGui::PushID(0);
            if (ImGui::ListBox("##Shape", &selection, items, IM_ARRAYSIZE(items)))
            {
                game.RestartDemo();
            }
            ImGui::PopID();

            ImGui::Text("Attenuation");
            if (ImGui::ListBox("##Attenuation", &selection2, items2, IM_ARRAYSIZE(items2)))
            {
                game.RestartDemo();
            }

            ImGui::Text("Restitution threshold");
            if (ImGui::SliderFloat("##Restitution threshold", &threshold, 2.0f, 10.0f, "%.2f m/s"))
            {
                game.RestartDemo();
            }
        }
        ImGui::End();
    }
};

static Demo* CreateRestitutionTest(Game& game)
{
    return new RestitutionTest(game);
}

static int32 restitution_test = register_demo("Dynamics", "Restitution test", CreateRestitutionTest, 1);

} // namespace muli3
