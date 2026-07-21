#include "demo.h"
#include "window.h"

namespace muli3
{

class DzhanibekovEffect : public Demo
{
public:
    DzhanibekovEffect(Game& game)
        : Demo(game)
    {
        settings.apply_gravity = false;

        world->CreateBox(24.0f, 0.5f, 24.0f, Vec3{ 0.0f, -3.25f, 0.0f }, Body::static_body);

        spinner = world->CreateEmptyBody(Transform{ Vec3{ 0.0f, 1.2f, 0.0f } });
        spinner->CreateBoxCollider(0.5f, 2.5f, 0.5f, Transform{ Vec3{ 0.0f, 0.0f, 0.0f } });
        spinner->CreateBoxCollider(1.0f, 0.5f, 0.5f, Transform{ Vec3{ 0.75f, 0.0f, 0.0f } });

        spinner->SetAngularVelocity(8.0f, 0.05f, 0.15f);

        spinner->SetLinearDamping(0.0f);
        spinner->SetAngularDamping(0.0f);

        camera.SetPosition(Vec3{ 0.0f, 2.0f, 10.0f });
        camera.SetRotation(-90.0f, -8.0f);
    }

    void UpdateUI() override
    {
        if (spinner == nullptr)
        {
            return;
        }

        Vec2 windowSize = Window::Get()->GetWindowSize();
        ImGui::SetNextWindowPos({ windowSize.x - 8.0f, 8.0f }, ImGuiCond_Always, { 1.0f, 0.0f });

        // Angular momentum
        Vec3 L = spinner->GetWorldInertiaTensor() * spinner->GetAngularVelocity();
        float magnitude = Length(L);

        if (ImGui::Begin(
                "Dzhanibekov", NULL,
                ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_AlwaysAutoResize | ImGuiWindowFlags_NoScrollbar
            ))
        {

            ImGui::Text("Angular momentum");
            ImGui::Separator();
            ImGui::Text("Lx: %.4f", L.x);
            ImGui::Text("Ly: %.4f", L.y);
            ImGui::Text("Lz: %.4f", L.z);
            ImGui::Text("|L|: %.4f", magnitude);
            ImGui::Separator();
        }
        ImGui::End();
    }

private:
    Body* spinner = nullptr;
};

static Demo* CreateDzhanibekovEffect(Game& game)
{
    return new DzhanibekovEffect(game);
}

static int32 dzhanibekov_effect = register_demo("Dynamics", "Dzhanibekov Effect", CreateDzhanibekovEffect, 3);

} // namespace muli3
