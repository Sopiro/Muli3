#include "demo.h"
#include "game.h"
#include "muli3/random.h"
#include "ragdoll.h"
#include "window.h"

namespace muli3
{

static Vec3 SampleUniformHemisphere(Vec2 u)
{
    float z = u[0];
    float r = std::sqrt(std::fmax(0.0f, 1 - z * z));
    float phi = two_pi * u[1];

    return Vec3(r * std::cos(phi), z, r * std::sin(phi));
}

static bool fix = false;
static bool projectile = true;

class RagdollTest : public Demo
{
    Ragdoll ragdoll;

public:
    RagdollTest(Game& game)
        : Demo(game)
    {
        Body* ground = world->CreateBox(50.0f, 0.2f, 50.0f, identity, Body::static_body);

        ragdoll = CreateRagdoll(world, Vec3{ 0, 4, 0 }, 1.0f, 1, 10.0f);

        if (fix)
        {
            Body* body = ragdoll.bones[0].body;
            world->CreateWeldJoint(ground, body, body->GetPosition(), 30);
        }

        camera.SetPosition(Vec3{ 0.0f, 5.0f, 8.0f });
        camera.SetRotation(0.0f, -20.0f);

        // Srand(123);

        if (projectile)
        {
            Body* c = world->CreateSphere(0.5f);
            Vec3 p = SampleUniformHemisphere(RandVec2());
            p *= 8.0f;

            c->SetLinearVelocity(-p * Rand(3.0f, 6.0f) + Vec3{ 0.0f, Rand(1.0f, 5.0f), 0.0f });
            p.y += 0.5f;
            c->SetPosition(p);
        }
    }

    void UpdateUI() override
    {
        ImGui::SetNextWindowPos({ Window::Get()->GetWindowSize().x - 5.0f, 5.0f }, ImGuiCond_Always, { 1.0f, 0.0f });

        if (ImGui::Begin("Ragdoll", NULL, ImGuiWindowFlags_AlwaysAutoResize))
        {
            if (ImGui::Checkbox("Hide Joints", &hideJoint))
            {
                for (int32 i = 0; i < Ragdoll::bone_count; ++i)
                {
                    Joint* joint = ragdoll.bones[i].joint;
                    if (joint)
                    {
                        UserFlag::SetFlag(joint, UserFlag::hide_joint, hideJoint);
                    }
                }
            }
            if (ImGui::Checkbox("Fix", &fix)) game.RestartDemo();
            if (ImGui::Checkbox("Projectile", &projectile)) game.RestartDemo();
        }
        ImGui::End();
    }
};

static Demo* CreateRagdollTest(Game& game)
{
    return new RagdollTest(game);
}

static int32 single_box = register_demo("Ragdoll", "Ragdoll", CreateRagdollTest, 0);

} // namespace muli3
