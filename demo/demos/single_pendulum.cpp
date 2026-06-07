#include "demo.h"
#include "game.h"
#include "window.h"

namespace muli3
{

static float frequency = 10.0f;
static float dampingRatio = 1.0f;

class SinglePendulum : public Demo
{
public:
    SinglePendulum(Game& game)
        : Demo(game)
    {
        Body* ground = world->CreateBox(40.0f, 0.5f, 40.0f, identity, Body::static_body);

        Vec3 anchor{ 0.0f, 6.0f, 0.0f };
        Vec3 position{ -2.4f, 3.8f, 1.6f };

        Body* body = world->CreateBox(0.45f, Transform{ position });
        body->SetAngularVelocity(0.8f, -0.35f, 0.45f);

        world->CreateBallSocketJoint(body, ground, anchor, frequency, dampingRatio);

        camera.SetPosition(Vec3{ 0.0f, 5.2f, 9.0f });
        camera.SetRotation(-90.0f, -18.0f);
    }

    void UpdateUI() override
    {
        ImGui::SetNextWindowPos({ Window::Get()->GetWindowSize().x - 5.0f, 5.0f }, ImGuiCond_Once, { 1.0f, 0.0f });

        if (ImGui::Begin("Single pendulum", NULL, ImGuiWindowFlags_AlwaysAutoResize))
        {
            ImGui::PushID(0);
            ImGui::Text("Frequency");
            if (ImGui::SliderFloat("##Frequency", &frequency, 0.0f, 20.0f, "%.2f"))
            {
                game.RestartDemo();
            }

            ImGui::Text("Damping ratio");
            if (ImGui::SliderFloat("##Damping ratio", &dampingRatio, 0.0f, 1.0f, "%.2f"))
            {
                game.RestartDemo();
            }
            ImGui::PopID();
        }
        ImGui::End();
    }
};

static Demo* CreateSinglePendulum(Game& game)
{
    return new SinglePendulum(game);
}

static int32 single_pendulum = register_demo("Joints", "Single pendulum", CreateSinglePendulum, 0);

} // namespace muli3
