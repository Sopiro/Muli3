#include "demo.h"
#include "game.h"
#include "window.h"

namespace muli3
{

static int32 selection = 0;
static float frequency = 15.0f;
static float dampingRatio = 0.5f;

class MultiPendulum : public Demo
{
public:
    MultiPendulum(Game& game)
        : Demo(game)
    {
        Body* ground = world->CreateBox(40.0f, 0.5f, 40.0f, identity, Body::static_body);

        float xStart = 0.0f;
        float yStart = 5.0f;
        float sizeW = 0.3f;
        float sizeH = 0.15f;
        float gap = 0.1f;

        Body* bodyA = world->CreateBox(sizeW, sizeH, sizeH, Transform{ Vec3{ xStart - (gap + sizeW), yStart, 0.0f } });
        world->CreateBallSocketJoint(ground, bodyA, Vec3{ xStart, yStart, 0.0f }, -1.0f);

        int32 count = 12;
        for (int32 i = 1; i < count; ++i)
        {
            Body* bodyB =
                world->CreateBox(sizeW, sizeH, sizeH, Transform{ Vec3{ xStart - (gap + sizeW) * (i + 1), yStart, 0.0f } });

            world->CreateBallSocketJoint(
                bodyA, bodyB, Vec3{ xStart - (sizeW + gap) * 0.5f - (gap + sizeW) * i, yStart, 0.0f }, frequency, dampingRatio
            );
            // world->CreateLimitedRevoluteAngleJoint(
            //     bodyA, bodyB, -z_axis, DegToRad(-60), DegToRad(60), frequency, dampingRatio, jointMass
            // );
            // world->CreateRevoluteJoint(
            //     bodyA, bodyB, Vec3{ xStart - (sizeW + gap) * 0.5f - (gap + sizeW) * i, yStart, 0.0f }, -z_axis, frequency,
            //     dampingRatio, jointMass
            // );

            bodyA = bodyB;
        }

        camera.SetPosition(Vec3{ -2.5f, 4.8f, 8.5f });
        camera.SetRotation(-90.0f, -18.0f);
    }

    void UpdateUI() override
    {
        ImGui::SetNextWindowPos({ Window::Get()->GetWindowSize().x - 5.0f, 5.0f }, ImGuiCond_Once, { 1.0f, 0.0f });

        if (ImGui::Begin("Multi pendulum", NULL, ImGuiWindowFlags_AlwaysAutoResize))
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

static Demo* CreateMultiPendulum(Game& game)
{
    return new MultiPendulum(game);
}

static int32 multi_pendulum = register_demo("Joints", "Multi pendulum", CreateMultiPendulum, 1);

} // namespace muli3
