#include "demo.h"
#include "game.h"
#include "window.h"

namespace muli3
{

static float coneSwingAngle = 35.0f;
static float coneSwingFrequency = 20.0f;
static float coneSwingDampingRatio = 1.0f;

class ConeSwingJointDemo : public Demo
{
public:
    ConeSwingJointDemo(Game& game)
        : Demo(game)
    {
        RigidBody* ground = world->CreateBox(24.0f, 0.5f, 24.0f, identity, RigidBody::static_body);

        RigidBody* base = world->CreateEmptyBody(Transform{ Vec3{ 0.0f, 4.5f, 0.0f } }, RigidBody::static_body);

        RigidBody* arm = world->CreateEmptyBody(Transform{ Vec3{ 0.0f, 3.5f, 0.0f } });
        arm->CreateBoxCollider(0.25f, 2.0f, 0.25f);
        arm->CreateBoxCollider(0.75f, 0.15f, 0.15f, Transform{ Vec3{ 0.0f, -0.8f, 0.0f } });
        arm->CreateBoxCollider(0.15f, 0.15f, 0.75f, Transform{ Vec3{ 0.0f, -0.8f, 0.0f } });
        arm->SetGyroscopicTorqueEnabled(true);

        world->CreateBallSocketJoint(base, arm, base->GetPosition(), -1.0f);
        world->CreateConeSwingJoint(
            base, arm, -y_axis, DegToRad(coneSwingAngle), coneSwingFrequency, coneSwingDampingRatio, arm->GetMass()
        );

        camera.SetPosition(Vec3{ 0.0f, 4.8f, 8.5f });
        camera.SetRotation(-90.0f, -15.0f);
    }

    void UpdateUI() override
    {
        ImGui::SetNextWindowPos({ Window::Get()->GetWindowSize().x - 5.0f, 5.0f }, ImGuiCond_Once, { 1.0f, 0.0f });

        if (ImGui::Begin("Cone swing joint", nullptr, ImGuiWindowFlags_AlwaysAutoResize))
        {
            ImGui::Text("Cone swing");
            if (ImGui::SliderFloat("Max angle", &coneSwingAngle, 0.0f, 89.0f, "%.1f deg"))
            {
                game.RestartDemo();
            }

            if (ImGui::SliderFloat("Frequency", &coneSwingFrequency, -1.0f, 20.0f, "%.2f"))
            {
                game.RestartDemo();
            }

            if (ImGui::SliderFloat("Damping ratio", &coneSwingDampingRatio, 0.0f, 1.0f, "%.2f"))
            {
                game.RestartDemo();
            }
        }
        ImGui::End();
    }
};

static Demo* CreateConeSwingJointDemo(Game& game)
{
    return new ConeSwingJointDemo(game);
}

static int32 cone_swing_joint = register_demo("Joints", "Cone swing joint", CreateConeSwingJointDemo, 4);

} // namespace muli3
