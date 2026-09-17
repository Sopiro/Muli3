#include "demo.h"
#include "game.h"
#include "window.h"

namespace muli3
{

static float maxSwingAngle = 30.0f;
static float minTwistAngle = -35.0f;
static float maxTwistAngle = 35.0f;

class SwingTwistJointDemo : public Demo
{
public:
    SwingTwistJointDemo(Game& game)
        : Demo(game)
    {
        world->CreateBox(24.0f, 0.5f, 24.0f, identity, Body::static_body);
        Body* base = world->CreateEmptyBody(Transform{ Vec3{ 0.0f, 4.5f, 0.0f } }, Body::static_body);
        Body* arm = world->CreateEmptyBody(Transform{ Vec3{ 0.0f, 3.5f, 0.0f } });
        arm->CreateBoxCollider(0.25f, 2.0f, 0.25f);
        arm->CreateBoxCollider(0.75f, 0.15f, 0.15f, Transform{ Vec3{ 0.0f, -0.8f, 0.0f } });
        joint = world->CreateSwingTwistJoint(
            base, arm, base->GetPosition(), -y_axis, DegToRad(maxSwingAngle), DegToRad(minTwistAngle), DegToRad(maxTwistAngle)
        );
        camera.SetPosition(Vec3{ 0.0f, 4.0f, 8.0f });
        camera.SetRotation(0.0f, 0.0f);
    }

    void UpdateUI() override
    {
        ImGui::SetNextWindowPos({ Window::Get()->GetWindowSize().x - 5.0f, 5.0f }, ImGuiCond_Always, { 1.0f, 0.0f });
        if (ImGui::Begin("Swing twist joint", nullptr, ImGuiWindowFlags_AlwaysAutoResize))
        {
            ImGui::Text("Swing: %.2f deg", RadToDeg(joint->GetSwingAngle()));
            ImGui::Text("Twist: %.2f deg", RadToDeg(joint->GetTwistAngle()));
            if (ImGui::SliderFloat("Max swing", &maxSwingAngle, 0.0f, 180.0f, "%.1f deg"))
            {
                joint->SetMaxSwingAngle(DegToRad(maxSwingAngle));
                joint->GetBodyB()->Awake();
            }
            if (ImGui::SliderFloat("Min twist", &minTwistAngle, -180.0f, 180.0f, "%.1f deg"))
            {
                maxTwistAngle = Max(minTwistAngle, maxTwistAngle);
                joint->SetMinTwistAngle(DegToRad(minTwistAngle));
                joint->GetBodyB()->Awake();
            }
            if (ImGui::SliderFloat("Max twist", &maxTwistAngle, -180.0f, 180.0f, "%.1f deg"))
            {
                minTwistAngle = Min(minTwistAngle, maxTwistAngle);
                joint->SetMaxTwistAngle(DegToRad(maxTwistAngle));
                joint->GetBodyB()->Awake();
            }
        }
        ImGui::End();
    }

private:
    SwingTwistJoint* joint;
};

static Demo* CreateSwingTwistJointDemo(Game& game)
{
    return new SwingTwistJointDemo(game);
}

static int32 swing_twist_joint = register_demo("Joints", "Swing twist joint", CreateSwingTwistJointDemo, 9);

} // namespace muli3
