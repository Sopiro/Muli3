#include "demo.h"
#include "game.h"
#include "window.h"

namespace muli3
{

static float twistMinAngle = -35.0f;
static float twistMaxAngle = 35.0f;
static float twistFrequency = 20.0f;
static float twistDampingRatio = 1.0f;

class TwistAngleJointDemo : public Demo
{
public:
    TwistAngleJointDemo(Game& game)
        : Demo(game)
    {
        world->CreateBox(24.0f, 0.5f, 24.0f, identity, Body::static_body);

        Body* base = world->CreateEmptyBody(Transform{ Vec3{ 0.0f, 4.5f, 0.0f } }, Body::static_body);

        Quat armRotation{ DegToRad(0.0f), z_axis };
        Vec3 armCenter = base->GetPosition() - armRotation.Rotate(y_axis);

        Body* arm = world->CreateEmptyBody(Transform{ armCenter, armRotation });
        arm->CreateBoxCollider(0.25f, 2.0f, 0.25f);
        arm->CreateBoxCollider(0.75f, 0.15f, 0.15f, Transform{ Vec3{ 0.0f, -0.8f, 0.0f } });
        arm->CreateBoxCollider(0.15f, 0.15f, 0.75f, Transform{ Vec3{ 0.0f, -0.8f, 0.0f } });

        world->CreateLineJoint(base, arm);
        world->CreateBallSocketJoint(base, arm, base->GetPosition());
        world->CreateConeSwingJoint(base, arm, -y_axis, DegToRad(30));
        joint = world->CreateTwistAngleJoint(
            base, arm, y_axis, DegToRad(twistMinAngle), DegToRad(twistMaxAngle), twistFrequency, twistDampingRatio
        );

        camera.SetPosition(Vec3{ 0.0f, 4.0f, 8.0f });
        camera.SetRotation(0.0f, 0.0f);
    }

    void UpdateUI() override
    {
        ImGui::SetNextWindowPos({ Window::Get()->GetWindowSize().x - 5.0f, 5.0f }, ImGuiCond_Always, { 1.0f, 0.0f });

        if (ImGui::Begin("Twist angle joint", nullptr, ImGuiWindowFlags_AlwaysAutoResize))
        {
            float currentAngle = joint ? RadToDeg(joint->GetJointAngle()) : 0.0f;
            ImGui::Text("Current angle: %.2f deg", currentAngle);
            ImGui::Text("Ball socket + twist angle");
            ImGui::Text("Only the relative angle around");
            ImGui::Text("the blue world axis is limited.");

            if (ImGui::SliderFloat("Min angle", &twistMinAngle, -180.0f, 180.0f, "%.1f deg"))
            {
                twistMaxAngle = Max(twistMinAngle, twistMaxAngle);
                game.RestartDemo();
            }

            if (ImGui::SliderFloat("Max angle", &twistMaxAngle, -180.0f, 180.0f, "%.1f deg"))
            {
                twistMinAngle = Min(twistMinAngle, twistMaxAngle);
                game.RestartDemo();
            }

            if (ImGui::SliderFloat("Frequency", &twistFrequency, -1.0f, 20.0f, "%.2f"))
            {
                game.RestartDemo();
            }

            if (ImGui::SliderFloat("Damping ratio", &twistDampingRatio, 0.0f, 1.0f, "%.2f"))
            {
                game.RestartDemo();
            }
        }
        ImGui::End();
    }

private:
    TwistAngleJoint* joint = nullptr;
};

static Demo* CreateTwistAngleJointDemo(Game& game)
{
    return new TwistAngleJointDemo(game);
}

static int32 twist_angle_joint = register_demo("Joints", "Twist angle joint", CreateTwistAngleJointDemo, 7);

} // namespace muli3
