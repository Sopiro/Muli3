#include "demo.h"
#include "game.h"
#include "window.h"

namespace muli3
{

static float revoluteFrequency = 20.0f;
static float revoluteDampingRatio = 1.0f;
static float revoluteMinAngle = -60.0f;
static float revoluteMaxAngle = 60.0f;

static bool revoluteMotorEnabled = false;
static float revoluteMotorSpeed = 90.0f;
static float revoluteMaxMotorTorque = 20.0f;

class RevoluteJointDemo : public Demo
{
public:
    RevoluteJointDemo(Game& game)
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

        joint = world->CreateLimitedRevoluteJoint(
            base, arm, base->GetPosition(), z_axis, DegToRad(revoluteMinAngle), DegToRad(revoluteMaxAngle), revoluteFrequency,
            revoluteDampingRatio
        );
        joint->SetMotorEnabled(revoluteMotorEnabled);
        joint->SetMotorSpeed(DegToRad(revoluteMotorSpeed));
        joint->SetMaxMotorTorque(revoluteMaxMotorTorque);

        camera.SetPosition(Vec3{ 0.0f, 4.0f, 8.0f });
        camera.SetRotation(0.0f, 0.0f);
    }

    void UpdateUI() override
    {
        ImGui::SetNextWindowPos({ Window::Get()->GetWindowSize().x - 5.0f, 5.0f }, ImGuiCond_Always, { 1.0f, 0.0f });

        if (ImGui::Begin("Revolute joint", nullptr, ImGuiWindowFlags_AlwaysAutoResize))
        {
            float currentAngle = joint ? RadToDeg(joint->GetJointAngle()) : 0.0f;
            ImGui::Text("Current angle: %.2f deg", currentAngle);
            ImGui::Text("3D hinge joint with twist limit");

            if (ImGui::Checkbox("Enable motor", &revoluteMotorEnabled))
            {
                joint->SetMotorEnabled(revoluteMotorEnabled);
                joint->GetBodyB()->Awake();
            }

            if (ImGui::SliderFloat("Motor speed", &revoluteMotorSpeed, -360.0f, 360.0f, "%.1f deg/s"))
            {
                joint->SetMotorSpeed(DegToRad(revoluteMotorSpeed));
            }

            if (ImGui::SliderFloat("Max motor torque", &revoluteMaxMotorTorque, 0.0f, 100.0f, "%.1f"))
            {
                joint->SetMaxMotorTorque(revoluteMaxMotorTorque);
            }

            if (ImGui::SliderFloat("Min angle", &revoluteMinAngle, -180.0f, 180.0f, "%.1f deg"))
            {
                revoluteMaxAngle = Max(revoluteMinAngle, revoluteMaxAngle);
                game.RestartDemo();
            }

            if (ImGui::SliderFloat("Max angle", &revoluteMaxAngle, -180.0f, 180.0f, "%.1f deg"))
            {
                revoluteMinAngle = Min(revoluteMinAngle, revoluteMaxAngle);
                game.RestartDemo();
            }

            if (ImGui::SliderFloat("Frequency", &revoluteFrequency, -1.0f, 20.0f, "%.2f"))
            {
                game.RestartDemo();
            }

            if (ImGui::SliderFloat("Damping ratio", &revoluteDampingRatio, 0.0f, 1.0f, "%.2f"))
            {
                game.RestartDemo();
            }
        }
        ImGui::End();
    }

private:
    RevoluteJoint* joint = nullptr;
};

static Demo* CreateRevoluteJointDemo(Game& game)
{
    return new RevoluteJointDemo(game);
}

static int32 revolute_joint = register_demo("Joints", "Revolute joint", CreateRevoluteJointDemo, 8);

} // namespace muli3
