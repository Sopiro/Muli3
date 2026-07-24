#include "demo.h"
#include "game.h"
#include "window.h"

namespace muli3
{

static float universalFrequency = 20.0f;
static float universalDampingRatio = 1.0f;
static bool universalSteeringMotorEnabled = true;
static float universalTargetSteeringAngle = 25.0f;
static float universalSteeringFrequency = 5.0f;
static float universalSteeringDampingRatio = 1.0f;
static float universalMaxSteeringTorque = 20.0f;
static bool universalSteeringLimitEnabled = true;
static float universalMinSteeringAngle = -40.0f;
static float universalMaxSteeringAngle = 40.0f;
static bool universalSpinMotorEnabled = true;
static float universalSpinSpeed = 180.0f;
static float universalMaxSpinTorque = 10.0f;
static bool universalSpinLimitEnabled = false;
static float universalMinSpinAngle = -90.0f;
static float universalMaxSpinAngle = 90.0f;

class UniversalAngleJointDemo : public Demo
{
public:
    UniversalAngleJointDemo(Game& game)
        : Demo(game)
    {
        world->CreateBox(24.0f, 0.5f, 24.0f, identity, Body::static_body);

        CollisionFilter filter;
        filter.group = -1;

        Vec3 center{ 0.0f, 4.0f, 0.0f };
        Body* base = world->CreateEmptyBody(Transform{ center }, Body::static_body);
        base->CreateBoxCollider(2.5f, 0.2f, 0.2f, Transform{ Vec3{ -1.45f, 0.0f, 0.0f } });
        base->SetCollisionFilter(filter);

        Quat wheelRotation{ -0.5f * pi, z_axis };
        Body* wheel = world->CreateCylinder(0.4f, 1.0f, 1.0f, 24, Transform{ center, wheelRotation });
        wheel->SetCollisionFilter(filter);

        world->CreateBallSocketJoint(base, wheel, center, universalFrequency, universalDampingRatio);
        joint = world->CreateUniversalAngleJoint(base, wheel, y_axis, x_axis, universalFrequency, universalDampingRatio);

        joint->SetSteeringMotorEnabled(universalSteeringMotorEnabled);
        joint->SetTargetSteeringAngle(DegToRad(universalTargetSteeringAngle));
        joint->SetSteeringFrequency(universalSteeringFrequency);
        joint->SetSteeringDampingRatio(universalSteeringDampingRatio);
        joint->SetMaxSteeringTorque(universalMaxSteeringTorque);
        joint->SetSteeringLimitEnabled(universalSteeringLimitEnabled);
        joint->SetSteeringMinAngle(DegToRad(universalMinSteeringAngle));
        joint->SetSteeringMaxAngle(DegToRad(universalMaxSteeringAngle));

        joint->SetSpinMotorEnabled(universalSpinMotorEnabled);
        joint->SetSpinSpeed(DegToRad(universalSpinSpeed));
        joint->SetMaxSpinTorque(universalMaxSpinTorque);
        joint->SetSpinLimitEnabled(universalSpinLimitEnabled);
        joint->SetSpinMinAngle(DegToRad(universalMinSpinAngle));
        joint->SetSpinMaxAngle(DegToRad(universalMaxSpinAngle));

        camera.SetPosition(Vec3{ 0.0f, 4.5f, 8.0f });
        camera.SetRotation(0.0f, -10.0f);
    }

    void UpdateUI() override
    {
        ImGui::SetNextWindowPos({ Window::Get()->GetWindowSize().x - 5.0f, 5.0f }, ImGuiCond_Always, { 1.0f, 0.0f });

        if (ImGui::Begin("Universal angle joint", nullptr, ImGuiWindowFlags_AlwaysAutoResize))
        {
            float steeringAngle = joint ? RadToDeg(joint->GetSteeringAngle()) : 0.0f;
            float spinAngle = joint ? RadToDeg(joint->GetSpinAngle()) : 0.0f;
            ImGui::Text("Steering angle: %.2f deg", steeringAngle);
            ImGui::Text("Spin angle: %.2f deg", spinAngle);
            ImGui::Text("Red axis: steering");
            ImGui::Text("Green axis: spin");

            if (ImGui::Checkbox("Steering motor", &universalSteeringMotorEnabled))
            {
                joint->SetSteeringMotorEnabled(universalSteeringMotorEnabled);
            }

            if (ImGui::SliderFloat("Target steering", &universalTargetSteeringAngle, -40.0f, 40.0f, "%.1f deg"))
            {
                joint->SetTargetSteeringAngle(DegToRad(universalTargetSteeringAngle));
            }

            if (ImGui::SliderFloat("Steering frequency", &universalSteeringFrequency, 0.0f, 20.0f, "%.2f"))
            {
                joint->SetSteeringFrequency(universalSteeringFrequency);
            }

            if (ImGui::SliderFloat("Steering damping", &universalSteeringDampingRatio, 0.0f, 3.0f, "%.2f"))
            {
                joint->SetSteeringDampingRatio(universalSteeringDampingRatio);
            }

            if (ImGui::SliderFloat("Max steering torque", &universalMaxSteeringTorque, 0.0f, 100.0f, "%.1f"))
            {
                joint->SetMaxSteeringTorque(universalMaxSteeringTorque);
            }

            if (ImGui::Checkbox("Steering limit", &universalSteeringLimitEnabled))
            {
                joint->SetSteeringLimitEnabled(universalSteeringLimitEnabled);
            }

            if (ImGui::SliderFloat("Min steering", &universalMinSteeringAngle, -90.0f, 0.0f, "%.1f deg"))
            {
                universalMaxSteeringAngle = Max(universalMinSteeringAngle, universalMaxSteeringAngle);
                joint->SetSteeringMinAngle(DegToRad(universalMinSteeringAngle));
                joint->SetSteeringMaxAngle(DegToRad(universalMaxSteeringAngle));
            }

            if (ImGui::SliderFloat("Max steering", &universalMaxSteeringAngle, 0.0f, 90.0f, "%.1f deg"))
            {
                universalMinSteeringAngle = Min(universalMinSteeringAngle, universalMaxSteeringAngle);
                joint->SetSteeringMinAngle(DegToRad(universalMinSteeringAngle));
                joint->SetSteeringMaxAngle(DegToRad(universalMaxSteeringAngle));
            }

            if (ImGui::Checkbox("Spin motor", &universalSpinMotorEnabled))
            {
                joint->SetSpinMotorEnabled(universalSpinMotorEnabled);
            }

            if (ImGui::SliderFloat("Spin speed", &universalSpinSpeed, -720.0f, 720.0f, "%.1f deg/s"))
            {
                joint->SetSpinSpeed(DegToRad(universalSpinSpeed));
            }

            if (ImGui::SliderFloat("Max spin torque", &universalMaxSpinTorque, 0.0f, 100.0f, "%.1f"))
            {
                joint->SetMaxSpinTorque(universalMaxSpinTorque);
            }

            if (ImGui::Checkbox("Spin limit", &universalSpinLimitEnabled))
            {
                joint->SetSpinLimitEnabled(universalSpinLimitEnabled);
            }

            if (ImGui::SliderFloat("Min spin", &universalMinSpinAngle, -180.0f, 0.0f, "%.1f deg"))
            {
                universalMaxSpinAngle = Max(universalMinSpinAngle, universalMaxSpinAngle);
                joint->SetSpinMinAngle(DegToRad(universalMinSpinAngle));
                joint->SetSpinMaxAngle(DegToRad(universalMaxSpinAngle));
            }

            if (ImGui::SliderFloat("Max spin", &universalMaxSpinAngle, 0.0f, 180.0f, "%.1f deg"))
            {
                universalMinSpinAngle = Min(universalMinSpinAngle, universalMaxSpinAngle);
                joint->SetSpinMinAngle(DegToRad(universalMinSpinAngle));
                joint->SetSpinMaxAngle(DegToRad(universalMaxSpinAngle));
            }

            if (ImGui::SliderFloat("Constraint frequency", &universalFrequency, -1.0f, 20.0f, "%.2f"))
            {
                game.RestartDemo();
            }

            if (ImGui::SliderFloat("Constraint damping", &universalDampingRatio, 0.0f, 3.0f, "%.2f"))
            {
                game.RestartDemo();
            }
        }
        ImGui::End();
    }

private:
    UniversalAngleJoint* joint = nullptr;
};

static Demo* CreateUniversalAngleJointDemo(Game& game)
{
    return new UniversalAngleJointDemo(game);
}

static int32 universal_angle_joint = register_demo("Joints", "Universal angle joint", CreateUniversalAngleJointDemo, 9);

} // namespace muli3
