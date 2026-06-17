#include "demo.h"
#include "game.h"
#include "window.h"

namespace muli3
{

static float jointFrequency = 30.0f;
static float dampingRatio = 1.0f;

class FixedRotation : public Demo
{
public:
    FixedRotation(Game& game)
        : Demo(game)
    {
        options.draw_outline = false;

        world->CreateBox(24.0f, 0.5f, 24.0f, identity, Body::static_body);

        float start = 1.2f;
        float size = 0.6f;
        float gap = 0.08f;
        float error = 0.3f;

        float px = 0.0f;
        int32 d = 1;

        int32 count = 15;
        for (int32 i = 0; i < count; ++i)
        {
            if (i % (count / 4) == 0)
            {
                d = -d;
            }

            px += d * error;

            Body* body = world->CreateBox(size, Transform{ Vec3{ px, start + i * (size + gap), 0.0f } });
            world->CreateFixedRotationJoint(body, jointFrequency, dampingRatio);
        }

        camera.SetPosition(Vec3{ 0.0f, 6.2f, 14.0f });
        camera.SetRotation(-90.0f, -9.0f);
    }

    void UpdateInput() override
    {
        Demo::UpdateInput();

        if (ImGui::GetIO().WantCaptureKeyboard || Window::Get()->GetCursorHidden())
        {
            return;
        }

        if (targetBody == nullptr || targetBody->GetType() != Body::dynamic_body)
        {
            return;
        }

        if (Input::IsKeyPressed(GLFW_KEY_T))
        {
            FixedRotationJoint* joint = FindFixedRotationJoint(targetBody);
            if (joint)
            {
                world->Destroy(joint);
            }
            else
            {
                world->CreateFixedRotationJoint(targetBody, jointFrequency, dampingRatio);
            }
        }
    }

    void UpdateUI() override
    {
        ImGui::SetNextWindowPos({ Window::Get()->GetWindowSize().x - 5.0f, 5.0f }, ImGuiCond_Once, { 1.0f, 0.0f });

        if (ImGui::Begin("Fixed rotation", nullptr, ImGuiWindowFlags_AlwaysAutoResize))
        {
            if (ImGui::SliderFloat("Frequency", &jointFrequency, -1.0f, 20.0f, "%.2f"))
            {
                game.RestartDemo();
            }

            if (ImGui::SliderFloat("Damping ratio", &dampingRatio, 0.0f, 1.0f, "%.2f"))
            {
                game.RestartDemo();
            }
        }
        ImGui::End();

        ImGui::SetNextWindowPos(
            { Window::Get()->GetWindowSize().x - 5.0f, Window::Get()->GetWindowSize().y - 5.0f }, ImGuiCond_Always, { 1.0f, 1.0f }
        );
        ImGui::Begin(
            "FixedRotationsHelp", nullptr,
            ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoInputs | ImGuiWindowFlags_AlwaysAutoResize |
                ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_NoBackground
        );
        ImGui::TextColored(ImColor{ 12, 11, 14 }, "Press T on body to toggle fixed");
        ImGui::End();
    }

private:
    FixedRotationJoint* FindFixedRotationJoint(Body* body) const
    {
        for (Joint* joint = world->GetJoints(); joint; joint = joint->GetNext())
        {
            if (joint->GetType() == Joint::fixed_rotation_joint && joint->GetBodyA() == body)
            {
                return (FixedRotationJoint*)joint;
            }
        }

        return nullptr;
    }
};

static Demo* CreateFixedRotation(Game& game)
{
    return new FixedRotation(game);
}

static int32 fixed_rotations = register_demo("Joints", "Fixed rotation", CreateFixedRotation, 2);

} // namespace muli3
