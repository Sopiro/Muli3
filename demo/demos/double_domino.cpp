#include "demo.h"
#include "game.h"
#include "window.h"

namespace muli3
{

static int32 rows = 15;

class DoubleDomino : public Demo
{
public:
    DoubleDomino(Game& game)
        : Demo(game)
    {
        float l = 0.25f + linear_slop;
        float boxWidth = 1.0f;
        float boxHeight = boxWidth * 4.0f;
        float boxDepth = 2.4f;
        float xGap = boxHeight - boxWidth * 0.95f;
        float xStart = -(rows - 1.0f) * (boxWidth + xGap) / 2.0f;
        float yStart = l + boxHeight / 2.0f;

        for (int32 x = 0; x < rows; ++x)
        {
            RigidBody* b = world->CreateBox(
                boxWidth, boxHeight, boxDepth, Transform{ Vec3{ xStart + x * (boxWidth + xGap), yStart, 0.0f } },
                RigidBody::dynamic_body
            );

            if (x == 0)
            {
                b->ApplyLinearImpulseLocal(
                    { boxWidth / 2.0f, boxHeight / 2.0f, 0.0f }, Vec3{ 1.0f, 0.0f, 0.0f } * b->GetMass(), true
                );
            }
        }

        float w = Max(15.0f, (float)rows) * (boxWidth + xGap) - xGap;
        world->CreateBox(w + 30, 0.5f, 20.0f, identity, RigidBody::static_body);

        camera.SetPosition(Vec3{ 0.0f, w / 5.0f, w * 0.75f });
        camera.SetRotation(-90.0f, -16.0f);
    }

    void UpdateUI() override
    {
        ImGui::SetNextWindowPos({ Window::Get()->GetWindowSize().x - 5.0f, 5.0f }, ImGuiCond_Once, { 1.0f, 0.0f });

        if (ImGui::Begin("Double domino", NULL, ImGuiWindowFlags_AlwaysAutoResize))
        {
            if (ImGui::SliderInt("Rows", &rows, 2, 30)) game.RestartDemo();
        }
        ImGui::End();
    }
};

static Demo* CreateDoubleDomino(Game& game)
{
    return new DoubleDomino(game);
}

static int32 double_domino = register_demo("Dynamics", "Double domino", CreateDoubleDomino, 2);

} // namespace muli3
