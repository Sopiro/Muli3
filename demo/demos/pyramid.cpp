#include "demo.h"
#include "game.h"
#include "muli3/hash.h"
#include "window.h"

namespace muli3
{

static int32 rows = 50;

class Pyramid : public Demo
{
    int s = 0;
    RigidBody* test[1000];

public:
    Pyramid(Game& game)
        : Demo(game)
    {
        float size = 1.0f;
        float gap = 0.03f;
        float xStep = size + gap;
        float yStep = size + gap;
        float xStart = -(rows - 1.0f) * xStep * 0.5f;
        float yStart = 0.25f + size * 0.5f;

        for (int32 y = 0; y < rows; ++y)
        {
            for (int32 x = 0; x < rows - y; ++x)
            {
                RigidBody* box = world->CreateBox(
                    size,
                    Transform{
                        Vec3{ xStart + y * xStep * 0.5f + x * xStep, yStart + y * yStep, 0.0f },
                    },
                    RigidBody::dynamic_body
                );
                if (s < 1000)
                {
                    test[s++] = box;
                }
            }
        }

        float h = Max(12.0f, (float)rows * yStep);
        world->CreateBox(h * 2, 0.5f, h * 2, identity, RigidBody::static_body);
        camera.SetPosition(Vec3{ 0.0f, h * 0.7f, h * 1.5f });
        camera.SetRotation(-90.0f, -18.0f);
    }

    std::string ts = "";
    void Render() override
    {
        if (world->GetStepIndex() < 2500)
        {
            for (int i = 0; i < s; ++i)
            {
                if (test[i] && test[i]->IsSleeping())
                {
                    std::cout << world->GetStepIndex() << ": " << test[i]->GetPosition().ToString() << std::endl;
                    ts += std::to_string(world->GetStepIndex()) + ": " + test[i]->GetPosition().ToString();
                    test[i] = nullptr;
                }
                if (test[i]) renderer.DrawPoint(test[i]->GetPosition(), { 1, 0, 1, 1 });
            }
        }

        if (world->GetStepIndex() == 2500)
        {
            auto hash = HashBuffer(ts.data(), ts.size());
            std::cout << hash << std::endl;
            if (hash == 3452869397254911038u)
            {
                std::cout << "Hash matched!" << std::endl;
            }
            else
            {
                std::cout << "Hash UN MATCHED" << std::endl;
            }
            exit(0);
        }
    }

    void UpdateUI() override
    {
        ImGui::SetNextWindowPos({ Window::Get()->GetWindowSize().x - 5.0f, 5.0f }, ImGuiCond_Once, { 1.0f, 0.0f });

        if (ImGui::Begin("Pyramid", NULL, ImGuiWindowFlags_AlwaysAutoResize))
        {
            if (ImGui::SliderInt("Rows", &rows, 1, 100)) game.RestartDemo();
        }
        ImGui::End();
    }
};

static Demo* CreatePyramid(Game& game)
{
    return new Pyramid(game);
}

static int32 pyramid = register_demo("Stacking", "Pyramid", CreatePyramid, 3);

} // namespace muli3
