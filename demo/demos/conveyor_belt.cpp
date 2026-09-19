#include "demo.h"
#include "game.h"
#include "muli3/random.h"
#include "window.h"

namespace muli3
{

static float surfaceSpeed = 4.0f;
static bool spawn = true;

class ConveyorBelt : public Demo
{
    float r = 15.0f;
    float h = 4.0f;
    float y = 1.0f;

    Body* belt[4];

public:
    ConveyorBelt(Game& game)
        : Demo(game)
    {
        Body* ground = world->CreateBox(50.0f, 0.2f, 50.0f, identity, Body::static_body);

        Transform tf = Vec3{ -h / 2, y, -r / 2 };
        Transform tf2 = Vec3{ 0, y, -r / 2 - h / 2 };
        Transform tf3 = Vec3{ 0, y, -r / 2 + h / 2 };
        Transform rot = Quat::FromEuler(0.0f, -pi / 2, 0.0f);

        for (int32 i = 0; i < 4; ++i)
        {
            belt[i] = world->CreateBox(r, 0.5f, h, tf, Body::static_body);
            world->CreateBox(r + h, 2.0f + i * 1e-4f, 0.5f, tf2, Body::static_body);
            world->CreateBox(r - h, 1.0f + i * 1e-4f, 0.5f, tf3, Body::static_body);

            tf = rot * tf;
            tf2 = rot * tf2;
            tf3 = rot * tf3;
        }

        belt[0]->SetSurfaceSpeed({ surfaceSpeed, 0 });
        belt[1]->SetSurfaceSpeed({ 0, -surfaceSpeed });
        belt[2]->SetSurfaceSpeed({ -surfaceSpeed, 0 });
        belt[3]->SetSurfaceSpeed({ 0, surfaceSpeed });

        camera.SetPosition(Vec3{ 0.0f, 8.0f, 15.0f });
        camera.SetRotation(0.0f, -30.0f);
    }

    void UpdateInput() override
    {
        Demo::UpdateInput();

        static float t = 0;
        t += game.GetDeltaTime();

        float k = 0.5f;

        if (t > k)
        {
            if (spawn && !options.pause)
            {
                float f = Rand() * 10;
                Vec3 p = RandVec3({ -1, -1, -1 }, { 1, 1, 1 }) + Vec3{ -r / 2, 5, -r / 2 };

                if (f < 5)
                {
                    world->CreateBox(0.5f, p);
                }
                else if (f < 8)
                {
                    world->CreateSphere(0.5f / 2.0f, p);
                }
                else
                {
                    world->CreateCapsule(0.5f, 0.25f, p);
                }
            }

            t -= k;
        }
    }

    void UpdateUI() override
    {
        ImGui::SetNextWindowPos({ Window::Get()->GetWindowSize().x - 5.0f, 5.0f }, ImGuiCond_Always, { 1.0f, 0.0f });

        if (ImGui::Begin("ConveyorBelt", nullptr, ImGuiWindowFlags_AlwaysAutoResize))
        {
            if (ImGui::SliderFloat("Surface speed", &surfaceSpeed, 0.0f, 10.0f, "%.1f"))
            {
                belt[0]->SetSurfaceSpeed({ surfaceSpeed, 0 });
                belt[1]->SetSurfaceSpeed({ 0, -surfaceSpeed });
                belt[2]->SetSurfaceSpeed({ -surfaceSpeed, 0 });
                belt[3]->SetSurfaceSpeed({ 0, surfaceSpeed });

                world->Awake();
            }
            ImGui::Checkbox("Spawn", &spawn);
        }
        ImGui::End();
    }
};

static Demo* CreateConveyorBelt(Game& game)
{
    return new ConveyorBelt(game);
}

static int32 conveyor_belt = register_demo("Dynamics", "Conveyor belt", CreateConveyorBelt, 7);

} // namespace muli3
