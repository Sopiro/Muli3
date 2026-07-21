#include "demo.h"
#include "window.h"

#include <muli3/random.h>

namespace muli3
{

static float speed = 90.0f;

class KinematicBody : public Demo
{
public:
    KinematicBody(Game& game)
        : Demo(game)
    {
        float size = 15.0f;
        float halfSize = size * 0.5f;
        float wallWidth = 0.4f;
        float wallHeight = 1.0f;
        float groundHeight = 0.4f;

        world->CreateBox(size, groundHeight, size, Transform{ Vec3{ 0.0f, -groundHeight * 0.5f, 0.0f } }, Body::static_body);
        world->CreateBox(wallWidth, wallHeight, size, Transform{ Vec3{ -halfSize, wallHeight * 0.5f, 0.0f } }, Body::static_body);
        world->CreateBox(wallWidth, wallHeight, size, Transform{ Vec3{ halfSize, wallHeight * 0.5f, 0.0f } }, Body::static_body);
        world->CreateBox(size, wallHeight, wallWidth, Transform{ Vec3{ 0.0f, wallHeight * 0.5f, -halfSize } }, Body::static_body);
        world->CreateBox(size, wallHeight, wallWidth, Transform{ Vec3{ 0.0f, wallHeight * 0.5f, halfSize } }, Body::static_body);

        float r = 0.22f;
        float range = size - wallWidth * 3.0f;

        for (int32 i = 0; i < 500; ++i)
        {
            float x = Rand(0.0f, range) - range * 0.5f;
            float z = Rand(0.0f, range) - range * 0.5f;

            Body* b = world->CreateSphere(r, Transform{ Vec3{ x, r, z } }, Body::dynamic_body);
            b->SetRotation(Quat::FromEuler(RandVec3(Vec3{ 0.0f, 0.0f, 0.0f }, Vec3{ two_pi, two_pi, two_pi })));
        }

        k = world->CreateEmptyBody(Transform{ Vec3{ 0.0f, 0.25f, 0.0f } }, Body::kinematic_body);
        k->CreateCapsuleCollider(Vec3{ -size * 0.45f, 0.0f, 0.0f }, Vec3{ size * 0.45f, 0.0f, 0.0f }, 0.15f);
        k->CreateCapsuleCollider(Vec3{ 0.0f, 0.0f, -size * 0.45f }, Vec3{ 0.0f, 0.0f, size * 0.45f }, 0.15f);
        k->SetAngularVelocity(0.0f, DegToRad(speed), 0.0f);

        camera.SetPosition(Vec3{ 0.0f, 13.0f, 17.0f });
        camera.SetRotation(-90.0f, -34.0f);
    }

    void UpdateUI() override
    {
        ImGui::SetNextWindowPos({ Window::Get()->GetWindowSize().x - 5.0f, 5.0f }, ImGuiCond_Always, { 1.0f, 0.0f });

        if (ImGui::Begin("Kinematic body", nullptr, ImGuiWindowFlags_AlwaysAutoResize))
        {
            if (ImGui::SliderFloat("Speed", &speed, 0.0f, 360.0f, "%.2f deg/s"))
            {
                k->Awake();
                k->SetAngularVelocity(0.0f, DegToRad(speed), 0.0f);
            }
        }
        ImGui::End();
    }

private:
    Body* k = nullptr;
};

static Demo* CreateKinematicBody(Game& game)
{
    return new KinematicBody(game);
}

static int32 kinematic_body = register_demo("Dynamics", "Kinematic body", CreateKinematicBody, 4);

} // namespace muli3
