#include "demo.h"

namespace muli3
{

class FrictionTest : public Demo
{
public:
    FrictionTest(Game& game)
        : Demo(game)
    {
        float groundFriction = 0.5f;

        Body* ground = world->CreateBox(50.0f, 0.5f, 50.0f, identity, Body::static_body);
        ground->SetFriction(groundFriction);

        Body* b = world->CreateBox(
            6.5f, 0.1f, 1.2f, Transform{ Vec3{ -0.6f, 5.0f, 0.0f }, Quat::FromEuler(Vec3{ 0.0f, 0.0f, -0.15f }) },
            Body::static_body
        );
        b->SetFriction(groundFriction);

        b = world->CreateBox(
            6.5f, 0.1f, 1.2f, Transform{ Vec3{ 0.0f, 3.0f, 0.0f }, Quat::FromEuler(Vec3{ 0.0f, 0.0f, 0.15f }) }, Body::static_body
        );
        b->SetFriction(groundFriction);

        b = world->CreateBox(
            6.5f, 0.1f, 1.2f, Transform{ Vec3{ -0.6f, 1.0f, 0.0f }, Quat::FromEuler(Vec3{ 0.0f, 0.0f, -0.15f }) },
            Body::static_body
        );
        b->SetFriction(groundFriction);

        b = world->CreateBox(0.1f, 1.1f, 1.2f, Transform{ Vec3{ 3.2f, 4.3f, 0.0f } }, Body::static_body);
        b = world->CreateBox(0.1f, 1.1f, 1.2f, Transform{ Vec3{ -3.8f, 2.3f, 0.0f } }, Body::static_body);

        float xStart = -4.5f;
        float yStart = 7.0f;
        float gap = 0.30f;
        float size = 0.30f;

        std::array<float, 5> frictions = { 0.4f, 0.2f, 0.12f, 0.04f, 0.0f };

        for (size_t i = 0; i < frictions.size(); ++i)
        {
            b = world->CreateBox(size, Transform{ Vec3{ xStart + (size + gap) * (float)i, yStart, 0.0f } }, Body::dynamic_body);
            b->SetFriction(frictions[i]);
            b->SetLinearVelocity(2.0f, 0.0f, 0.0f);
        }

        camera.SetPosition(Vec3{ 0.0f, 4.0f, 12.0f });
        camera.SetRotation(-90.0f, -12.0f);
    }
};

static Demo* CreateFrictionTest(Game& game)
{
    return new FrictionTest(game);
}

static int32 friction_test = register_demo("Dynamics", "Friction test", CreateFrictionTest, 0);

} // namespace muli3
