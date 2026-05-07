#include "demo.h"

namespace muli3
{

class Logo : public Demo
{
public:
    Logo(Game& game)
        : Demo(game)
    {
        world->CreateBox(40.0f, 0.5f, 12.0f, identity, RigidBody::static_body);

        RigidBody* body = world->CreateEmptyBody();

        float offset = 0.5f;
        float radius = 0.1f;

        body->CreateCapsuleCollider(Vec3{ -4.0f + offset, 0.0f, 0.0f }, Vec3{ -4.0f + offset, 2.0f, 0.0f }, radius);
        body->CreateCapsuleCollider(Vec3{ -4.0f + offset, 2.0f, 0.0f }, Vec3{ -3.5f + offset, 1.0f, 0.0f }, radius);
        body->CreateCapsuleCollider(Vec3{ -3.5f + offset, 1.0f, 0.0f }, Vec3{ -3.0f + offset, 2.0f, 0.0f }, radius);
        body->CreateCapsuleCollider(Vec3{ -3.0f + offset, 2.0f, 0.0f }, Vec3{ -3.0f + offset, 0.0f, 0.0f }, radius);

        body->CreateCapsuleCollider(Vec3{ -2.0f + offset, 2.0f, 0.0f }, Vec3{ -2.0f + offset, 0.2f, 0.0f }, radius);
        body->CreateCapsuleCollider(Vec3{ -1.0f + offset, 0.2f, 0.0f }, Vec3{ -1.0f + offset, 2.0f, 0.0f }, radius);
        body->CreateCapsuleCollider(Vec3{ -2.0f + offset, 0.2f, 0.0f }, Vec3{ -1.8f + offset, 0.0f, 0.0f }, radius);
        body->CreateCapsuleCollider(Vec3{ -1.2f + offset, 0.0f, 0.0f }, Vec3{ -1.0f + offset, 0.2f, 0.0f }, radius);
        body->CreateCapsuleCollider(Vec3{ -1.8f + offset, 0.0f, 0.0f }, Vec3{ -1.2f + offset, 0.0f, 0.0f }, radius);

        body->CreateCapsuleCollider(Vec3{ 0.0f + offset, 0.0f, 0.0f }, Vec3{ 0.0f + offset, 2.0f, 0.0f }, radius);
        body->CreateCapsuleCollider(Vec3{ 0.0f + offset, 0.0f, 0.0f }, Vec3{ 1.0f + offset, 0.0f, 0.0f }, radius);

        body->CreateCapsuleCollider(Vec3{ 2.0f + offset, 2.0f, 0.0f }, Vec3{ 3.0f + offset, 2.0f, 0.0f }, radius);
        body->CreateCapsuleCollider(Vec3{ 2.0f + offset, 0.0f, 0.0f }, Vec3{ 3.0f + offset, 0.0f, 0.0f }, radius);
        body->CreateCapsuleCollider(Vec3{ 2.5f + offset, 0.0f, 0.0f }, Vec3{ 2.5f + offset, 2.0f, 0.0f }, radius);

        body->SetPosition(0.0f, 4.0f, 0.0f);

        // for (int32 y = 0; y < 8; ++y)
        // {
        //     for (int32 x = 0; x < 6; ++x)
        //     {
        //         float size = 0.16f + 0.02f * (x % 3);
        //         float px = -2.5f + x * 1.0f;
        //         float py = 8.0f + y * 0.9f;
        //         float pz = ((x + y) & 1) == 0 ? -0.35f : 0.35f;

        //         RigidBody* debris = world->CreateSphere(size, Transform{ Vec3{ px, py, pz } });
        //         debris->SetAngularVelocity(1.5f + 0.2f * y, 0.8f + 0.15f * x, -1.0f - 0.1f * (x + y));
        //     }
        // }

        camera.SetPosition(Vec3{ 0.0f, 1.0f, 6.0f });
        camera.SetRotation(-90.0f, 0.0f);
    }
};

static Demo* CreateLogo(Game& game)
{
    return new Logo(game);
}

static int32 logo = register_demo("Shapes", "Logo", CreateLogo, 11);

} // namespace muli3
