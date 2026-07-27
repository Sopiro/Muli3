#include "demo.h"
#include "muli3/random.h"

namespace muli3
{

class Logo : public Demo
{
public:
    Logo(Game& game)
        : Demo(game)
    {
        world->CreateBox(24.0f, 0.5f, 24.0f, identity, Body::static_body);

        Body* b = world->CreateEmptyBody();

        float offset = 0.5f;
        float radius = 0.1f;

        b->CreateCapsuleCollider(Vec3{ -4.0f + offset, 0.0f, 0.0f }, Vec3{ -4.0f + offset, 2.0f, 0.0f }, radius);
        b->CreateCapsuleCollider(Vec3{ -4.0f + offset, 2.0f, 0.0f }, Vec3{ -3.5f + offset, 1.0f, 0.0f }, radius);
        b->CreateCapsuleCollider(Vec3{ -3.5f + offset, 1.0f, 0.0f }, Vec3{ -3.0f + offset, 2.0f, 0.0f }, radius);
        b->CreateCapsuleCollider(Vec3{ -3.0f + offset, 2.0f, 0.0f }, Vec3{ -3.0f + offset, 0.0f, 0.0f }, radius);

        b->CreateCapsuleCollider(Vec3{ -2.0f + offset, 2.0f, 0.0f }, Vec3{ -2.0f + offset, 0.2f, 0.0f }, radius);
        b->CreateCapsuleCollider(Vec3{ -1.0f + offset, 0.2f, 0.0f }, Vec3{ -1.0f + offset, 2.0f, 0.0f }, radius);
        b->CreateCapsuleCollider(Vec3{ -2.0f + offset, 0.2f, 0.0f }, Vec3{ -1.8f + offset, 0.0f, 0.0f }, radius);
        b->CreateCapsuleCollider(Vec3{ -1.2f + offset, 0.0f, 0.0f }, Vec3{ -1.0f + offset, 0.2f, 0.0f }, radius);
        b->CreateCapsuleCollider(Vec3{ -1.8f + offset, 0.0f, 0.0f }, Vec3{ -1.2f + offset, 0.0f, 0.0f }, radius);

        b->CreateCapsuleCollider(Vec3{ 0.0f + offset, 0.0f, 0.0f }, Vec3{ 0.0f + offset, 2.0f, 0.0f }, radius);
        b->CreateCapsuleCollider(Vec3{ 0.0f + offset, 0.0f, 0.0f }, Vec3{ 1.0f + offset, 0.0f, 0.0f }, radius);

        b->CreateCapsuleCollider(Vec3{ 2.0f + offset, 2.0f, 0.0f }, Vec3{ 3.0f + offset, 2.0f, 0.0f }, radius);
        b->CreateCapsuleCollider(Vec3{ 2.0f + offset, 0.0f, 0.0f }, Vec3{ 3.0f + offset, 0.0f, 0.0f }, radius);
        b->CreateCapsuleCollider(Vec3{ 2.5f + offset, 0.0f, 0.0f }, Vec3{ 2.5f + offset, 2.0f, 0.0f }, radius);

        b->SetPosition(0.0f, 4.0f, 0.0f);

        Srand(123);

        for (int32 i = 0; i < 100; ++i)
        {
            float size = Rand(0.2f, 0.5f);
            // float r = Rand(default_radius, 0.06f);

            Vec3 pos = RandVec3(Vec3(-5, 0, 0), Vec3(5, 5, 0));
            pos.y += 30.0f;

            float r = Rand();
            if (r < 0.25)
            {
                b = world->CreateSphere(size / 2);
            }
            else if (r < 0.5)
            {
                b = world->CreateCapsule(size, size / 4);
            }
            else if (r < 0.75)
            {
                b = world->CreateBox(size);
            }
            else
            {
                b = world->CreateCylinder(size, size / 2, size / 2);
            }

            b->SetPosition(pos);
            b->SetRotation(Quat::FromEuler(RandVec3()));
        }

        camera.SetPosition(Vec3{ 0.0f, 1.0f, 6.0f });
        camera.SetRotation(0.0f, 0.0f);
    }
};

static Demo* CreateLogo(Game& game)
{
    return new Logo(game);
}

static int32 logo = register_demo("Shapes", "Logo", CreateLogo, 0);

} // namespace muli3
