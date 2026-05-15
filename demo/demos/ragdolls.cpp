#include "demo.h"
#include "muli3/random.h"
#include "ragdoll.h"

namespace muli3
{

class Ragdolls : public Demo
{
public:
    Ragdolls(Game& game)
        : Demo(game)
    {
        RigidBody* ground = world->CreateBox(50.0f, 0.2f, 50.0f, identity, RigidBody::static_body);

        CreateRagdoll(world, Vec3{ 0, 4, 0 }, 1.0f, 1, 10.0f);

        camera.SetPosition(Vec3{ 0.0f, 5.0f, 8.0f });
        camera.SetRotation(-90.0f, -20.0f);

        RigidBody* c = world->CreateSphere(0.6f);
        float r = Rand(0.0f, pi);
        Vec3 p{ std::cos(r), std::sin(r), 0 };
        p *= 8.0f;

        c->SetLinearVelocity(-p * Rand(4.0f, 8.0f) + Vec3{ 0.0f, Rand(5.0f, 15.0f), 0.0f });
        p.y += 0.5f;
        c->SetPosition(p);

        options.draw_joint = false;
    }
};

static Demo* CreateRagdoll(Game& game)
{
    return new Ragdolls(game);
}

static int32 single_box = register_demo("Ragdolls", "Ragdoll", CreateRagdoll, 0);

} // namespace muli3
