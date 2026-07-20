#include "demo.h"
#include "muli3/random.h"
#include "ragdoll.h"

namespace muli3
{

static Vec3 SampleUniformHemisphere(Vec2 u)
{
    float z = u[0];
    float r = std::sqrt(std::fmax(0.0f, 1 - z * z));
    float phi = two_pi * u[1];

    return Vec3(r * std::cos(phi), z, r * std::sin(phi));
}

class RagdollTest : public Demo
{
public:
    RagdollTest(Game& game)
        : Demo(game)
    {
        Body* ground = world->CreateBox(50.0f, 0.2f, 50.0f, identity, Body::static_body);

        CreateRagdoll(world, Vec3{ 0, 4, 0 }, 1.0f, 1, 10.0f);

        camera.SetPosition(Vec3{ 0.0f, 5.0f, 8.0f });
        camera.SetRotation(-90.0f, -20.0f);

        // Srand(123);

        Body* c = world->CreateSphere(0.6f);
        Vec3 p = SampleUniformHemisphere(RandVec2());
        p *= 8.0f;

        c->SetLinearVelocity(-p * Rand(4.0f, 8.0f) + Vec3{ 0.0f, Rand(5.0f, 10.0f), 0.0f });
        p.y += 0.5f;
        c->SetPosition(p);
    }
};

static Demo* CreateRagdollTest(Game& game)
{
    return new RagdollTest(game);
}

static int32 single_box = register_demo("Ragdoll", "Ragdoll", CreateRagdollTest, 0);

} // namespace muli3
