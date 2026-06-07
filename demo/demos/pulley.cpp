#include "demo.h"
#include "game.h"

namespace muli3
{

class Pulley : public Demo
{
public:
    Pulley(Game& game)
        : Demo(game)
    {
        world->CreateBox(24.0f, 0.5f, 24.0f, identity, Body::static_body);

        groundAnchorA = Vec3{ -1.0f, 5.0f, 0.0f };
        groundAnchorB = Vec3{ 1.0f, 5.0f, 0.0f };
        localAnchorOffset = Vec3{ 0.0f, 0.25f, 0.0f };

        world->CreateBox(2.4f, 0.12f, 0.12f, Transform{ Vec3{ 0.0f, 5.0f, 0.0f } }, Body::static_body);

        bodyA = world->CreateBox(0.5f, Transform{ Vec3{ -1.0f, 3.0f, 0.0f } });
        bodyB = world->CreateBox(0.5f, Transform{ Vec3{ 1.0f, 3.0f, 0.0f } });

        float ratio = 1.0f;

        pulleyJoint = world->CreatePulleyJoint(
            bodyA, bodyB, bodyA->GetPosition() + localAnchorOffset, bodyB->GetPosition() + localAnchorOffset, groundAnchorA,
            groundAnchorB, ratio, -1.0f
        );

        camera.SetPosition(Vec3{ 0.0f, 4.2f, 8.0f });
        camera.SetRotation(-90.0f, -14.0f);
    }

    void Render() override
    {
        renderer.FlushAll();
    }

private:
    Body* bodyA = nullptr;
    Body* bodyB = nullptr;
    PulleyJoint* pulleyJoint = nullptr;
    Vec3 groundAnchorA = Vec3::zero;
    Vec3 groundAnchorB = Vec3::zero;
    Vec3 localAnchorOffset = Vec3::zero;
};

static Demo* CreatePulley(Game& game)
{
    return new Pulley(game);
}

static int32 pulley = register_demo("Joints", "Pulley", CreatePulley, 5);

} // namespace muli3
