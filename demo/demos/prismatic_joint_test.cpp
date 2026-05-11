#include "demo.h"

namespace muli3
{

class PrismaticJointTest : public Demo
{
public:
    PrismaticJointTest(Game& game)
        : Demo(game)
    {
        RigidBody* ground = world->CreateBox(24.0f, 0.5f, 24.0f, identity, RigidBody::static_body);

        RigidBody* body = world->CreateBox(0.5f, Transform{ Vec3{ 0.0f, 2.0f, 0.0f } });
        world->CreatePrismaticJoint(ground, body, body->GetPosition(), Vec3::zero);
        world->CreateLimitedDistanceJoint(ground, body, ground->GetPosition(), body->GetPosition(), 1.0f, 8.0f);

        body = world->CreateBox(0.5f, Transform{ Vec3{ 0.0f, 5.0f, 0.0f } });
        world->CreatePrismaticJoint(ground, body, body->GetPosition(), Vec3{ 1.0f, 0.0f, 0.0f });
        world->CreateLimitedDistanceJoint(ground, body, ground->GetPosition(), body->GetPosition(), -1.0f, 8.0f);

        camera.SetPosition(Vec3{ 0.0f, 5.0f, 10.0f });
        camera.SetRotation(-90.0f, -18.0f);
    }
};

static Demo* CreatePrismaticJointTest(Game& game)
{
    return new PrismaticJointTest(game);
}

static int32 prismatic_joint_test = register_demo("Joints", "Prismatic joint", CreatePrismaticJointTest, 3);

} // namespace muli3
