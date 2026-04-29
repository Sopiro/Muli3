#include "demo.h"

namespace muli3
{

class SingleBox final : public Demo
{
public:
    SingleBox(Game& game)
        : Demo(game)
    {
        RigidBody* ground =
            world->CreateBox(100.0f, 0.2f, 100.0f, Transform{ Vec3{ 0.0f, -0.1f, 0.0f } }, RigidBody::static_body);

        RigidBody* box = world->CreateBox(0.5f, Transform{ Vec3{ 0.0f, 5.0f, 0.0f } });
        box->SetAngularVelocity(7.0f, -5.0f, 9.0f);

        camera.SetPosition(Vec3{ 0.0f, 3.0f, 8.0f });
        camera.SetRotation(-90.0f, -18.0f);
    }
};

static Demo* CreateSingleBox(Game& game)
{
    return new SingleBox(game);
}

static int32 single_box = register_demo("Single box", CreateSingleBox, 1);

} // namespace muli3
