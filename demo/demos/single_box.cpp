#include "demo.h"

namespace muli3
{

class SingleBox : public Demo
{
public:
    SingleBox(Game& game)
        : Demo(game)
    {
        RigidBody* ground = world->CreateBox(50.0f, 0.2f, 50.0f, identity, RigidBody::static_body);

        RigidBody* box = world->CreateBox(0.5f, Transform{ Vec3{ 0.0f, 5.0f, 0.0f } });
        box->SetAngularVelocity(7.0f, -5.0f, 9.0f);

        camera.SetPosition(Vec3{ 0.0f, 5.0f, 8.0f });
        camera.SetRotation(-90.0f, -20.0f);
    }
};

static Demo* CreateSingleBox(Game& game)
{
    return new SingleBox(game);
}

static int32 single_box = register_demo("Basics", "Single box", CreateSingleBox, 0);

} // namespace muli3
