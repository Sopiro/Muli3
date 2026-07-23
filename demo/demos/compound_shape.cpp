#include "demo.h"

namespace muli3
{

class CompoundShape : public Demo
{
public:
    CompoundShape(Game& game)
        : Demo(game)
    {
        world->CreateBox(24.0f, 0.5f, 24.0f, identity, Body::static_body);

        for (int32 i = 0; i < 10; ++i)
        {
            float y = 1.0f + i * 1.15f;

            Body* body = world->CreateEmptyBody(Transform{ Vec3{ 0.0f, y, 0.0f } });
            body->CreateBoxCollider(1.1f, 0.25f, 0.25f);
            body->CreateBoxCollider(0.25f, 1.1f, 0.25f);
            body->CreateBoxCollider(0.25f, 0.25f, 1.1f);
        }

        camera.SetPosition(Vec3{ 0.0f, 6.0f, 11.0f });
        camera.SetRotation(0.0f, -22.0f);
    }
};

static Demo* CreateCompoundShape(Game& game)
{
    return new CompoundShape(game);
}

static int32 compound_shape = register_demo("Shapes", "Compound shape", CreateCompoundShape, 2);

} // namespace muli3
