#include "demo.h"

namespace muli3
{

class CapsuleStacking : public Demo
{
public:
    CapsuleStacking(Game& game)
        : Demo(game)
    {
        world->CreateBox(28.0f, 0.5f, 18.0f, identity, Body::static_body);

        int32 verticalCount = 10;
        int32 horizontalCount = 10;

        float height = 1.0f;
        float radius = 0.28f;
        float gap = 0.05f;

        float groundTop = 0.3f;

        for (int32 i = 0; i < verticalCount; ++i)
        {
            float y = groundTop + height * 0.5f + radius + i * (height + radius * 2.0f + gap);
            float x = -4.0f;
            float z = 0.0f;

            Body* b = world->CreateCapsule(height, radius, Vec3{ x, y, z }, Body::dynamic_body);
            b->SetGyroscopicTorqueEnabled(true);
        }

        Quat horizontalX{ -pi * 0.5f, z_axis };

        for (int32 i = 0; i < horizontalCount; ++i)
        {
            float y = groundTop + radius + i * (radius * 2.0f + gap);
            float x = 4.0f;
            float z = 0.0f;

            Body* b = world->CreateCapsule(height, radius, Transform{ Vec3{ x, y, z }, horizontalX }, Body::dynamic_body);
            b->SetGyroscopicTorqueEnabled(true);
        }

        camera.SetPosition(Vec3{ 0.0f, 10.0f, 18.0f });
        camera.SetRotation(-90.0f, -10.0f);
    }
};

static Demo* CreateCapsuleStacking(Game& game)
{
    return new CapsuleStacking(game);
}

static int32 capsule_stacking = register_demo("Stacking", "Capsule stacking", CreateCapsuleStacking, 2);

} // namespace muli3
