#include "demo.h"

namespace muli3
{

class SphereStacking final : public Demo
{
public:
    SphereStacking(Game& game)
        : Demo(game)
    {
        RigidBody* ground = world->CreateSphere(100.0f, Transform{ Vec3{ 0.0f, -100.0f, 0.0f } }, true);
        ground->restitution = 0.15f;
        ground->friction = 0.9f;
        ground->visible = true;

        constexpr int width = 4;
        constexpr int height = 4;
        constexpr int depth = 4;

        for (int y = 0; y < height; ++y)
        {
            for (int z = 0; z < depth; ++z)
            {
                for (int x = 0; x < width; ++x)
                {
                    RigidBody* body = world->CreateSphere(
                        0.5f,
                        Transform{
                            Vec3{
                                ((float)x - 1.5f) * 1.25f,
                                1.5f + (float)y * 1.2f,
                                ((float)z - 1.5f) * 1.25f,
                            },
                        },
                        false, 1.0f
                    );

                    body->restitution = 0.3f;
                    body->friction = 0.55f;
                }
            }
        }
    }
};

static Demo* CreateSphereStacking(Game& game)
{
    return new SphereStacking(game);
}

static int32 sphere_stacking = register_demo("Sphere stacking", CreateSphereStacking, 10);

} // namespace muli3
