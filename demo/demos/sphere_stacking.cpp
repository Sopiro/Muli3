#include "demo.h"

namespace muli3
{

class SphereStacking : public Demo
{
public:
    SphereStacking(Game& game)
        : Demo(game)
    {
        RigidBody* ground = world->CreateBox(24.0f, 0.5f, 24.0f, Transform{ Vec3{ 0.0f, -0.25f, 0.0f } }, RigidBody::static_body);

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
                        RigidBody::dynamic_body
                    );
                }
            }
        }
    }
};

static Demo* CreateSphereStacking(Game& game)
{
    return new SphereStacking(game);
}

static int32 sphere_stacking = register_demo("Sphere stacking", CreateSphereStacking, 4);

} // namespace muli3
