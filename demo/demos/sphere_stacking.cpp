#include "demo.h"

namespace muli3
{

class SphereStacking : public Demo
{
public:
    SphereStacking(Game& game)
        : Demo(game)
    {
        world->CreateBox(24.0f, 0.5f, 24.0f, identity, Body::static_body);

        constexpr int size = 10;
        constexpr float radius = 0.5f;
        constexpr float xzStep = 1.25f;
        constexpr float yStep = 1.2f;
        float xStart = -(size - 1.0f) * xzStep * 0.5f;
        float zStart = -(size - 1.0f) * xzStep * 0.5f;
        float yStart = 1.5f;

        for (int x = 0; x < size; ++x)
        {
            for (int y = 0; y < x; ++y)
            {
                for (int z = 0; z < y; ++z)
                {
                    world->CreateSphere(
                        radius,
                        Transform{
                            Vec3{
                                xStart + (float)x * xzStep,
                                yStart + (float)y * yStep,
                                zStart + (float)z * xzStep,
                            },
                        },
                        Body::dynamic_body
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

static int32 sphere_stacking = register_demo("Stacking", "Sphere stacking", CreateSphereStacking, 1);

} // namespace muli3
