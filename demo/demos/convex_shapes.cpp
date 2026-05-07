#include "demo.h"

namespace muli3
{

class ConvexShapes : public Demo
{
public:
    ConvexShapes(Game& game)
        : Demo(game)
    {
        options.draw_outlined = false;

        world->CreateBox(28.0f, 0.5f, 28.0f, identity, RigidBody::static_body);

        const std::array<Vec3, 8> hullA = {
            Vec3{ -0.45f, -0.40f, -0.30f }, Vec3{ 0.42f, -0.35f, -0.26f }, Vec3{ 0.36f, -0.38f, 0.32f },
            Vec3{ -0.34f, -0.32f, 0.28f },  Vec3{ -0.18f, 0.52f, -0.20f }, Vec3{ 0.28f, 0.46f, -0.08f },
            Vec3{ 0.14f, 0.58f, 0.30f },    Vec3{ -0.30f, 0.40f, 0.18f },
        };

        const std::array<Vec3, 10> hullB = {
            Vec3{ -0.28f, -0.55f, -0.22f }, Vec3{ 0.30f, -0.48f, -0.18f }, Vec3{ 0.36f, -0.18f, 0.26f },
            Vec3{ -0.22f, -0.26f, 0.34f },  Vec3{ -0.34f, 0.16f, -0.30f }, Vec3{ 0.16f, 0.10f, -0.36f },
            Vec3{ 0.38f, 0.28f, 0.02f },    Vec3{ 0.06f, 0.55f, 0.22f },   Vec3{ -0.26f, 0.48f, 0.10f },
            Vec3{ -0.12f, 0.62f, -0.04f },
        };

        const std::array<Vec3, 12> hullC = {
            Vec3{ -0.50f, -0.20f, -0.12f }, Vec3{ -0.24f, -0.48f, -0.34f }, Vec3{ 0.18f, -0.50f, -0.28f },
            Vec3{ 0.44f, -0.18f, -0.10f },  Vec3{ 0.48f, 0.12f, 0.20f },    Vec3{ 0.14f, 0.34f, 0.42f },
            Vec3{ -0.18f, 0.28f, 0.38f },   Vec3{ -0.44f, 0.06f, 0.18f },   Vec3{ -0.08f, -0.04f, -0.46f },
            Vec3{ 0.08f, 0.46f, -0.18f },   Vec3{ -0.32f, 0.44f, -0.10f },  Vec3{ 0.28f, -0.02f, 0.46f },
        };

        for (int32 i = 0; i < 7; ++i)
        {
            float y = 1.0f + i * 1.35f;

            RigidBody* bodyA = world->CreateConvex(hullA, Transform{ Vec3{ -2.4f, y, 0.0f } });
            bodyA->SetRotation(Quat::FromEuler(Vec3{ 0.18f * i, 0.11f * i, 0.07f * i }));

            RigidBody* bodyB = world->CreateConvex(hullB, Transform{ Vec3{ 0.0f, y, 0.0f } });
            bodyB->SetRotation(Quat::FromEuler(Vec3{ 0.09f * i, 0.23f * i, 0.05f * i }));

            RigidBody* bodyC = world->CreateConvex(hullC, Transform{ Vec3{ 2.4f, y, 0.0f } });
            bodyC->SetRotation(Quat::FromEuler(Vec3{ 0.14f * i, 0.08f * i, 0.19f * i }));
        }

        camera.SetPosition(Vec3{ 0.0f, 6.0f, 12.5f });
        camera.SetRotation(-90.0f, -20.0f);
    }
};

static Demo* CreateConvexShapes(Game& game)
{
    return new ConvexShapes(game);
}

static int32 convex_shapes = register_demo("Convex shape", CreateConvexShapes, 11);

} // namespace muli3
