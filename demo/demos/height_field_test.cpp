#include "demo.h"

namespace muli3
{

class HeightFieldDemo : public Demo
{
public:
    HeightFieldDemo(Game& game)
        : Demo(game)
    {
        constexpr int32 sampleCount = 128;
        constexpr float cellSize = 0.5f;
        constexpr float halfExtent = (sampleCount - 1) * cellSize * 0.5f;

        std::vector<float> heights(sampleCount * sampleCount);
        for (int32 z = 0; z < sampleCount; ++z)
        {
            for (int32 x = 0; x < sampleCount; ++x)
            {
                float wx = (x - (sampleCount - 1) * 0.5f) * cellSize;
                float wz = (z - (sampleCount - 1) * 0.5f) * cellSize;
                float h = 0.55f * std::sin(wx * 0.55f) + 0.35f * std::cos(wz * 0.7f) + 0.18f * std::sin((wx + wz) * 0.9f);
                heights[z * sampleCount + x] = h;
            }
        }

        world->CreateHeightField(
            sampleCount, sampleCount, heights, cellSize, cellSize, identity, Vec3{ -halfExtent, 0.0f, -halfExtent }
        );

        for (int32 z = 0; z < 5; ++z)
        {
            for (int32 x = 0; x < 5; ++x)
            {
                Vec3 p{ -6.0f + x * 3.0f, 3.0f + z * 1.2f, -6.0f + z * 3.0f };
                Body* body = nullptr;
                int32 type = (x + z) % 3;
                if (type == 0)
                {
                    body = world->CreateBox(0.9f, 0.7f, 0.9f, Transform{ p });
                }
                else if (type == 1)
                {
                    body = world->CreateSphere(0.45f, Transform{ p });
                }
                else
                {
                    body = world->CreateCapsule(0.8f, 0.35f, Transform{ p });
                }
                body->SetRotation(Quat::FromEuler(Vec3{ 0.15f * x, 0.28f * z, 0.1f * (x + z) }));
                body->SetGyroscopicTorqueEnabled(true);
            }
        }

        camera.SetPosition(Vec3{ 0.0f, 9.5f, 18.0f });
        camera.SetRotation(-90.0f, -28.0f);
    }
};

static Demo* CreateHeightFieldDemo(Game& game)
{
    return new HeightFieldDemo(game);
}

static int32 height_field_shape = register_demo("Shapes", "Height field shape", CreateHeightFieldDemo, 3);

} // namespace muli3
