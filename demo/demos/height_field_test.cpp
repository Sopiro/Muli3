#include "demo.h"
#include "game.h"
#include "muli3/noise.h"
#include "muli3/random.h"
#include "window.h"

namespace muli3
{

static int32 sampleCount = 128;
static float cellSize = 0.5f;
static int32 noiseType = 1;
static float frequency = 0.12f;
static float amplitude = 3.0f;
static int32 octaves = 5;
static float lacunarity = 2.0f;
static float gain = 0.5f;
static int32 seed = 0;
static float ridgeOffset = 1.0f;

static void BuildHeightSamples(std::vector<float>* heights)
{
    heights->resize(sampleCount * sampleCount);

    for (int32 z = 0; z < sampleCount; ++z)
    {
        for (int32 x = 0; x < sampleCount; ++x)
        {
            float wx = x * cellSize * frequency;
            float wz = z * cellSize * frequency;
            float h = 0.0f;

            if (noiseType == 0)
            {
                h = Noise2(wx, wz, (uint32)seed);
            }
            else if (noiseType == 1)
            {
                h = FractalNoise2(wx, wz, octaves, lacunarity, gain, (uint32)seed);
            }
            else if (noiseType == 2)
            {
                h = TurbulenceNoise2(wx, wz, octaves, lacunarity, gain, (uint32)seed);
            }
            else
            {
                h = RidgedNoise2(wx, wz, octaves, lacunarity, gain, ridgeOffset, (uint32)seed);
            }

            (*heights)[z * sampleCount + x] = h * amplitude;
        }
    }
}

class HeightFieldDemo : public Demo
{
public:
    HeightFieldDemo(Game& game)
        : Demo(game)
    {
        float halfExtent = (sampleCount - 1) * cellSize * 0.5f;

        std::vector<float> heights;
        BuildHeightSamples(&heights);

        world->CreateHeightField(
            sampleCount, sampleCount, heights, cellSize, cellSize, identity, Vec3{ -halfExtent, 0.0f, -halfExtent }
        );

        for (int32 z = 0; z < 5; ++z)
        {
            for (int32 x = 0; x < 5; ++x)
            {
                Vec3 p{ -6.0f + x * 3.0f, 5.0f + (5 - z) * 1.2f, -6.0f + z * 3.0f };
                Body* body = nullptr;
                int32 type = (x + z) % 3;
                if (type == 0)
                {
                    body = world->CreateBox(1.0f, Transform{ p });
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

    void UpdateUI() override
    {
        ImGui::SetNextWindowPos({ Window::Get()->GetWindowSize().x - 5.0f, 5.0f }, ImGuiCond_Always, { 1.0f, 0.0f });

        if (ImGui::Begin("Height field shape", NULL, ImGuiWindowFlags_AlwaysAutoResize))
        {
            bool rebuild = false;
            const char* noiseItems[] = { "Noise", "Fractal", "Turbulence", "Ridged" };

            rebuild |= ImGui::SliderInt("Samples", &sampleCount, 16, 1024);
            rebuild |= ImGui::SliderFloat("Cell size", &cellSize, 0.1f, 2.0f, "%.2f");
            rebuild |= ImGui::Combo("Noise", &noiseType, noiseItems, IM_ARRAYSIZE(noiseItems));
            rebuild |= ImGui::DragFloat("Frequency", &frequency, 0.005f, 0.01f, 1.0f, "%.3f");
            rebuild |= ImGui::DragFloat("Amplitude", &amplitude, 0.05f, 0.0f, 10.0f, "%.2f");
            rebuild |= ImGui::SliderInt("Octaves", &octaves, 1, 8);
            rebuild |= ImGui::SliderFloat("Lacunarity", &lacunarity, 1.0f, 4.0f, "%.2f");
            rebuild |= ImGui::SliderFloat("Gain", &gain, 0.0f, 1.0f, "%.2f");
            rebuild |= ImGui::SliderInt("Seed", &seed, 0, 1024);
            if (ImGui::Button("Random Generate"))
            {
                frequency = Rand(0.04f, 0.25f);
                amplitude = Rand(1.5f, 6.0f);
                octaves = 1 + (int32)g_rng.NextUint(8);
                lacunarity = Rand(1.6f, 3.2f);
                gain = Rand(0.35f, 0.75f);
                seed = (int32)g_rng.NextUint(1025);
                ridgeOffset = Rand(0.75f, 1.35f);

                rebuild = true;
            }

            if (noiseType == 3)
            {
                rebuild |= ImGui::SliderFloat("Ridge offset", &ridgeOffset, 0.1f, 2.0f, "%.2f");
            }

            if (rebuild)
            {
                game.RestartDemo();
            }
        }

        ImGui::End();
    }
};

static Demo* CreateHeightFieldDemo(Game& game)
{
    return new HeightFieldDemo(game);
}

static int32 height_field_shape = register_demo("Shapes", "Height field shape", CreateHeightFieldDemo, 3);

} // namespace muli3
