#include "demo.h"

#include "renderer.h"
#include "window.h"

namespace muli3
{

class HeightFieldRayCasting : public Demo
{
public:
    float maxDistance = 100.0f;

    HeightFieldRayCasting(Game& game)
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

        camera.SetPosition(Vec3{ 0.0f, 12.0f, 20.0f });
        camera.SetRotation(-90.0f, -35.0f);
        camera.speed = 0.65f;
    }

    void UpdateInput() override
    {
        FindTargetBody();
        EnableKeyboardShortcut();
        EnableCameraControl();
        EnableBodyCreate();

        RecordRayTrace();
    }

    void UpdateUI() override
    {
        ImGui::SetNextWindowPos({ Window::Get()->GetWindowSize().x - 5.0f, 5.0f }, ImGuiCond_Once, { 1.0f, 0.0f });

        if (ImGui::Begin("Height field ray casting", NULL, ImGuiWindowFlags_AlwaysAutoResize | ImGuiWindowFlags_NoScrollbar))
        {
            ImGui::DragFloat("Ray distance", &maxDistance, 1.0f, 0.0f, 100.0f, "%.2f");
            if (ImGui::Button("Clear traces"))
            {
                rayTraces.clear();
            }
        }
        ImGui::End();
    }

    void Render() override
    {
        DrawRayTraces();
    }

private:
    struct RayTrace
    {
        Vec3 from;
        Vec3 to;
        Vec3 normal;
        bool hit;
    };

    void RecordRayTrace()
    {
        if (ImGui::GetIO().WantCaptureMouse || Input::IsMousePressed(GLFW_MOUSE_BUTTON_LEFT) == false)
        {
            return;
        }

        Ray ray;
        if (Window::Get()->GetCursorHidden())
        {
            ray = Ray{ camera.GetPosition(), camera.GetForward() };
        }
        else
        {
            ray = GetMouseRay();
            ray.o = camera.GetPosition();
        }

        RayTrace trace;
        trace.from = ray.o;
        trace.to = ray.o + ray.d * maxDistance;
        trace.normal = Vec3::zero;
        trace.hit = false;

        world->RayCastClosest(trace.from, trace.to, [&](Collider*, Vec3 point, Vec3 normal, float) {
            trace.to = point;
            trace.normal = normal;
            trace.hit = true;
        });

        rayTraces.push_back(trace);
    }

    void DrawRayTraces()
    {
        Vec4 missColor{ 0.95f, 0.75f, 0.15f, 0.75f };
        Vec4 hitColor{ 0.0f, 0.0f, 1.0f, 1.0f };
        Vec4 lineColor{ 0.0f, 0.0f, 0.0f, 0.6f };
        Vec4 startColor{ 1.0f, 0.0f, 0.0f, 1.0f };
        Vec4 normalColor{ 0.05f, 0.25f, 1.0f, 0.85f };

        float prevPointSize = renderer.GetPointSize();
        float prevLineWidth = renderer.GetLineWidth();
        renderer.SetPointSize(6.0f);
        renderer.SetLineWidth(1.5f);

        for (const RayTrace& trace : rayTraces)
        {
            Vec4 endColor = trace.hit ? hitColor : missColor;
            renderer.DrawLine(trace.from, trace.to, lineColor);
            renderer.DrawPoint(trace.from, startColor);
            renderer.DrawPoint(trace.to, endColor);

            if (trace.hit)
            {
                renderer.DrawLine(trace.to, trace.to + trace.normal * 0.35f, normalColor);
            }
        }

        renderer.FlushAll();

        renderer.SetPointSize(prevPointSize);
        renderer.SetLineWidth(prevLineWidth);
    }

    std::vector<RayTrace> rayTraces;
};

static Demo* CreateHeightFieldRayCasting(Game& game)
{
    return new HeightFieldRayCasting(game);
}

static int32 height_field_ray_casting = register_demo("Collision", "Height field ray casting", CreateHeightFieldRayCasting, 4);

} // namespace muli3
