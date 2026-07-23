#include "demo.h"

#include "muli3/noise.h"
#include "muli3/random.h"
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
        constexpr float frequency = 0.12f;
        constexpr float amplitude = 3.0f;

        std::vector<float> heights(sampleCount * sampleCount);
        for (int32 z = 0; z < sampleCount; ++z)
        {
            for (int32 x = 0; x < sampleCount; ++x)
            {
                float wx = x * cellSize * frequency;
                float wz = z * cellSize * frequency;
                heights[z * sampleCount + x] = Noise2(wx, wz, 2026) * amplitude;
            }
        }

        world->CreateHeightField(
            sampleCount, sampleCount, heights, cellSize, cellSize, identity, Vec3{ -halfExtent, 0.0f, -halfExtent }
        );

        camera.SetPosition(Vec3{ 0.0f, 12.0f, 20.0f });
        camera.SetRotation(0.0f, -35.0f);
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
        ImGui::SetNextWindowPos({ Window::Get()->GetWindowSize().x - 5.0f, 5.0f }, ImGuiCond_Always, { 1.0f, 0.0f });

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
        renderer.FlushAll();
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

        renderer.FlushAll(false);

        renderer.SetPointSize(prevPointSize);
        renderer.SetLineWidth(prevLineWidth);
    }

    std::vector<RayTrace> rayTraces;
};

static Demo* CreateHeightFieldRayCasting(Game& game)
{
    return new HeightFieldRayCasting(game);
}

static int32 height_field_ray_casting = register_demo("Raycast", "Height field ray casting", CreateHeightFieldRayCasting, 2);

static const char* shapeCastItems[] = { "Sphere", "Capsule", "Box", "Polygon", "Triangle", "Convex" };

class HeightFieldShapeCasting : public Demo
{
public:
    float maxDistance = 100.0f;
    Vec3 rot = Vec3::zero;
    bool rotate = false;
    bool showOverlay = false;
    int32 item = 1;

    HeightFieldShapeCasting(Game& game)
        : Demo(game)
    {
        constexpr int32 sampleCount = 128;
        constexpr float cellSize = 0.5f;
        constexpr float halfExtent = (sampleCount - 1) * cellSize * 0.5f;
        constexpr float frequency = 0.12f;
        constexpr float amplitude = 3.0f;

        std::vector<float> heights(sampleCount * sampleCount);
        for (int32 z = 0; z < sampleCount; ++z)
        {
            for (int32 x = 0; x < sampleCount; ++x)
            {
                float wx = x * cellSize * frequency;
                float wz = z * cellSize * frequency;
                heights[z * sampleCount + x] = Noise2(wx, wz, 2026) * amplitude;
            }
        }

        world->CreateHeightField(
            sampleCount, sampleCount, heights, cellSize, cellSize, identity, Vec3{ -halfExtent, 0.0f, -halfExtent }
        );

        camera.SetPosition(Vec3{ 0.0f, 12.0f, 20.0f });
        camera.SetRotation(0.0f, -35.0f);
        camera.speed = 0.65f;
    }

    void UpdateInput() override
    {
        FindTargetBody();
        EnableKeyboardShortcut();
        EnableCameraControl();
        EnableBodyCreate();

        if (rotate)
        {
            rotateTime += dt;
            rot.x = 45.0f * std::sin(rotateTime * 1.1f) + 18.0f * std::cos(rotateTime * 2.3f);
            rot.y = 80.0f * std::cos(rotateTime * 0.8f) + 35.0f * std::sin(rotateTime * 1.6f);
            rot.z = 55.0f * std::sin(rotateTime * 1.4f + 0.7f) + 20.0f * std::cos(rotateTime * 2.1f);
        }

        if (showOverlay)
        {
            UpdateShapeTraceOverlay();
        }
        RecordShapeTrace();
    }

    void UpdateUI() override
    {
        ImGui::SetNextWindowPos({ Window::Get()->GetWindowSize().x - 5.0f, 5.0f }, ImGuiCond_Always, { 1.0f, 0.0f });

        if (ImGui::Begin("Height field shape casting", NULL, ImGuiWindowFlags_AlwaysAutoResize | ImGuiWindowFlags_NoScrollbar))
        {
            ImGui::SetNextItemWidth(100.0f);
            ImGui::Combo("shape", &item, shapeCastItems, IM_ARRAYSIZE(shapeCastItems));
            ImGui::DragFloat("Cast distance", &maxDistance, 1.0f, 0.0f, 100.0f, "%.2f");
            ImGui::Checkbox("Rotate", &rotate);
            ImGui::Checkbox("Overlay", &showOverlay);
            ImGui::DragFloat3("Rot", &rot.x, 1.0f, -360.0f, 360.0f);
            if (ImGui::Button("Clear traces"))
            {
                shapeTraces.clear();
            }
        }
        ImGui::End();
    }

    void Render() override
    {
        renderer.FlushAll();

        float prevPointSize = renderer.GetPointSize();
        float prevLineWidth = renderer.GetLineWidth();
        renderer.SetPointSize(6.0f);
        renderer.SetLineWidth(1.5f);

        DrawShapeTraces();
        if (showOverlay)
        {
            DrawShapeTrace(overlayTrace, nextShape.get(), true);
        }

        renderer.FlushAll(false);

        renderer.SetPointSize(prevPointSize);
        renderer.SetLineWidth(prevLineWidth);
    }

private:
    struct ShapeTrace
    {
        std::unique_ptr<Shape> shape;
        Vec3 from;
        Vec3 to;
        Vec3 point;
        Vec3 normal;
        Quat q;
        int32 colorIndex;
        bool hit;
    };

    std::unique_ptr<Shape> CreateShape() const
    {
        switch (item)
        {
        case 0:
            return std::make_unique<SphereShape>(0.35f);
        case 1:
            return std::make_unique<CapsuleShape>(0.7f, 0.22f);
        case 2:
            return std::make_unique<BoxShape>(0.55f);
        case 3:
        {
            constexpr int32 segmentCount = 16;
            Vec3 vertices[segmentCount];
            for (int32 i = 0; i < segmentCount; ++i)
            {
                float angle = two_pi * i / segmentCount;
                vertices[i] = Vec3{ 0.4f * std::cos(angle), 0.4f * std::sin(angle), 0.0f };
            }
            return std::make_unique<PolygonShape>(vertices, default_radius);
        }
        case 4:
            return std::make_unique<TriangleShape>(
                Vec3{ -0.35f, -0.3f, 0.0f }, Vec3{ 0.35f, -0.3f, 0.0f }, Vec3{ 0.0f, 0.4f, 0.0f }, default_radius
            );
        case 5:
        {
            std::vector<Vec3> vertices;
            vertices.reserve(12);
            for (int32 i = 0; i < 12; ++i)
            {
                Vec3 dir = RandVec3(Vec3{ -1.0f }, Vec3{ 1.0f });
                if (Length2(dir) <= epsilon)
                {
                    --i;
                    continue;
                }

                dir.Normalize();
                Vec3 scale{ Rand(0.28f, 0.5f), Rand(0.24f, 0.45f), Rand(0.3f, 0.55f) };
                vertices.push_back(dir * scale);
            }
            return std::make_unique<ConvexShape>(vertices, default_radius);
        }
        default:
            return std::make_unique<SphereShape>(0.35f);
        }
    }

    void PrepareShape()
    {
        if (nextShape == nullptr || nextItem != item)
        {
            nextShape = CreateShape();
            nextItem = item;
        }
    }

    void BuildShapeTrace(ShapeTrace* trace, const Shape* shape, int32 colorIndex)
    {
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

        trace->from = ray.o;
        trace->to = ray.o + ray.d * maxDistance;
        trace->point = trace->to;
        trace->normal = Vec3::zero;
        trace->q = Quat::FromEuler({ DegToRad(rot.x), DegToRad(rot.y), DegToRad(rot.z) });
        trace->colorIndex = colorIndex;
        trace->hit = false;

        Transform tf{ trace->from, trace->q };
        Vec3 translation = trace->to - trace->from;
        world->ShapeCastClosest(shape, tf, translation, [&](Collider*, Vec3 point, Vec3 normal, float t) {
            trace->to = trace->from + translation * t;
            trace->point = point;
            trace->normal = normal;
            trace->hit = true;
        });
    }

    void UpdateShapeTraceOverlay()
    {
        PrepareShape();
        BuildShapeTrace(&overlayTrace, nextShape.get(), item);
    }

    void RecordShapeTrace()
    {
        if (ImGui::GetIO().WantCaptureMouse || Input::IsMousePressed(GLFW_MOUSE_BUTTON_LEFT) == false)
        {
            return;
        }

        ShapeTrace trace;
        PrepareShape();
        trace.shape = std::move(nextShape);
        BuildShapeTrace(&trace, trace.shape.get(), nextColorIndex++);
        shapeTraces.push_back(std::move(trace));

        nextShape = CreateShape();
        nextItem = item;
        if (showOverlay)
        {
            UpdateShapeTraceOverlay();
        }
    }

    void DrawShapeTrace(const ShapeTrace& trace, const Shape* shape, bool overlay)
    {
        Vec4 missColor{ 0.95f, 0.75f, 0.15f, 0.85f };
        Vec4 hitColor{ 0.0f, 0.0f, 1.0f, 1.0f };
        Vec4 lineColor{ 0.0f, 0.0f, 0.0f, 0.6f };
        Vec4 startColor{ 1.0f, 0.0f, 0.0f, 1.0f };
        Vec4 normalColor{ 0.05f, 0.25f, 1.0f, 0.9f };

        if (shape == nullptr)
        {
            return;
        }

        Vec4 endColor = trace.hit ? hitColor : missColor;
        Vec4 shapeColor = renderer.GetColor(trace.colorIndex);
        shapeColor.w = trace.hit ? 0.45f : 0.35f;
        if (overlay)
        {
            lineColor.w = 0.35f;
            startColor.w = 0.65f;
            endColor.w = 0.65f;
            shapeColor.w = 0.18f;
            normalColor.w = 0.65f;
        }

        renderer.DrawLine(trace.from, trace.to, lineColor);
        renderer.DrawPoint(trace.from, startColor);
        renderer.DrawPoint(trace.to, endColor);
        if (overlay)
        {
            renderer.DrawShape(shape, Transform{ trace.to, trace.q });
        }
        else
        {
            renderer.DrawShape(shape, Transform{ trace.to, trace.q }, shapeColor);
        }

        if (trace.hit)
        {
            renderer.DrawPoint(trace.point, hitColor);
            renderer.DrawLine(trace.point, trace.point + trace.normal * 0.45f, normalColor);
        }
    }

    void DrawShapeTraces()
    {
        for (const ShapeTrace& trace : shapeTraces)
        {
            DrawShapeTrace(trace, trace.shape.get(), false);
        }
    }

    std::vector<ShapeTrace> shapeTraces;
    ShapeTrace overlayTrace;
    std::unique_ptr<Shape> nextShape;
    int32 nextItem = -1;
    float rotateTime = 0.0f;
    int32 nextColorIndex = 0;
};

static Demo* CreateHeightFieldShapeCasting(Game& game)
{
    return new HeightFieldShapeCasting(game);
}

static int32 height_field_shape_casting =
    register_demo("Raycast", "Height field shape casting", CreateHeightFieldShapeCasting, 3);

} // namespace muli3
