#include "demo.h"

#include "renderer.h"
#include "window.h"

#include <memory>

namespace muli3
{

static const char* distanceShapeItems[] = { "Sphere", "Capsule", "Box" };

class ComputeDistanceShape : public Demo
{
public:
    ComputeDistanceShape(Game& game)
        : Demo(game)
    {
        settings.apply_gravity = false;
        camera.SetPosition(Vec3{ 0.0f, 1.1f, 2.6f });
        camera.SetRotation(-90.0f, 0.0f);
        camera.speed = 0.2f;
        Reset();
        Step();
    }

    void UpdateInput() override
    {
        EnableKeyboardShortcut();
        EnableCameraControl();
    }

    void UpdateUI() override
    {
        ImGui::SetNextWindowPos({ Window::Get()->GetWindowSize().x - 5.0f, 5.0f }, ImGuiCond_Always, { 1.0f, 0.0f });
        if (ImGui::Begin("Distance between shapes", NULL, ImGuiWindowFlags_AlwaysAutoResize | ImGuiWindowFlags_NoScrollbar))
        {
            bool changed = false;

            ImGui::SetNextItemWidth(100.0f);
            if (ImGui::Combo("shape1", &item1, distanceShapeItems, IM_ARRAYSIZE(distanceShapeItems)))
            {
                UpdateShape1();
                changed = true;
            }

            ImGui::SameLine();
            ImGui::SetNextItemWidth(100.0f);
            if (ImGui::Combo("shape2", &item2, distanceShapeItems, IM_ARRAYSIZE(distanceShapeItems)))
            {
                UpdateShape2();
                changed = true;
            }

            ImGui::SetNextItemWidth(120.0f);
            if (ImGui::DragFloat3("pos1", &tf1.p.x, 0.01f))
            {
                changed = true;
            }

            ImGui::SameLine();
            ImGui::SetNextItemWidth(120.0f);
            if (ImGui::DragFloat3("pos2", &tf2.p.x, 0.01f))
            {
                changed = true;
            }

            ImGui::SetNextItemWidth(120.0f);
            if (ImGui::DragFloat3("rot1", &rot1.x, 0.5f))
            {
                tf1.q = Quat::FromEuler(Vec3{ DegToRad(rot1.x), DegToRad(rot1.y), DegToRad(rot1.z) });
                changed = true;
            }

            ImGui::SameLine();
            ImGui::SetNextItemWidth(120.0f);
            if (ImGui::DragFloat3("rot2", &rot2.x, 0.5f))
            {
                tf2.q = Quat::FromEuler(Vec3{ DegToRad(rot2.x), DegToRad(rot2.y), DegToRad(rot2.z) });
                changed = true;
            }

            ImGui::SetNextItemWidth(120.0f);
            if (ImGui::DragFloat3("size1", &size1.x, 0.01f, 0.05f, 5.0f))
            {
                UpdateShape1();
                changed = true;
            }

            ImGui::SameLine();
            ImGui::SetNextItemWidth(120.0f);
            if (ImGui::DragFloat3("size2", &size2.x, 0.01f, 0.05f, 5.0f))
            {
                UpdateShape2();
                changed = true;
            }

            ImGui::SetNextItemWidth(120.0f);
            if (ImGui::DragFloat("convex radius1", &convexRadius1, 0.001f, 0.0f, 0.2f))
            {
                UpdateShape1();
                changed = true;
            }

            ImGui::SameLine();
            ImGui::SetNextItemWidth(120.0f);
            if (ImGui::DragFloat("convex radius2", &convexRadius2, 0.001f, 0.0f, 0.2f))
            {
                UpdateShape2();
                changed = true;
            }

            if (ImGui::Button("Reset"))
            {
                Reset();
                changed = true;
            }

            ImGui::Separator();
            if (distance > 0.0f)
            {
                ImGui::Text("Distance: %.5f", distance);
                ImGui::Text("Point A: %.3f, %.3f, %.3f", pointA.x, pointA.y, pointA.z);
                ImGui::Text("Point B: %.3f, %.3f, %.3f", pointB.x, pointB.y, pointB.z);
            }
            else
            {
                ImGui::Text("%s", "Collide");
            }

            if (changed)
            {
                Step();
            }
        }
        ImGui::End();
    }

    void Step() override
    {
        distance = ComputeDistance(shape1.get(), tf1, shape2.get(), tf2, &pointA, &pointB);
    }

    void Render() override
    {
        float prevSize = renderer.GetPointSize();
        renderer.SetPointSize(7.0f);
        renderer.DrawShape(shape1.get(), tf1);
        renderer.DrawShape(shape2.get(), tf2);

        if (distance > 0.0f)
        {
            const Vec4 pointAColor{ 1.0f, 0.18f, 0.08f, 1.0f };
            const Vec4 pointBColor{ 0.05f, 0.25f, 1.0f, 1.0f };
            const Vec4 lineColor{ 0.08f, 0.09f, 0.10f, 1.0f };

            renderer.DrawPoint(pointA, pointAColor);
            renderer.DrawPoint(pointB, pointBColor);
            renderer.DrawLine(pointA, pointB, lineColor);
        }

        renderer.FlushAll();
        renderer.SetPointSize(prevSize);
    }

private:
    void Reset()
    {
        tf1 = Transform{ Vec3{ -0.65f, 1.0f, 0.0f } };
        tf2 = Transform{ Vec3{ 0.65f, 1.0f, 0.0f } };
        rot1 = Vec3::zero;
        rot2 = Vec3{ 15.0f, 30.0f, 0.0f };
        tf1.q = Quat::FromEuler(Vec3{ DegToRad(rot1.x), DegToRad(rot1.y), DegToRad(rot1.z) });
        tf2.q = Quat::FromEuler(Vec3{ DegToRad(rot2.x), DegToRad(rot2.y), DegToRad(rot2.z) });
        size1 = Vec3{ 0.55f, 0.55f, 0.55f };
        size2 = Vec3{ 0.7f, 0.5f, 0.6f };
        convexRadius1 = default_radius;
        convexRadius2 = default_radius;
        item1 = 1;
        item2 = 1;
        UpdateShape1();
        UpdateShape2();
    }

    void UpdateShape1()
    {
        switch (item1)
        {
        case 0:
            shape1.reset(new Sphere(size1.x));
            break;
        case 1:
            shape1.reset(new Capsule(size1.x, size1.y / 2));
            break;
        case 2:
            shape1.reset(new Box(size1, convexRadius1));
            break;
        default:
            break;
        }
    }

    void UpdateShape2()
    {
        switch (item2)
        {
        case 0:
            shape2.reset(new Sphere(size2.x));
            break;
        case 1:
            shape2.reset(new Capsule(size2.x, size2.y / 2));
            break;
        case 2:
            shape2.reset(new Box(size2, convexRadius2));
            break;
        default:
            break;
        }
    }

    int32 item1 = 1;
    int32 item2 = 1;

    Transform tf1;
    Transform tf2;
    std::unique_ptr<Shape> shape1;
    std::unique_ptr<Shape> shape2;
    Vec3 rot1;
    Vec3 rot2;
    Vec3 size1;
    Vec3 size2;
    float convexRadius1 = default_radius;
    float convexRadius2 = default_radius;
    Vec3 pointA = Vec3::zero;
    Vec3 pointB = Vec3::zero;
    float distance = 0.0f;
};

static Demo* CreateComputeDistanceShape(Game& game)
{
    return new ComputeDistanceShape(game);
}

static int32 compute_distance_shape = register_demo("Distance between shapes", CreateComputeDistanceShape, 8);

} // namespace muli3
