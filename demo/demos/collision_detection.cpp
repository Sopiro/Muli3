#include "demo.h"

#include "renderer.h"
#include "window.h"

namespace muli3
{

static const char* shapeItems[] = { "Sphere", "Box" };

class CollisionDetection : public Demo
{
public:
    CollisionDetection(Game& game)
        : Demo(game)
    {
        settings.apply_gravity = false;
        camera.SetPosition(Vec3{ 0.0f, 1.0f, 2.2f });
        camera.SetRotation(-90.0f, 0.0f);
        camera.speed = 0.2f;
        Reset();
        Step();
    }

    void UpdateUI() override
    {
        ImGui::SetNextWindowPos({ Window::Get()->GetWindowSize().x - 5.0f, 5.0f }, ImGuiCond_Always, { 1.0f, 0.0f });
        if (ImGui::Begin("Collision detection", NULL, ImGuiWindowFlags_AlwaysAutoResize | ImGuiWindowFlags_NoScrollbar))
        {
            bool changed = false;

            ImGui::SetNextItemWidth(100.0f);
            if (ImGui::Combo("shape1", &item1, shapeItems, IM_ARRAYSIZE(shapeItems)))
            {
                UpdateShape1();
                changed = true;
            }

            ImGui::SameLine();
            ImGui::SetNextItemWidth(100.0f);
            if (ImGui::Combo("shape2", &item2, shapeItems, IM_ARRAYSIZE(shapeItems)))
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
            ImGui::Text("Collide: %s", collide ? "true" : "false");
            ImGui::Text("Contacts: %d", manifold.contactCount);
            ImGui::Text("Penetration: %.4f", manifold.penetrationDepth);
            ImGui::Text("Normal: %.3f, %.3f, %.3f", manifold.contactNormal.x, manifold.contactNormal.y, manifold.contactNormal.z);

            if (changed)
            {
                Step();
            }
        }
        ImGui::End();
    }

    void Step() override
    {
        collide = Collide(shape1.get(), tf1, shape2.get(), tf2, &manifold);
    }

    void Render() override
    {
        float prevSize = renderer.GetPointSize();
        renderer.SetPointSize(7.0f);

        Vec4 color = Renderer::default_white;
        if (options.draw_outlined)
        {
            glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
            color = Renderer::default_black;
        }
        else
        {
            glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
        }
        renderer.DrawShape(shape1.get(), tf1, color);
        renderer.DrawShape(shape2.get(), tf2, color);

        if (collide)
        {
            const Vec4 pointColor{ 1.0f, 0.18f, 0.08f, 1.0f };
            const Vec4 normalColor{ 0.05f, 0.25f, 1.0f, 1.0f };

            for (int32 i = 0; i < manifold.contactCount; ++i)
            {
                Vec3 p1 = manifold.contactPoints[i].p;
                Vec3 p2 = p1 + manifold.contactNormal * 0.35f;
                Vec3 reference = Abs(manifold.contactNormal.y) < 0.8f ? Vec3{ 0.0f, 1.0f, 0.0f } : Vec3{ 1.0f, 0.0f, 0.0f };
                Vec3 tangent = NormalizeSafe(Cross(manifold.contactNormal, reference));
                Vec3 arrowBase = p2 - manifold.contactNormal * 0.08f;
                Vec3 arrowA = arrowBase + tangent * 0.04f;
                Vec3 arrowB = arrowBase - tangent * 0.04f;

                renderer.DrawPoint(p1, pointColor);
                renderer.DrawLine(p1, p2, normalColor);
                renderer.DrawLine(p2, arrowA, normalColor);
                renderer.DrawLine(p2, arrowB, normalColor);
            }
        }

        renderer.FlushAll();
        renderer.SetPointSize(prevSize);
    }

private:
    void Reset()
    {
        tf1 = Transform{ Vec3{ -0.55f, 1.0f, 0.0f } };
        tf2 = Transform{ Vec3{ 0.55f, 1.0f, 0.0f } };
        rot1 = Vec3::zero;
        rot2 = Vec3{ 15.0f, 30.0f, 0.0f };
        tf1.q = Quat::FromEuler(Vec3{ DegToRad(rot1.x), DegToRad(rot1.y), DegToRad(rot1.z) });
        tf2.q = Quat::FromEuler(Vec3{ DegToRad(rot2.x), DegToRad(rot2.y), DegToRad(rot2.z) });
        size1 = Vec3{ 0.65f, 0.65f, 0.65f };
        size2 = Vec3{ 0.7f, 0.55f, 0.6f };
        convexRadius1 = default_radius;
        convexRadius2 = default_radius;
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
            shape2.reset(new Box(size2, convexRadius2));
            break;
        default:
            break;
        }
    }

    Transform tf1{ Vec3{ -0.55f, 1.0f, 0.0f } };
    Transform tf2{ Vec3{ 0.55f, 1.0f, 0.0f } };
    std::unique_ptr<Shape> shape1;
    std::unique_ptr<Shape> shape2;
    int32 item1 = 0;
    int32 item2 = 1;
    Vec3 rot1 = Vec3::zero;
    Vec3 rot2{ 0.0f, 24.0f, 0.0f };
    Vec3 size1{ 0.65f, 0.65f, 0.65f };
    Vec3 size2{ 0.7f, 0.55f, 0.6f };
    float convexRadius1 = default_radius;
    float convexRadius2 = default_radius;
    bool collide = false;
    ContactManifold manifold;
};

static Demo* CreateCollisionDetection(Game& game)
{
    return new CollisionDetection(game);
}

static int32 collision_detection = register_demo("Collision detection", CreateCollisionDetection, 24);

} // namespace muli3
