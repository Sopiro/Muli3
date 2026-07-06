#include "demo.h"

#include "renderer.h"
#include "window.h"

namespace muli3
{

static const char* shapeItems[] = { "Sphere", "Capsule", "Box", "Quad", "Triangle" };

class CollisionDetection : public Demo
{
public:
    CollisionDetection(Game& game)
        : Demo(game)
    {
        settings.apply_gravity = false;
        camera.SetPosition(Vec3{ 0.0f, 0.0f, 3.0f });
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

        if (options.body_draw_mode == body_draw_depth_wireframe)
        {
            glColorMask(GL_FALSE, GL_FALSE, GL_FALSE, GL_FALSE);
            renderer.DrawShape(shape1.get(), tf1);
            renderer.DrawShape(shape2.get(), tf2);
            renderer.FlushShapes();
            glColorMask(GL_TRUE, GL_TRUE, GL_TRUE, GL_TRUE);
        }

        Renderer::DrawMode mode;
        mode.fill = options.body_draw_mode == body_draw_solid || options.body_draw_mode == body_draw_solid_wireframe;
        mode.outline = options.body_draw_mode == body_draw_solid_wireframe || options.body_draw_mode == body_draw_wireframe ||
                       options.body_draw_mode == body_draw_depth_wireframe;
        renderer.DrawShape(shape1.get(), tf1, mode);
        renderer.DrawShape(shape2.get(), tf2, mode);

        if (collide)
        {
            const Vec4 pointColor1{ 1.0f, 0.18f, 0.08f, 1.0f };
            const Vec4 pointColor2{ 0.08f, 0.18f, 1.0f, 1.0f };
            const Vec4 normalColor{ 0.05f, 0.25f, 1.0f, 1.0f };

            for (int32 i = 0; i < manifold.contactCount; ++i)
            {
                Vec3 p1 = manifold.contactPoints[i].anchorB;
                Vec3 normal = manifold.normal;
                Vec3 p2 = p1 + normal * 0.35f;
                Vec3 reference = Abs(normal.y) < 0.8f ? Vec3{ 0.0f, 1.0f, 0.0f } : Vec3{ 1.0f, 0.0f, 0.0f };
                Vec3 tangent = NormalizeSafe(Cross(normal, reference));
                Vec3 arrowBase = p2 - normal * 0.08f;
                Vec3 arrowA = arrowBase + tangent * 0.04f;
                Vec3 arrowB = arrowBase - tangent * 0.04f;

                renderer.DrawPoint(manifold.contactPoints[i].anchorA, pointColor1);
                renderer.DrawPoint(manifold.contactPoints[i].anchorB, pointColor2);
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
        tf1 = Transform{ Vec3{ -1.0f, 0.0f, 0.0f } };
        tf2 = Transform{ Vec3{ 0, 0.0f, 0.0f } };
        rot1 = { 0, 0, 0 };
        rot2 = { 15, 30, 0 };
        tf1.q = Quat::FromEuler(Vec3{ DegToRad(rot1.x), DegToRad(rot1.y), DegToRad(rot1.z) });
        tf2.q = Quat::FromEuler(Vec3{ DegToRad(rot2.x), DegToRad(rot2.y), DegToRad(rot2.z) });
        size1 = Vec3{ 1.0f };
        size2 = Vec3{ 1.0f };
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
            shape1.reset(new SphereShape(size1.x / 2));
            break;
        case 1:
            shape1.reset(new CapsuleShape(size1.x, size1.y / 2));
            break;
        case 2:
            shape1.reset(new BoxShape(size1, convexRadius1));
            break;
        case 3:
            shape1.reset(new QuadShape(
                Vec3{ -size1.x * 0.5f, -size1.y * 0.5f, 0.0f }, Vec3{ size1.x * 0.5f, -size1.y * 0.5f, 0.0f },
                Vec3{ size1.x * 0.5f, size1.y * 0.5f, 0.0f }, Vec3{ -size1.x * 0.5f, size1.y * 0.5f, 0.0f }, convexRadius1
            ));
            break;
        case 4:
            shape1.reset(new TriangleShape(
                Vec3{ -size1.x * 0.5f, -size1.y * 0.5f, 0.0f }, Vec3{ size1.x * 0.5f, -size1.y * 0.5f, 0.0f },
                Vec3{ 0.0f, size1.y * 0.5f, 0.0f }, convexRadius1
            ));
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
            shape2.reset(new SphereShape(size2.x / 2));
            break;
        case 1:
            shape2.reset(new CapsuleShape(size2.x, size2.y / 2));
            break;
        case 2:
            shape2.reset(new BoxShape(size2, convexRadius2));
            break;
        case 3:
            shape2.reset(new QuadShape(
                Vec3{ -size2.x * 0.5f, -size2.y * 0.5f, 0.0f }, Vec3{ size2.x * 0.5f, -size2.y * 0.5f, 0.0f },
                Vec3{ size2.x * 0.5f, size2.y * 0.5f, 0.0f }, Vec3{ -size2.x * 0.5f, size2.y * 0.5f, 0.0f }, convexRadius2
            ));
            break;
        case 4:
            shape2.reset(new TriangleShape(
                Vec3{ -size2.x * 0.5f, -size2.y * 0.5f, 0.0f }, Vec3{ size2.x * 0.5f, -size2.y * 0.5f, 0.0f },
                Vec3{ 0.0f, size2.y * 0.5f, 0.0f }, convexRadius2
            ));
            break;
        default:
            break;
        }
    }

    int32 item1 = 1;
    int32 item2 = 2;

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
    bool collide = false;
    ContactManifold manifold;
};

static Demo* CreateCollisionDetection(Game& game)
{
    return new CollisionDetection(game);
}

static int32 collision_detection = register_demo("Collision", "Collision detection", CreateCollisionDetection, 0);

} // namespace muli3
