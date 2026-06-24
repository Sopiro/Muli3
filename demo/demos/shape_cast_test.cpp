#include "demo.h"

#include "renderer.h"
#include "window.h"

#include <memory>

namespace muli3
{

static const char* shapeCastItems[] = { "Sphere", "Capsule", "Box", "Triangle" };

class ShapeCastTest : public Demo
{
public:
    Transform tf = identity;
    Vec3 rot = Vec3::zero;
    Vec3 targetRot = Vec3::zero;

    std::unique_ptr<Shape> shape;

    Vec3 from{ -3.0f, 0.0, 0.0f };
    Vec3 to{ 3.0f, 0.5f, 0.0f };

    bool closest = true;
    int32 item = 1;

    ShapeCastTest(Game& game)
        : Demo(game)
    {
        settings.apply_gravity = false;
        settings.sleeping = false;

        camera.SetPosition(Vec3{ 0.0f, 0.0f, 5.0f });
        camera.SetRotation(-90.0f, 0.0f);
        camera.speed = 0.35f;

        targetBodies[0] = world->CreateSphere(0.3f, Transform{ Vec3{ 1.5f, 0.0f, 0.0f } });
        targetBodies[1] = world->CreateCapsule(0.5f, 0.2f, Transform{ Vec3{ -0.5f, 0.0f, 0.0f } });
        targetBodies[2] = world->CreateBox(0.3f, Transform{ Vec3{ 0.5f, 0.0f, 0.0f } }, Body::dynamic_body);
        targetBodies[3] = world->CreateBox(0.35f, Transform{ Vec3{ -1.5f, 0.0f, 0.0f } }, Body::dynamic_body);

        Vec3 triangleVertices[3] = {
            Vec3{ -0.35f, -0.3f, 0.0f },
            Vec3{ 0.35f, -0.3f, 0.0f },
            Vec3{ 0.0f, 0.35f, 0.0f },
        };
        targetBodies[4] = world->CreateTriangle(triangleVertices, Transform{ Vec3{ 2.3f, 0.0f, 0.0f } });

        UpdateShape();
    }

    void UpdateInput() override
    {
        FindTargetBody();
        EnableKeyboardShortcut();
        EnableCameraControl();

        if (Window::Get()->GetCursorHidden() || ImGui::GetIO().WantCaptureMouse)
        {
            return;
        }

        Vec3 point;
        if (!dragging && Input::IsMousePressed(GLFW_MOUSE_BUTTON_LEFT) && GetMousePoint(&point))
        {
            from = point;
            to = point;
            dragging = true;
        }

        if (dragging && Input::IsMouseDown(GLFW_MOUSE_BUTTON_LEFT) && GetMousePoint(&point))
        {
            to = point;
        }

        if (dragging && Input::IsMouseReleased(GLFW_MOUSE_BUTTON_LEFT))
        {
            dragging = false;
        }
    }

    void UpdateUI() override
    {
        ImGui::SetNextWindowPos({ Window::Get()->GetWindowSize().x - 5.0f, 5.0f }, ImGuiCond_Once, { 1.0f, 0.0f });

        if (ImGui::Begin(
                "Shape cast world", NULL,
                ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_AlwaysAutoResize | ImGuiWindowFlags_NoScrollbar
            ))
        {
            ImGui::Checkbox("Closest", &closest);

            ImGui::SetNextItemWidth(100.0f);
            if (ImGui::Combo("shape", &item, shapeCastItems, IM_ARRAYSIZE(shapeCastItems)))
            {
                UpdateShape();
            }
            ImGui::DragFloat3("Rot", &rot.x, 1.0f, -360.0f, 360.0f);
            if (ImGui::DragFloat3("Target rot", &targetRot.x, 1.0f, -360.0f, 360.0f))
            {
                UpdateTargetRotation();
            }
        }
        ImGui::End();
    }

    void Render() override
    {
        bool hit = false;
        float closestFraction = 1.0f;
        Vec3 closestPoint = Vec3::zero;
        Vec3 closestNormal = Vec3::zero;
        tf.p = from;
        tf.q = Quat::FromEuler({ DegToRad(rot.x), DegToRad(rot.y), DegToRad(rot.z) });
        Vec3 translation = to - from;

        Renderer::DrawMode dm{};
        float prevPointSize = renderer.GetPointSize();
        renderer.SetPointSize(7.0f);

        world->ShapeCastAny(
            shape.get(), tf, translation, [&](Collider* collider, Vec3 point, Vec3 normal, float fraction) -> float {
                MuliNotUsed(collider);

                hit = true;

                if (closest == false)
                {
                    renderer.DrawPoint(point);
                    renderer.DrawLine(point, point + normal * 0.25f);
                    ++dm.colorIndex;
                    renderer.DrawShape(shape.get(), Transform{ from + translation * fraction, tf.q }, dm);

                    return 1.0f;
                }

                closestPoint = point;
                closestNormal = normal;
                closestFraction = fraction;
                return fraction;
            }
        );

        renderer.DrawLine(from, to);
        renderer.DrawPoint(from, Vec4{ 1.0f, 0.0f, 0.0f, 1.0f });
        renderer.DrawPoint(to, Vec4{ 0.0f, 0.0f, 1.0f, 1.0f });

        if (closest && hit)
        {
            renderer.DrawPoint(closestPoint);
            renderer.DrawLine(closestPoint, closestPoint + closestNormal * 0.25f);
            renderer.DrawShape(shape.get(), Transform{ from + translation * closestFraction, tf.q });
        }

        renderer.FlushAll();
        renderer.SetPointSize(prevPointSize);
    }

private:
    void UpdateShape()
    {
        switch (item)
        {
        case 0:
            shape.reset(new SphereShape(0.2f));
            break;
        case 1:
            shape.reset(new CapsuleShape(0.3f, 0.14f));
            break;
        case 2:
            shape.reset(new BoxShape(0.3f));
            break;
        case 3:
            shape.reset(new TriangleShape(
                Vec3{ -0.25f, -0.22f, 0.0f },
                Vec3{ 0.25f, -0.22f, 0.0f },
                Vec3{ 0.0f, 0.28f, 0.0f },
                default_radius
            ));
            break;
        default:
            break;
        }
    }

    void UpdateTargetRotation()
    {
        Quat q = Quat::FromEuler({ DegToRad(targetRot.x), DegToRad(targetRot.y), DegToRad(targetRot.z) });
        for (Body* body : targetBodies)
        {
            body->SetRotation(q);
        }
    }

    bool GetMousePoint(Vec3* point) const
    {
        Ray ray = GetMouseRay();
        if (Abs(ray.d.z) <= epsilon)
        {
            return false;
        }

        float t = -ray.o.z / ray.d.z;
        if (t < 0.0f)
        {
            return false;
        }

        *point = ray.o + ray.d * t;
        return true;
    }

    bool dragging = false;
    Body* targetBodies[5] = {};
};

static Demo* CreateShapeCastWorld(Game& game)
{
    return new ShapeCastTest(game);
}

static int32 shape_cast_world = register_demo("Collision", "Shape casting", CreateShapeCastWorld, 3);

} // namespace muli3
