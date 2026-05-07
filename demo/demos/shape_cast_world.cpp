#include "demo.h"

#include "renderer.h"
#include "window.h"

#include <memory>

namespace muli3
{

static const char* shapeCastItems[] = { "Sphere", "Capsule", "Box" };

class ShapeCastWorld : public Demo
{
public:
    Transform tf = identity;
    std::unique_ptr<Shape> shape;

    Vec3 from{ -3.0f, 0.0, 0.0f };
    Vec3 to{ 3.0f, 0.5f, 0.0f };

    bool closest = true;
    int32 item = 1;

    ShapeCastWorld(Game& game)
        : Demo(game)
    {
        settings.apply_gravity = false;
        settings.sleeping = false;

        camera.SetPosition(Vec3{ 0.0f, 0.0f, 5.0f });
        camera.SetRotation(-90.0f, 0.0f);
        camera.speed = 0.35f;

        RigidBody* body;
        body = world->CreateSphere(0.3f, Transform{ Vec3{ 1.5f, 0.0f, 0.0f } });
        body = world->CreateCapsule(0.5f, 0.2f, Transform{ Vec3{ -0.5f, 0.0f, 0.0f } });
        body = world->CreateBox(0.3f, Transform{ Vec3{ 0.5f, 0.0f, 0.0f } }, RigidBody::dynamic_body);
        body = world->CreateBox(0.35f, Transform{ Vec3{ -1.5f, 0.0f, 0.0f } }, RigidBody::dynamic_body);

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
        tf.q = identity;
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
            shape.reset(new Sphere(0.2f));
            break;
        case 1:
            shape.reset(new Capsule(0.3f, 0.14f));
            break;
        case 2:
            shape.reset(new Box(0.3f));
            break;
        default:
            break;
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
};

static Demo* CreateShapeCastWorld(Game& game)
{
    return new ShapeCastWorld(game);
}

static int32 shape_cast_world = register_demo("Collision", "Shape cast world", CreateShapeCastWorld, 10);

} // namespace muli3
