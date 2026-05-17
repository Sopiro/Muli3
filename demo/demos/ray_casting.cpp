#include "demo.h"

#include "renderer.h"
#include "window.h"

namespace muli3
{

class RayCasting : public Demo
{
public:
    Vec3 from{ -3.0f, 0.0f, 0.0f };
    Vec3 to{ 3.0f, 0.5f, 0.0f };

    bool closest = true;
    float radius = 0.0f;

    RayCasting(Game& game)
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
                "Ray casting", NULL,
                ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_AlwaysAutoResize | ImGuiWindowFlags_NoScrollbar
            ))
        {
            ImGui::Checkbox("Closest", &closest);
            ImGui::DragFloat("Ray radius", &radius, 0.01f, 0.0f, 0.5f, "%.2f");
        }
        ImGui::End();
    }

    void Render() override
    {
        bool hit = false;
        Vec3 closestPoint = Vec3::zero;
        Vec3 closestNormal = Vec3::zero;
        SphereShape sphere{ radius };
        Renderer::DrawMode dm{};
        float prevPointSize = renderer.GetPointSize();
        renderer.SetPointSize(7.0f);

        world->RayCastAny(from, to, radius, [&](Collider* collider, Vec3 point, Vec3 normal, float fraction) -> float {
            MuliNotUsed(collider);

            hit = true;

            if (closest == false)
            {
                renderer.DrawPoint(point);
                renderer.DrawLine(point, point + normal * 0.25f);

                if (radius > 0.0f)
                {
                    ++dm.colorIndex;
                    renderer.DrawShape(&sphere, Transform{ point }, dm);
                }

                return 1.0f;
            }

            closestPoint = point;
            closestNormal = normal;
            return fraction;
        });

        renderer.DrawLine(from, to);
        renderer.DrawPoint(from, Vec4{ 1.0f, 0.0f, 0.0f, 1.0f });
        renderer.DrawPoint(to, Vec4{ 0.0f, 0.0f, 1.0f, 1.0f });

        if (closest && hit)
        {
            renderer.DrawPoint(closestPoint);
            renderer.DrawLine(closestPoint, closestPoint + closestNormal * 0.25f);

            if (radius > 0.0f)
            {
                renderer.DrawShape(&sphere, Transform{ closestPoint });
            }
        }

        renderer.FlushAll();
        renderer.SetPointSize(prevPointSize);
    }

private:
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

static Demo* CreateRayCasting(Game& game)
{
    return new RayCasting(game);
}

static int32 ray_casting = register_demo("Collision", "Ray casting", CreateRayCasting, 2);

} // namespace muli3
