#include "demo.h"

#include "renderer.h"
#include "window.h"

namespace muli3
{

class RayCasting : public Demo
{
public:
    Vec3 from{ -4.0f, -0.5, 0.0f };
    Vec3 to{ 4.0f, 0.5f, 0.0f };
    Vec3 targetRot = { 0, 15, 0 };

    bool closest = true;

    RayCasting(Game& game)
        : Demo(game)
    {
        settings.apply_gravity = false;
        settings.sleeping = false;

        camera.SetPosition(Vec3{ 0.0f, 0.0f, 5.0f });
        camera.SetRotation(-90.0f, 0.0f);
        camera.speed = 0.35f;

        targetBodies[0] = world->CreateSphere(0.3f, Transform{ Vec3{ -2.5f, 0.0f, 0.0f } });
        targetBodies[1] = world->CreateCapsule(0.5f, 0.2f, Transform{ Vec3{ -1.5f, 0.0f, 0.0f } });
        targetBodies[2] = world->CreateBox(0.3f, Transform{ Vec3{ -0.5f, 0.0f, 0.0f } }, Body::dynamic_body);
        targetBodies[3] = world->CreateBox(0.35f, Transform{ Vec3{ 0.5f, 0.0f, 0.0f } }, Body::dynamic_body);

        Vec3 triangleVertices[3] = {
            Vec3{ -0.35f, -0.3f, 0.0f },
            Vec3{ 0.35f, -0.3f, 0.0f },
            Vec3{ 0.0f, 0.35f, 0.0f },
        };
        targetBodies[4] = world->CreateTriangle(triangleVertices, Transform{ Vec3{ 1.5f, 0.0f, 0.0f } });

        constexpr int32 segmentCount = 16;
        Vec3 polygonVertices[segmentCount];
        for (int32 i = 0; i < segmentCount; ++i)
        {
            float angle = two_pi * i / segmentCount;
            polygonVertices[i] = Vec3{ 0.35f * std::cos(angle), 0.35f * std::sin(angle), 0.0f };
        }
        targetBodies[5] = world->CreatePolygon(polygonVertices, Transform{ Vec3{ 2.5f, 0.0f, 0.0f } });

        UpdateTargetRotation();
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
        ImGui::SetNextWindowPos({ Window::Get()->GetWindowSize().x - 5.0f, 5.0f }, ImGuiCond_Always, { 1.0f, 0.0f });

        if (ImGui::Begin("Ray casting", NULL, ImGuiWindowFlags_AlwaysAutoResize | ImGuiWindowFlags_NoScrollbar))
        {
            ImGui::Checkbox("Closest", &closest);
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
        Vec3 closestPoint = Vec3::zero;
        Vec3 closestNormal = Vec3::zero;
        float prevPointSize = renderer.GetPointSize();
        renderer.SetPointSize(7.0f);
        int32 colorIndex = 0;

        world->RayCastAny(from, to, [&](Collider* collider, Vec3 point, Vec3 normal, float fraction) -> float {
            MuliNotUsed(collider);

            hit = true;

            if (closest == false)
            {
                Vec3 rgb = color::HSLToRGB({ (colorIndex++) * 35.9f / 360.0f, 1.0f, 0.5f });
                Vec4 color{ rgb.x, rgb.y, rgb.z, 1.0f };
                renderer.DrawPoint(point, color);
                renderer.DrawLine(point, point + normal * 0.25f);

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
        }

        renderer.FlushAll();
        renderer.SetPointSize(prevPointSize);
    }

private:
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
    Body* targetBodies[6] = {};
};

static Demo* CreateRayCasting(Game& game)
{
    return new RayCasting(game);
}

static int32 ray_casting = register_demo("Raycast", "Ray casting", CreateRayCasting, 0);

} // namespace muli3
