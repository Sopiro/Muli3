#include "demo.h"

#include "renderer.h"
#include "window.h"

namespace muli3
{

namespace
{

static const char* items[] = { "Sphere" };

} // namespace

class CollisionDetection final : public Demo
{
public:
    CollisionDetection(Game& game)
        : Demo(game)
    {
        settings.apply_gravity = false;

        body1 = world->CreateEmptyBody(tf1, RigidBody::static_body);
        body2 = world->CreateEmptyBody(tf2, RigidBody::static_body);
        UpdateShape1();
        UpdateShape2();
    }

    void UpdateUI() override
    {
        ImGui::SetNextWindowPos({ Window::Get()->GetWindowSize().x - 5.0f, 5.0f }, ImGuiCond_Always, { 1.0f, 0.0f });
        if (ImGui::Begin("Collision detection", NULL, ImGuiWindowFlags_AlwaysAutoResize | ImGuiWindowFlags_NoScrollbar))
        {
            ImGui::SetNextItemWidth(100.0f);
            if (ImGui::Combo("shape1", &item1, items, IM_ARRAYSIZE(items)))
            {
                UpdateShape1();
            }

            ImGui::SameLine();
            ImGui::SetNextItemWidth(100.0f);
            if (ImGui::Combo("shape2", &item2, items, IM_ARRAYSIZE(items)))
            {
                UpdateShape2();
            }

            ImGui::SetNextItemWidth(120.0f);
            if (ImGui::DragFloat3("pos1", &tf1.p.x, 0.01f))
            {
                body1->SetTransform(tf1);
            }

            ImGui::SameLine();
            ImGui::SetNextItemWidth(120.0f);
            if (ImGui::DragFloat3("pos2", &tf2.p.x, 0.01f))
            {
                body2->SetTransform(tf2);
            }

            ImGui::SetNextItemWidth(120.0f);
            if (ImGui::DragFloat("radius1", &radius1, 0.01f, 0.05f, 5.0f))
            {
                UpdateShape1();
            }

            ImGui::SameLine();
            ImGui::SetNextItemWidth(120.0f);
            if (ImGui::DragFloat("radius2", &radius2, 0.01f, 0.05f, 5.0f))
            {
                UpdateShape2();
            }

            ImGui::Separator();
            ImGui::Text("Collide: %s", collide ? "true" : "false");
            ImGui::Text("Contacts: %d", manifold.contactCount);
            ImGui::Text("Penetration: %.4f", manifold.penetrationDepth);
        }
        ImGui::End();
    }

    void Step() override
    {
        collide = Collide(body1->GetShape(), tf1, body2->GetShape(), tf2, &manifold);
    }

    void Render() override
    {
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
    }

private:
    void UpdateShape1()
    {
        switch (item1)
        {
        case 0:
            body1->CreateSphereShape(radius1);
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
            body2->CreateSphereShape(radius2);
            break;
        default:
            break;
        }
    }

    Transform tf1{ Vec3{ -0.55f, 1.0f, 0.0f } };
    Transform tf2{ Vec3{ 0.55f, 1.0f, 0.0f } };
    RigidBody* body1 = nullptr;
    RigidBody* body2 = nullptr;
    int32 item1 = 0;
    int32 item2 = 0;
    float radius1 = 0.6f;
    float radius2 = 0.55f;
    bool collide = false;
    ContactManifold manifold;
};

static Demo* CreateCollisionDetection(Game& game)
{
    return new CollisionDetection(game);
}

static int32 collision_detection = register_demo("Collision detection", CreateCollisionDetection, 24);

} // namespace muli3
