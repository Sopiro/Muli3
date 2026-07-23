#include "demo.h"
#include "game.h"
#include "window.h"

namespace muli3
{

static float coneHeight = 2.0f;
static float coneRadius = 1.2f;
static float tiltAngle = 10.0f;
static float spinSpeed = 3600.0f;

class Precession : public Demo,
                   public BodyDestroyCallback
{
public:
    Precession(Game& game)
        : Demo(game)
    {
        float groundHeight = 0.5f;
        float radius = default_radius;

        Body* ground = world->CreateBox(
            24.0f, groundHeight, 24.0f, Transform{ Vec3{ 0.0f, -groundHeight * 0.5f, 0.0f } }, Body::static_body
        );

        Quat rotation{ DegToRad(tiltAngle), z_axis };
        Vec3 localTip{ 0.0f, -coneHeight * 0.5f, 0.0f };
        Vec3 tipPosition{ 0.0f, radius * 2 + 1.0f, 0.0f };
        Vec3 position = tipPosition - rotation.Rotate(localTip);

        spinner =
            world->CreateCylinder(coneHeight, coneRadius, 0.0f, 12, Transform{ position, rotation }, Body::dynamic_body, radius);
        spinner->OnDestroy = this;
        spinner->SetFriction(1.0f);
        spinner->SetLinearDamping(0.0f);
        spinner->SetAngularDamping(0.0f);

        Vec3 spinAxis = rotation.Rotate(y_axis);
        spinner->SetAngularVelocity(spinAxis * DegToRad(spinSpeed));

        camera.SetRotation(0, -15);
    }

    ~Precession()
    {
        if (spinner)
        {
            spinner->OnDestroy = nullptr;
        }
    }

    void OnBodyDestroy(Body*) override
    {
        spinner = nullptr;
    }

    void Render() override
    {
        if (spinner == nullptr)
        {
            return;
        }

        const Transform& transform = spinner->GetTransform();
        Vec3 tip = Mul(transform, Vec3{ 0.0f, -coneHeight * 0.5f, 0.0f });
        Vec3 top = Mul(transform, Vec3{ 0.0f, coneHeight * 0.5f, 0.0f });
        Vec3 center = spinner->GetMotion().c;

        renderer.DrawLine(tip, top, Vec4{ 0.15f, 0.75f, 0.25f, 1.0f });
        renderer.DrawLine(tip, tip + y_axis * coneHeight, Vec4{ 0.25f, 0.25f, 0.25f, 0.5f });

        Vec3 angularMomentum = spinner->GetWorldInertiaTensor() * spinner->GetAngularVelocity();
        float momentumLength = Length(angularMomentum);
        if (momentumLength > 0.0f)
        {
            renderer.DrawLine(
                center, center + angularMomentum * (coneHeight / momentumLength), Vec4{ 0.95f, 0.25f, 0.15f, 1.0f }
            );
        }

        trail[trailIndex] = top;
        trailIndex = (trailIndex + 1) % int32(trail.size());
        trailCount = Min(trailCount + 1, int32(trail.size()));

        for (int32 i = 1; i < trailCount; ++i)
        {
            int32 index0 = (trailIndex - trailCount + i - 1 + int32(trail.size())) % int32(trail.size());
            int32 index1 = (index0 + 1) % int32(trail.size());
            renderer.DrawLine(trail[index0], trail[index1], Vec4{ 0.2f, 0.45f, 0.95f, 0.7f });
        }
    }

    void UpdateUI() override
    {
        ImGui::SetNextWindowPos({ Window::Get()->GetWindowSize().x - 5.0f, 5.0f }, ImGuiCond_Always, { 1.0f, 0.0f });

        if (ImGui::Begin("Precession", nullptr, ImGuiWindowFlags_AlwaysAutoResize))
        {
            bool restart = false;
            restart |= ImGui::SliderFloat("Height", &coneHeight, 1.0f, 5.0f, "%.2f");
            restart |= ImGui::SliderFloat("Top radius", &coneRadius, 0.3f, 2.0f, "%.2f");
            restart |= ImGui::SliderFloat("Tilt", &tiltAngle, 1.0f, 30.0f, "%.1f deg");
            restart |= ImGui::SliderFloat("Spin", &spinSpeed, 0.0f, 5400.0f, "%.0f deg/s");

            if (restart)
            {
                game.RestartDemo();
            }

            ImGui::Separator();
            ImGui::Text("Green: symmetry axis");
            ImGui::Text("Red: angular momentum");
            ImGui::Text("Blue: axis trace");
        }
        ImGui::End();
    }

private:
    Body* spinner = nullptr;
    std::array<Vec3, 512> trail{};
    int32 trailIndex = 0;
    int32 trailCount = 0;
};

static Demo* CreatePrecession(Game& game)
{
    return new Precession(game);
}

static int32 precession = register_demo("Dynamics", "Precession", CreatePrecession, 4);

} // namespace muli3
