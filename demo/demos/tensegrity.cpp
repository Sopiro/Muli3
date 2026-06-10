#include "demo.h"
#include "game.h"
#include "window.h"

namespace muli3
{

static float cableFrequency = 15.0f;
static float prestress = 0.99f;

class Tensegrity : public Demo
{
public:
    Tensegrity(Game& game)
        : Demo(game)
    {
        Body* ground = world->CreateBox(18.0f, 0.5f, 16.0f, identity, Body::static_body);
        ground->SetFriction(0.8f);

        constexpr int32 count = 3;
        float prismX = -2.4f;
        float radius = 1.5f;
        float bottomHeight = 0.55f;
        float topHeight = 3.95f;
        float twist = DegToRad(30.0f);

        Vec3 bottom[count];
        Vec3 top[count];
        Body* bottomBodies[count];
        Body* topBodies[count];

        for (int32 i = 0; i < count; ++i)
        {
            float angle = 2.0f * pi * i / count;
            bottom[i] = Vec3{ prismX + radius * std::cos(angle), bottomHeight, radius * std::sin(angle) };
            top[i] = Vec3{ prismX + radius * std::cos(angle + twist), topHeight, radius * std::sin(angle + twist) };
        }

        CollisionFilter strutFilter;
        strutFilter.group = -1;

        // Each compression strut skips one top vertex, producing the characteristic twisted prism.
        for (int32 i = 0; i < count; ++i)
        {
            int32 topIndex = (i + 1) % count;
            Body* strut = world->CreateCapsule(bottom[i], top[topIndex], 0.12f);
            strut->SetCollisionFilter(strutFilter);
            strut->SetFriction(0.8f);
            strut->SetLinearDamping(0.08f);
            strut->SetAngularDamping(0.08f);
            strut->SetGyroscopicTorqueEnabled(true);

            bottomBodies[i] = strut;
            topBodies[topIndex] = strut;
        }

        auto createCable = [&](Body* bodyA, Body* bodyB, const Vec3& anchorA, const Vec3& anchorB) {
            float length = Dist(anchorA, anchorB) * prestress;
            world->CreateLimitedDistanceJoint(bodyA, bodyB, anchorA, anchorB, 0.0f, length, cableFrequency, 0.8f);
        };

        for (int32 i = 0; i < count; ++i)
        {
            int32 next = (i + 1) % count;

            createCable(bottomBodies[i], bottomBodies[next], bottom[i], bottom[next]);
            createCable(topBodies[i], topBodies[next], top[i], top[next]);
            createCable(bottomBodies[i], topBodies[i], bottom[i], top[i]);
        }

        float frameX = 2.5f;
        Vec3 frameVertices[count] = {
            Vec3{ -1.4f, 0.0f, -1.0f },
            Vec3{ -1.4f, 0.0f, 1.0f },
            Vec3{ 1.5f, 0.0f, 0.0f },
        };

        Body* bottomFrame = world->CreateEmptyBody(Transform{ Vec3{ frameX, bottomHeight, 0.0f } });
        Body* topFrame = world->CreateEmptyBody(Transform{ Vec3{ frameX, topHeight, 0.0f } });

        for (int32 i = 0; i < count; ++i)
        {
            int32 next = (i + 1) % count;
            bottomFrame->CreateCapsuleCollider(frameVertices[i], frameVertices[next], 0.11f);
            topFrame->CreateCapsuleCollider(frameVertices[i], frameVertices[next], 0.11f);
        }

        Vec3 bottomTip{ 0.0f, 2.5f, 0.0f };
        Vec3 topTip{ 0.0f, -2.5f, 0.0f };
        Vec3 bottomStrutBase = (frameVertices[0] + frameVertices[1]) * 0.5f;
        bottomFrame->CreateCapsuleCollider(bottomStrutBase, bottomTip, 0.12f);
        topFrame->CreateCapsuleCollider(frameVertices[2], topTip, 0.12f);

        CollisionFilter frameFilter;
        frameFilter.group = -2;
        bottomFrame->SetCollisionFilter(frameFilter);
        topFrame->SetCollisionFilter(frameFilter);
        bottomFrame->SetFriction(0.8f);
        topFrame->SetFriction(0.8f);
        bottomFrame->SetLinearDamping(0.08f);
        bottomFrame->SetAngularDamping(0.08f);
        topFrame->SetLinearDamping(0.08f);
        topFrame->SetAngularDamping(0.08f);

        Vec3 bottomFrameVertices[count];
        Vec3 topFrameVertices[count];
        for (int32 i = 0; i < count; ++i)
        {
            bottomFrameVertices[i] = bottomFrame->GetPosition() + frameVertices[i];
            topFrameVertices[i] = topFrame->GetPosition() + frameVertices[i];
        }

        for (int32 i = 0; i < count; ++i)
        {
            createCable(bottomFrame, topFrame, bottomFrameVertices[i], topFrameVertices[i]);
        }

        createCable(bottomFrame, topFrame, bottomFrame->GetPosition() + bottomTip, topFrame->GetPosition() + topTip);

        camera.SetPosition(Vec3{ 0.0f, 3.2f, 12.0f });
        camera.SetRotation(-90.0f, -8.0f);
    }

    void UpdateUI() override
    {
        ImGui::SetNextWindowPos({ Window::Get()->GetWindowSize().x - 5.0f, 5.0f }, ImGuiCond_Once, { 1.0f, 0.0f });

        if (ImGui::Begin("Tensegrity", nullptr, ImGuiWindowFlags_AlwaysAutoResize))
        {
            if (ImGui::SliderFloat("Cable frequency", &cableFrequency, 1.0f, 30.0f, "%.1f"))
            {
                game.RestartDemo();
            }

            if (ImGui::SliderFloat("Prestress", &prestress, 0.85f, 1.0f, "%.3f"))
            {
                game.RestartDemo();
            }
        }
        ImGui::End();
    }
};

static Demo* CreateTensegrity(Game& game)
{
    return new Tensegrity(game);
}

static int32 tensegrity = register_demo("Dynamics", "Tensegrity", CreateTensegrity, 5);

} // namespace muli3
