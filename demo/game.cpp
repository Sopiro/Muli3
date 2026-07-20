#include "muli3/frame.h"

#include "game.h"
#include "profile_graph.h"
#include "window.h"

namespace muli3
{

extern int32 GetFrameRate();
extern void SetFrameRate(int32 newFrameRate);
extern int32 GetUpdateRate();
extern void SetUpdateRate(int32 newUpdateRate);
extern Vec4 g_colors2[constraint_color_count];

Game::Game()
{
    bool rendererInitialized = renderer.Initialize();
    MuliAssert(rendererInitialized);
    if (!rendererInitialized)
    {
        std::exit(1);
    }

    sort_demos();
    demoCount = GetDemoFrames().size();
    MuliAssert(demoCount > 0);

    demoIndex = demoCount;

    workerCount = 8;
    RecreateThreadPool();
    renderItems.reserve(2048);
    InitDemo(30);
    Window::Get()->SetCursorHidden(false);
}

Game::~Game()
{
    delete demo;
    renderer.Shutdown();
}

void Game::Update(float deltaTime)
{
    if (restart)
    {
        InitDemo(newIndex);
        restart = false;
    }

    dt = deltaTime;
    time += dt;
    UpdateUI();
    UpdateInput();
}

void Game::FixedUpdate()
{
    demo->Step();

    if (profileStopped)
    {
        return;
    }

    if (profileWriteIndex == profile_capacity + profileReadIndex)
    {
        ++profileReadIndex;
    }

    int32 index = (int32)(profileWriteIndex & (profile_capacity - 1));
    profiles[index] = demo->GetWorld().GetProfile();
    ++profileWriteIndex;
}

static Vec4 GetBodyColor(Renderer& renderer, const Body& body, const DebugOptions& options)
{
    if (body.IsStatic())
    {
        return Renderer::default_white;
    }

    if (options.colorize_island == false)
    {
        return body.IsSleeping() ? Vec4{ 0.9f, 0.9f, 0.9f, Renderer::default_white.w } : Renderer::default_white;
    }

    if (body.IsSleeping())
    {
        return Renderer::default_white;
    }

    int32 colorIndex = body.GetIslandIndex();
    if (colorIndex < 0)
    {
        return Renderer::default_white;
    }

    return renderer.GetColor(colorIndex);
}

static Vec4 GetContactColor(const Contact* contact)
{
    int32 colorIndex = contact->GetColorIndex();
    if (colorIndex < 0)
    {
        return Vec4{ 0.3f, 0.3f, 0.3f, 0.9f };
    }

    return g_colors2[colorIndex];
}

static bool IsSameBodyPair(const Joint* joint, const Body* bodyA, const Body* bodyB)
{
    return (joint->GetBodyA() == bodyA && joint->GetBodyB() == bodyB) ||
           (joint->GetBodyA() == bodyB && joint->GetBodyB() == bodyA);
}

static Vec3 GetAngularJointAnchor(const World& world, const Joint* joint)
{
    const Body* bodyA = joint->GetBodyA();
    const Body* bodyB = joint->GetBodyB();

    for (const Joint* other = world.GetJoints(); other; other = other->GetNext())
    {
        if (other == joint || other->GetType() != Joint::ball_socket_joint)
        {
            continue;
        }

        if (IsSameBodyPair(other, bodyA, bodyB) == false)
        {
            continue;
        }

        const BallSocketJoint* ballSocketJoint = (const BallSocketJoint*)other;
        Vec3 anchorA = Mul(bodyA->GetTransform(), ballSocketJoint->GetLocalAnchorA());
        Vec3 anchorB = Mul(bodyB->GetTransform(), ballSocketJoint->GetLocalAnchorB());
        return (anchorA + anchorB) * 0.5f;
    }

    return (bodyA->GetPosition() + bodyB->GetPosition()) * 0.5f;
}

static void DrawBasis(Renderer& renderer, const Vec3& origin, const Quat& rotation, float scale, float alpha)
{
    renderer.DrawLine(origin, origin + rotation.Rotate(x_axis) * scale, Vec4{ 0.95f, 0.2f, 0.2f, alpha });
    renderer.DrawLine(origin, origin + rotation.Rotate(y_axis) * scale, Vec4{ 0.2f, 0.85f, 0.2f, alpha });
    renderer.DrawLine(origin, origin + rotation.Rotate(z_axis) * scale, Vec4{ 0.2f, 0.45f, 1.0f, alpha });
}

static void DrawAxis(Renderer& renderer, const Vec3& origin, const Vec3& axis, float halfLength, const Vec4& color)
{
    renderer.DrawLine(origin - axis * halfLength, origin + axis * halfLength, color);
}

static void DrawConeLimit(Renderer& renderer, const Vec3& origin, const Vec3& axis, float angle, float length, const Vec4& color)
{
    if (angle <= 0.0f)
    {
        return;
    }

    Frame frame = Frame::FromZ(axis);
    float radius = std::tan(angle) * length;
    Vec3 tip = origin + axis * length;
    constexpr int32 segmentCount = 24;

    Vec3 firstPoint = Vec3::zero;
    Vec3 prevPoint = Vec3::zero;
    for (int32 i = 0; i <= segmentCount; ++i)
    {
        float t = two_pi * (float)i / (float)segmentCount;
        Vec3 point = tip + frame.x * std::cos(t) * radius + frame.y * std::sin(t) * radius;

        if (i == 0)
        {
            firstPoint = point;
        }
        else
        {
            renderer.DrawLine(prevPoint, point, color);
        }

        if (i < segmentCount && (i % 6) == 0)
        {
            renderer.DrawLine(origin, point, color);
        }

        prevPoint = point;
    }

    renderer.DrawLine(prevPoint, firstPoint, color);
}

static void DrawCircle(Renderer& renderer, const Vec3& origin, const Vec3& normal, float radius, const Vec4& color)
{
    Frame frame = Frame::FromZ(normal);
    constexpr int32 segmentCount = 40;

    Vec3 firstPoint = origin + frame.x * radius;
    Vec3 prevPoint = firstPoint;
    for (int32 i = 1; i <= segmentCount; ++i)
    {
        float t = two_pi * (float)i / (float)segmentCount;
        Vec3 point = origin + (frame.x * std::cos(t) + frame.y * std::sin(t)) * radius;
        renderer.DrawLine(prevPoint, point, color);
        prevPoint = point;
    }
}

static void DrawTwistArc(
    Renderer& renderer,
    const Vec3& origin,
    const Vec3& axis,
    const Vec3& t1,
    const Vec3& t2,
    float radius,
    float minAngle,
    float maxAngle,
    float currentAngle,
    const Vec4& limitColor,
    const Vec4& currentColor
)
{
    constexpr int32 segmentCount = 32;
    float span = maxAngle - minAngle;

    if (span > 0.0f)
    {
        Vec3 prevPoint = origin + (t1 * std::cos(minAngle) + t2 * std::sin(minAngle)) * radius;
        for (int32 i = 1; i <= segmentCount; ++i)
        {
            float angle = minAngle + span * (float)i / (float)segmentCount;
            Vec3 point = origin + (t1 * std::cos(angle) + t2 * std::sin(angle)) * radius;
            renderer.DrawLine(prevPoint, point, limitColor);
            prevPoint = point;
        }
    }

    Vec3 minDir = t1 * std::cos(minAngle) + t2 * std::sin(minAngle);
    Vec3 maxDir = t1 * std::cos(maxAngle) + t2 * std::sin(maxAngle);
    Vec3 currentDir = t1 * std::cos(currentAngle) + t2 * std::sin(currentAngle);

    renderer.DrawLine(origin, origin + minDir * radius, limitColor);
    renderer.DrawLine(origin, origin + maxDir * radius, limitColor);
    renderer.DrawLine(origin - axis * 0.4f, origin + axis * 0.4f, Vec4{ 0.12f, 0.12f, 0.12f, 0.65f });
    renderer.DrawLine(origin, origin + currentDir * radius, currentColor);
}

void Game::Render(float alpha)
{
    if (options.pause)
    {
        alpha = 1.0f;
    }

    Window* window = Window::Get();
    Vec2 windowSize = window->GetWindowSize();
    float aspectRatio = windowSize.y > 0.0f ? windowSize.x / windowSize.y : 1.0f;
    World& world = demo->GetWorld();

    renderer.BeginFrame(demo->GetCamera(), aspectRatio, skyColor, skyIntensity, lightDirection, lightColor, lightIntensity);

    bool drawSolid = options.body_draw_mode == body_draw_solid || options.body_draw_mode == body_draw_solid_wireframe;
    bool drawDepth = options.body_draw_mode == body_draw_depth_wireframe;
    bool drawWireframe = options.body_draw_mode == body_draw_solid_wireframe || options.body_draw_mode == body_draw_wireframe ||
                         options.body_draw_mode == body_draw_depth_wireframe;

    Camera& camera = demo->GetCamera();
    Mat4 vp = camera.GetProjectionMatrix(aspectRatio) * camera.GetViewMatrix();
    Vec4 row0{ vp.ex.x, vp.ey.x, vp.ez.x, vp.ew.x };
    Vec4 row1{ vp.ex.y, vp.ey.y, vp.ez.y, vp.ew.y };
    Vec4 row2{ vp.ex.z, vp.ey.z, vp.ez.z, vp.ew.z };
    Vec4 row3{ vp.ex.w, vp.ey.w, vp.ez.w, vp.ew.w };
    Vec4 frustumPlanes[] = { row3 + row0, row3 - row0, row3 + row1, row3 - row1, row3 + row2, row3 - row2 };

    renderItems.clear();
    if (drawSolid || drawDepth || drawWireframe)
    {
        for (Body* body = world.GetBodyList(); body; body = body->GetNext())
        {
            Vec4 color = GetBodyColor(renderer, *body, options);
            Transform transform = body->GetTransform();
            if (body->IsStatic() == false && body->IsSleeping() == false)
            {
                body->GetMotion().GetTransform(alpha, &transform);
            }

            for (const Collider* collider = body->GetColliderList(); collider; collider = collider->GetNext())
            {
                AABB bounds = collider->GetAABB();
                Vec3 center = (bounds.min + bounds.max) * 0.5f;
                Vec3 extents = (bounds.max - bounds.min) * 0.5f;
                bool visible = true;
                for (const Vec4& plane : frustumPlanes)
                {
                    float distance = plane.x * center.x + plane.y * center.y + plane.z * center.z + plane.w;
                    float radius = std::abs(plane.x) * extents.x + std::abs(plane.y) * extents.y + std::abs(plane.z) * extents.z;
                    if (distance + radius < 0.0f)
                    {
                        visible = false;
                        break;
                    }
                }
                renderItems.push_back({ collider->GetShape(), transform, color, visible });
            }
        }
    }

    renderer.BeginShadowPass();
    if (drawSolid)
    {
        for (const RenderItem& item : renderItems)
        {
            renderer.DrawShape(item.shape, item.transform, Renderer::default_white, false);
        }
    }
    renderer.EndShadowPass();

    renderer.BeginAoPass();
    if (drawSolid || drawDepth)
    {
        for (const RenderItem& item : renderItems)
        {
            if (item.visible)
            {
                renderer.DrawShape(item.shape, item.transform, item.color, false);
            }
        }
    }
    renderer.EndAoPass();

    renderer.BeginShapePass(drawSolid);

    if (drawWireframe)
    {
        for (const RenderItem& item : renderItems)
        {
            if (item.visible)
            {
                renderer.DrawShape(item.shape, item.transform, Renderer::default_black, true);
            }
        }
        renderer.FlushShapes();
    }

    if (options.draw_joint)
    {
        for (const Joint* joint = world.GetJoints(); joint; joint = joint->GetNext())
        {
            if (UserFlag::IsEnabled(joint, UserFlag::hide_joint))
            {
                continue;
            }

            switch (joint->GetType())
            {
            case Joint::grab_joint:
            {
                const Body* body = joint->GetBodyA();
                const GrabJoint* grabJoint = (const GrabJoint*)joint;
                Vec3 anchor = Mul(body->GetTransform(), grabJoint->GetLocalAnchor());
                renderer.DrawPoint(anchor);
                renderer.DrawPoint(grabJoint->GetTarget());
                renderer.DrawLine(anchor, grabJoint->GetTarget());
            }
            break;
            case Joint::fixed_rotation_joint:
            {
                const Body* body = joint->GetBodyA();
                const FixedRotationJoint* fixedRotationJoint = (const FixedRotationJoint*)joint;
                Vec3 position = body->GetPosition();
                renderer.DrawPoint(position);
                DrawBasis(renderer, position, fixedRotationJoint->GetTargetOrientation(), 0.55f, 0.55f);
            }
            break;
            case Joint::cone_swing_joint:
            {
                const Body* bodyA = joint->GetBodyA();
                const Body* bodyB = joint->GetBodyB();
                const ConeSwingJoint* coneSwingJoint = (const ConeSwingJoint*)joint;

                Vec3 positionA = bodyA->GetPosition();
                Vec3 positionB = bodyB->GetPosition();
                Vec3 axisA = bodyA->GetRotation().Rotate(coneSwingJoint->GetLocalAxisA());
                Vec3 axisB = bodyB->GetRotation().Rotate(coneSwingJoint->GetLocalAxisB());
                DrawAxis(renderer, positionB, axisB, 0.7f, Vec4{ 0.2f, 0.85f, 0.2f, 0.8f });
                DrawConeLimit(
                    renderer, positionA, axisA, coneSwingJoint->GetJointMaxAngle(), 0.8f, Vec4{ 0.9f, 0.2f, 0.2f, 0.5f }
                );
            }
            break;
            case Joint::revolute_angle_joint:
            {
                const Body* bodyA = joint->GetBodyA();
                const Body* bodyB = joint->GetBodyB();
                const RevoluteAngleJoint* revoluteAngleJoint = (const RevoluteAngleJoint*)joint;
                Vec3 anchor = GetAngularJointAnchor(world, joint);
                Vec3 axisA = bodyA->GetRotation().Rotate(revoluteAngleJoint->GetLocalAxisA());
                Vec3 axisB = bodyB->GetRotation().Rotate(revoluteAngleJoint->GetLocalAxisB());
                Vec3 t1, t2;
                CoordinateSystem(axisA, &t1, &t2);

                DrawCircle(renderer, anchor, axisA, 0.45f, Vec4{ 0.15f, 0.15f, 0.15f, 0.25f });
                DrawAxis(renderer, anchor, axisA, 0.55f, Vec4{ 0.95f, 0.3f, 0.2f, 0.55f });
                DrawAxis(renderer, anchor, axisB, 0.45f, Vec4{ 0.2f, 0.85f, 0.2f, 0.8f });
                DrawTwistArc(
                    renderer, anchor, axisA, t1, t2, 0.45f, revoluteAngleJoint->GetJointMinAngle(),
                    revoluteAngleJoint->GetJointMaxAngle(), revoluteAngleJoint->GetJointAngle(), Vec4{ 0.95f, 0.3f, 0.2f, 0.55f },
                    Vec4{ 0.15f, 0.45f, 1.0f, 0.85f }
                );
            }
            break;
            case Joint::revolute_joint:
            {
                const Body* bodyA = joint->GetBodyA();
                const Body* bodyB = joint->GetBodyB();
                const RevoluteJoint* revoluteJoint = (const RevoluteJoint*)joint;
                Vec3 anchorA = Mul(bodyA->GetTransform(), revoluteJoint->GetLocalAnchorA());
                Vec3 anchorB = Mul(bodyB->GetTransform(), revoluteJoint->GetLocalAnchorB());
                Vec3 anchor = (anchorA + anchorB) * 0.5f;
                Vec3 axisA = bodyA->GetRotation().Rotate(revoluteJoint->GetLocalAxisA());
                Vec3 axisB = bodyB->GetRotation().Rotate(revoluteJoint->GetLocalAxisB());
                Vec3 refAxisA = bodyA->GetRotation().Rotate(revoluteJoint->GetLocalNormalAxisA());
                Vec3 binormalA = Cross(axisA, refAxisA);
                binormalA.Normalize();

                renderer.DrawPoint(anchorA);
                renderer.DrawPoint(anchorB);
                renderer.DrawLine(anchorA, bodyA->GetPosition());
                renderer.DrawLine(anchorB, bodyB->GetPosition());
                renderer.DrawLine(anchorA, anchorB, Vec4{ 0.12f, 0.12f, 0.12f, 0.35f });

                DrawCircle(renderer, anchor, axisA, 0.45f, Vec4{ 0.15f, 0.15f, 0.15f, 0.25f });
                DrawAxis(renderer, anchor, axisA, 0.55f, Vec4{ 0.95f, 0.3f, 0.2f, 0.55f });
                DrawAxis(renderer, anchor, axisB, 0.45f, Vec4{ 0.2f, 0.85f, 0.2f, 0.8f });
                DrawTwistArc(
                    renderer, anchor, axisA, refAxisA, binormalA, 0.45f, revoluteJoint->GetJointMinAngle(),
                    revoluteJoint->GetJointMaxAngle(), revoluteJoint->GetJointAngle(), Vec4{ 0.95f, 0.3f, 0.2f, 0.55f },
                    Vec4{ 0.15f, 0.45f, 1.0f, 0.85f }
                );
            }
            break;
            case Joint::twist_angle_joint:
            {
                const Body* bodyA = joint->GetBodyA();
                const Body* bodyB = joint->GetBodyB();
                const TwistAngleJoint* twistAngleJoint = (const TwistAngleJoint*)joint;
                Vec3 anchor = GetAngularJointAnchor(world, joint);
                Vec3 axisA = bodyA->GetRotation().Rotate(twistAngleJoint->GetLocalAxisA());
                Vec3 axisB = bodyB->GetRotation().Rotate(twistAngleJoint->GetLocalAxisB());
                Vec3 t1, t2;
                CoordinateSystem(axisA, &t1, &t2);

                DrawCircle(renderer, anchor, axisA, 0.4f, Vec4{ 0.15f, 0.15f, 0.15f, 0.25f });
                DrawAxis(renderer, anchor, axisA, 0.5f, Vec4{ 0.95f, 0.3f, 0.2f, 0.55f });
                DrawAxis(renderer, anchor, axisB, 0.4f, Vec4{ 0.2f, 0.85f, 0.2f, 0.8f });
                DrawTwistArc(
                    renderer, anchor, axisA, t1, t2, 0.4f, twistAngleJoint->GetJointMinAngle(),
                    twistAngleJoint->GetJointMaxAngle(), twistAngleJoint->GetJointAngle(), Vec4{ 0.95f, 0.3f, 0.2f, 0.55f },
                    Vec4{ 0.15f, 0.45f, 1.0f, 0.85f }
                );
            }
            break;
            case Joint::ball_socket_joint:
            {
                const Body* bodyA = joint->GetBodyA();
                const Body* bodyB = joint->GetBodyB();
                const BallSocketJoint* ballSocketJoint = (const BallSocketJoint*)joint;
                Vec3 anchorA = Mul(bodyA->GetTransform(), ballSocketJoint->GetLocalAnchorA());
                Vec3 anchorB = Mul(bodyB->GetTransform(), ballSocketJoint->GetLocalAnchorB());
                renderer.DrawPoint(anchorA);
                renderer.DrawPoint(anchorB);
                renderer.DrawLine(anchorA, bodyA->GetPosition());
                renderer.DrawLine(anchorB, bodyB->GetPosition());
            }
            break;
            case Joint::distance_joint:
            {
                const Body* bodyA = joint->GetBodyA();
                const Body* bodyB = joint->GetBodyB();
                const DistanceJoint* distanceJoint = (const DistanceJoint*)joint;
                Vec3 anchorA = Mul(bodyA->GetTransform(), distanceJoint->GetLocalAnchorA());
                Vec3 anchorB = Mul(bodyB->GetTransform(), distanceJoint->GetLocalAnchorB());
                renderer.DrawPoint(anchorA);
                renderer.DrawPoint(anchorB);

                Vec3 d = anchorB - anchorA;
                if (d.Normalize() > 0.0f)
                {
                    renderer.DrawPoint(anchorA + d * distanceJoint->GetJointMinLength());
                    renderer.DrawPoint(anchorA + d * distanceJoint->GetJointMaxLength());
                }

                renderer.DrawLine(anchorA, anchorB);
            }
            break;
            case Joint::weld_joint:
            {
                const Body* bodyA = joint->GetBodyA();
                const Body* bodyB = joint->GetBodyB();
                const WeldJoint* weldJoint = (const WeldJoint*)joint;
                Vec3 anchorA = Mul(bodyA->GetTransform(), weldJoint->GetLocalAnchorA());
                Vec3 anchorB = Mul(bodyB->GetTransform(), weldJoint->GetLocalAnchorB());
                renderer.DrawPoint(anchorA);
                renderer.DrawPoint(anchorB);
                renderer.DrawLine(anchorA, bodyA->GetPosition());
                renderer.DrawLine(anchorB, bodyB->GetPosition());
            }
            break;
            case Joint::line_joint:
            {
                const Body* bodyA = joint->GetBodyA();
                const Body* bodyB = joint->GetBodyB();
                const LineJoint* lineJoint = (const LineJoint*)joint;
                Vec3 anchorA = Mul(bodyA->GetTransform(), lineJoint->GetLocalAnchorA());
                Vec3 anchorB = Mul(bodyB->GetTransform(), lineJoint->GetLocalAnchorB());
                renderer.DrawPoint(anchorA);
                renderer.DrawPoint(anchorB);
                renderer.DrawLine(anchorA, anchorB);
            }
            break;
            case Joint::prismatic_joint:
            {
                const Body* bodyA = joint->GetBodyA();
                const Body* bodyB = joint->GetBodyB();
                const PrismaticJoint* prismaticJoint = (const PrismaticJoint*)joint;
                Vec3 anchorA = Mul(bodyA->GetTransform(), prismaticJoint->GetLocalAnchorA());
                Vec3 anchorB = Mul(bodyB->GetTransform(), prismaticJoint->GetLocalAnchorB());
                renderer.DrawPoint(anchorA);
                renderer.DrawPoint(anchorB);
                renderer.DrawLine(anchorA, anchorB);
            }
            break;
            case Joint::pulley_joint:
            {
                const Body* bodyA = joint->GetBodyA();
                const Body* bodyB = joint->GetBodyB();
                const PulleyJoint* pulleyJoint = (const PulleyJoint*)joint;
                Vec3 anchorA = Mul(bodyA->GetTransform(), pulleyJoint->GetLocalAnchorA());
                Vec3 anchorB = Mul(bodyB->GetTransform(), pulleyJoint->GetLocalAnchorB());
                renderer.DrawPoint(anchorA);
                renderer.DrawPoint(pulleyJoint->GetGroundAnchorA());
                renderer.DrawPoint(anchorB);
                renderer.DrawPoint(pulleyJoint->GetGroundAnchorB());
                renderer.DrawLine(anchorA, pulleyJoint->GetGroundAnchorA());
                renderer.DrawLine(anchorB, pulleyJoint->GetGroundAnchorB());
            }
            break;
            case Joint::motor_joint:
            {
                const Body* bodyA = joint->GetBodyA();
                const Body* bodyB = joint->GetBodyB();
                const MotorJoint* motorJoint = (const MotorJoint*)joint;
                Vec3 anchorA = Mul(bodyA->GetTransform(), motorJoint->GetLocalAnchorA());
                Vec3 anchorB = Mul(bodyB->GetTransform(), motorJoint->GetLocalAnchorB());
                renderer.DrawPoint(anchorA);
                renderer.DrawPoint(anchorB);
            }
            break;
            default:
                break;
            }
        }
    }

    if (options.show_bvh || options.show_aabb)
    {
        const AABBTree& tree = world.GetDynamicTree();
        tree.Traverse([&](const AABBTree::Node* node) -> void {
            if (options.show_bvh == false && node->IsLeaf() == false)
            {
                return;
            }

            renderer.DrawAABB(node->aabb);
        });
    }

    if (options.show_contact_point || options.show_contact_normal)
    {
        const Vec4 normalColor{ 0.05f, 0.25f, 1.0f, 0.9f };
        for (const Contact* contact = world.GetContacts(); contact; contact = contact->GetNext())
        {
            if (contact->IsEnabled() == false || contact->IsTouching() == false)
            {
                continue;
            }

            const Vec4 pointColor = GetContactColor(contact);

            for (int32 m = 0; m < contact->GetManifoldCount(); ++m)
            {
                const ContactManifold& manifold = contact->GetContactManifold(m);

                for (int32 i = 0; i < manifold.contactCount; ++i)
                {
                    const Vec3 p1 = (manifold.contactPoints[i].anchorA + manifold.contactPoints[i].anchorB) * 0.5f;

                    if (options.show_contact_point)
                    {
                        renderer.DrawPoint(p1, pointColor);
                    }

                    if (options.show_contact_normal)
                    {
                        const Vec3 p2 = p1 + manifold.normal * 0.18f;
                        renderer.DrawLine(p1, p2, normalColor);
                    }
                }
            }
        }
    }

    renderer.FlushPoints(false);
    renderer.FlushLines(false);

    demo->Render();
    renderer.EndFrame();
}

void Game::UpdateInput()
{
    if (Input::IsKeyPressed(GLFW_KEY_R)) RestartDemo();
    if (Input::IsKeyPressed(GLFW_KEY_PAGE_DOWN)) PrevDemo();
    if (Input::IsKeyPressed(GLFW_KEY_PAGE_UP)) NextDemo();
    demo->UpdateInput();
}

void Game::UpdateUI()
{
    std::vector<DemoFrame>& demoFrames = GetDemoFrames();
    World& world = demo->GetWorld();
    WorldSettings& settings = demo->GetWorldSettings();

    // ImGui::ShowDemoWindow();

    ImGui::SetNextWindowPos({ 2, 2 }, ImGuiCond_Once, { 0.0f, 0.0f });
    ImGui::SetNextWindowSize({ 240, 470 }, ImGuiCond_Once);

    static bool collapsed = false;
    if (Input::IsKeyPressed(GLFW_KEY_GRAVE_ACCENT))
    {
        collapsed = !collapsed;
        ImGui::SetNextWindowCollapsed(collapsed, ImGuiCond_None);
    }

    ImGuiWindowFlags flags = ImGuiWindowFlags_AlwaysAutoResize | ImGuiWindowFlags_NoMove;
    if (ImGui::Begin("Muli Engine", NULL, flags))
    {
        if (ImGui::BeginTabBar("TabBar", ImGuiTabBarFlags_AutoSelectNewTabs))
        {
            if (ImGui::BeginTabItem("Control"))
            {
                ImGui::BeginDisabled(options.pause);
                if (ImGui::Button("Pause")) options.pause = true;
                ImGui::EndDisabled();

                ImGui::SameLine();
                ImGui::BeginDisabled(!options.pause);
                ImGui::PushButtonRepeat(true);
                if (ImGui::Button("Step")) options.step = true;
                ImGui::PopButtonRepeat();
                ImGui::EndDisabled();

                ImGui::SameLine();
                ImGui::BeginDisabled(!options.pause);
                if (ImGui::Button("Start")) options.pause = false;
                ImGui::EndDisabled();

                ImGui::SameLine();
                if (ImGui::Button("Restart")) InitDemo(demoIndex);

                static int32 fps = GetFrameRate();
                static int32 ups = GetUpdateRate();

                ImGui::SetNextItemWidth(120);
                if (ImGui::SliderInt("Frame Rate", &fps, 30, 300))
                {
                    SetFrameRate(fps);
                }

                ImGui::SetNextItemWidth(120);
                if (ImGui::SliderInt("Update Rate", &ups, 30, 300))
                {
                    SetUpdateRate(ups);
                }

                ImGui::Separator();
                ImGui::Text("%.3f ms/frame (%.1f FPS)", 1000.0f / ImGui::GetIO().Framerate, ImGui::GetIO().Framerate);
                ImGui::Separator();

                ImGui::SetNextItemOpen(false, ImGuiCond_Once);
                if (ImGui::CollapsingHeader("Debug Options"))
                {
                    ImGui::ColorEdit3("Sky Color", &skyColor.x);
                    ImGui::SliderFloat("Sky Intensity", &skyIntensity, 0.0f, 4.0f, "%.2f");
                    ImGui::ColorEdit3("Light Color", &lightColor.x);
                    ImGui::DragFloat3("Light Direction", &lightDirection.x, 0.01f, -1.0f, 1.0f, "%.2f");
                    ImGui::SliderFloat("Light Intensity", &lightIntensity, 0.0f, 10.0f, "%.2f");
                    ImGui::Checkbox("Show Profiler", &options.show_profiler);
                    ImGui::Checkbox("Camera Reset", &options.reset_camera);
                    ImGui::Checkbox("Colorize Island", &options.colorize_island);
                    ImGui::Checkbox("Draw Joint", &options.draw_joint);
                    const char* bodyDrawModes[] = { "Solid", "Solid + Wireframe", "Solid Wireframe", "Wireframe", "None" };
                    int32 bodyDrawMode = (int32)options.body_draw_mode;
                    ImGui::Text("Draw Body");
                    ImGui::SetNextItemWidth(140.0f);
                    if (ImGui::Combo("##Draw Body", &bodyDrawMode, bodyDrawModes, IM_ARRAYSIZE(bodyDrawModes)))
                    {
                        options.body_draw_mode = (BodyDrawMode)bodyDrawMode;
                    }
                    ImGui::Checkbox("Show BVH", &options.show_bvh);
                    ImGui::Checkbox("Show AABB", &options.show_aabb);
                    ImGui::Checkbox("Show Contact Point", &options.show_contact_point);
                    ImGui::Checkbox("Show Contact Normal", &options.show_contact_normal);
                }

                ImGui::SetNextItemOpen(true, ImGuiCond_Once);
                if (ImGui::CollapsingHeader("Simulation Settings"))
                {
                    ImGui::Text("Solver Iterations");
                    ImGui::SetNextItemWidth(120);
                    ImGui::SliderInt("Velocity", &settings.velocity_iterations, 0, 50);
                    ImGui::SetNextItemWidth(120);
                    ImGui::SliderInt("Position", &settings.position_iterations, 0, 50);
                    ImGui::SetNextItemWidth(120);
                    if (ImGui::SliderInt("Workers", &workerCount, 1, int32(std::max(1u, std::thread::hardware_concurrency()))))
                    {
                        RecreateThreadPool();
                    }
                    if (ImGui::Checkbox("Apply Gravity", &settings.apply_gravity))
                    {
                        world.Awake();
                    }
                    ImGui::Checkbox("Sleeping", &settings.sleeping);
                }
                ImGui::Separator();
                ImGui::Text("%zu.%s", demoIndex, demoFrames[demoIndex].name);
                ImGui::EndTabItem();
            }

            if (ImGui::BeginTabItem("Demos"))
            {
                if (ImGui::BeginChild("##demos", ImVec2{ -FLT_MIN, 24 * ImGui::GetTextLineHeightWithSpacing() }, false))
                {
                    const char* currentCategory = nullptr;
                    bool categoryOpen = false;

                    for (int32 i = 0; i < (int32)demoCount; ++i)
                    {
                        if (currentCategory == nullptr || std::strcmp(currentCategory, demoFrames[i].category) != 0)
                        {
                            currentCategory = demoFrames[i].category;

                            bool hasSelectedDemo = false;
                            for (int32 j = i; j < (int32)demoCount; ++j)
                            {
                                if (std::strcmp(currentCategory, demoFrames[j].category) != 0)
                                {
                                    break;
                                }

                                if (demoIndex == (size_t)j)
                                {
                                    hasSelectedDemo = true;
                                    break;
                                }
                            }

                            if (hasSelectedDemo)
                            {
                                ImGui::SetNextItemOpen(true, ImGuiCond_Once);
                            }

                            ImGui::PushID(currentCategory);
                            categoryOpen = ImGui::CollapsingHeader(currentCategory, ImGuiTreeNodeFlags_SpanAvailWidth);
                            ImGui::PopID();
                        }

                        if (!categoryOpen)
                        {
                            continue;
                        }

                        bool selected = demoIndex == (size_t)i;
                        ImGui::PushID(i);
                        ImGui::Bullet();
                        ImGui::SameLine();
                        if (ImGui::Selectable(demoFrames[i].name, selected))
                        {
                            InitDemo((size_t)i);
                        }
                        ImGui::PopID();
                    }

                    ImGui::EndChild();
                }
                ImGui::EndTabItem();
            }

            ImGui::EndTabBar();
        }
    }

    if (!collapsed && ImGui::IsWindowCollapsed()) collapsed = true;
    if (collapsed && !ImGui::IsWindowCollapsed()) collapsed = false;

    ImGui::End();

    if (options.show_profiler)
    {
        ImGui::SetNextWindowSize({ 600.0f, 240.0f }, ImGuiCond_Once);
        if (ImGui::Begin("Profiler", &options.show_profiler, ImGuiWindowFlags_None))
        {
            if (ImGui::BeginTabBar("ProfilerTabs"))
            {
                if (ImGui::BeginTabItem("Frame Profile"))
                {
                    int count = (int)(profileWriteIndex - profileReadIndex);
                    float broadPhaseValues[profile_capacity]{};
                    float narrowPhaseValues[profile_capacity]{};
                    float buildIslandsValues[profile_capacity]{};
                    float integrateVelocitiesValues[profile_capacity]{};
                    float prepareConstraintsValues[profile_capacity]{};
                    float warmStartValues[profile_capacity]{};
                    float solveVelocitiesValues[profile_capacity]{};
                    float integratePositionsValues[profile_capacity]{};
                    float solvePositionsValues[profile_capacity]{};
                    float sleepAndSyncValues[profile_capacity]{};
                    float finalizeValues[profile_capacity]{};
                    float postSolveValues[profile_capacity]{};

                    for (int32 i = 0; i < count; ++i)
                    {
                        int32 index = (int32)((profileReadIndex + i) & (profile_capacity - 1));
                        const WorldProfile& profile = profiles[index];
                        broadPhaseValues[i] = profile.broad_phase;
                        narrowPhaseValues[i] = profile.narrow_phase;
                        buildIslandsValues[i] = profile.build_islands;
                        integrateVelocitiesValues[i] = profile.integrate_velocities;
                        prepareConstraintsValues[i] = profile.prepare_constraints;
                        warmStartValues[i] = profile.warm_start;
                        solveVelocitiesValues[i] = profile.solve_velocities;
                        integratePositionsValues[i] = profile.integrate_positions;
                        solvePositionsValues[i] = profile.solve_positions;
                        sleepAndSyncValues[i] = profile.sleep_and_sync;
                        finalizeValues[i] = profile.finalize;
                        postSolveValues[i] = profile.post_solve;
                    }

                    ProfileGraphEntry entries[] = {
                        { "Broad phase", color::broad_phase, broadPhaseValues },
                        { "Narrow phase", color::narrow_phase, narrowPhaseValues },
                        { "Build islands", color::build_islands, buildIslandsValues },
                        { "Integrate velocities", color::integrate_velocities, integrateVelocitiesValues },
                        { "Prepare constraints", color::prepare_constraints, prepareConstraintsValues },
                        { "Warm start", color::warm_start, warmStartValues },
                        { "Solve velocities", color::solve_velocities, solveVelocitiesValues },
                        { "Integrate positions", color::integrate_positions, integratePositionsValues },
                        { "Solve positions", color::solve_positions, solvePositionsValues },
                        { "Sleep and sync", color::sleep_and_sync, sleepAndSyncValues },
                        { "Finalize", color::finalize, finalizeValues },
                        { "Post solve", color::post_solve, postSolveValues },
                    };

                    DrawProfileGraph(
                        "", { -1.0f, -1.0f }, profile_capacity, count, entries, (int32)(sizeof(entries) / sizeof(entries[0])),
                        true, profileShowOverlay, profileShowAverage
                    );

                    ImGui::SetCursorPosX(ImGui::GetCursorPosX() + 32.0f);
                    ImGui::Checkbox("Stop", &profileStopped);
                    ImGui::SameLine();
                    ImGui::Checkbox("Overlay", &profileShowOverlay);
                    ImGui::SameLine();
                    if (ImGui::Button("Clear")) ClearProfiles();
                    ImGui::SameLine();
                    ImGui::Checkbox("Average", &profileShowAverage);
                    ImGui::EndTabItem();
                }

                if (ImGui::BeginTabItem("World Status"))
                {
                    int32 staticBodyCount = 0;
                    int32 dynamicBodyCount = 0;
                    int32 kinematicBodyCount = 0;
                    int32 awakeDynamicBodyCount = 0;
                    int32 enabledBodyCount = 0;
                    int32 colliderCount = 0;

                    for (Body* body = world.GetBodyList(); body; body = body->GetNext())
                    {
                        colliderCount += body->GetColliderCount();
                        enabledBodyCount += body->IsEnabled();

                        if (body->IsStatic())
                        {
                            ++staticBodyCount;
                        }
                        else if (body->IsKinematic())
                        {
                            ++kinematicBodyCount;
                        }
                        else
                        {
                            ++dynamicBodyCount;
                            awakeDynamicBodyCount += body->IsSleeping() == false;
                        }
                    }

                    int32 constraintCounts[constraint_color_count];
                    int32 normalConstraintCount = 0;
                    int32 totalConstraintCount = 0;
                    int32 colorCount = 0;
                    for (int32 i = 0; i < constraint_color_count; ++i)
                    {
                        constraintCounts[i] = world.GetConstraintCount(i);
                        if (constraintCounts[i] > 0)
                        {
                            ++colorCount;
                            totalConstraintCount += constraintCounts[i];
                        }
                        if (i != constraint_overflow_index)
                        {
                            normalConstraintCount += constraintCounts[i];
                        }
                    }

                    if (ImGui::BeginTable("CountersLayout", 2, ImGuiTableFlags_SizingFixedFit))
                    {
                        ImGui::TableSetupColumn("Body", ImGuiTableColumnFlags_WidthFixed, 220.0f);
                        ImGui::TableSetupColumn("Simulation", ImGuiTableColumnFlags_WidthStretch);
                        ImGui::TableHeadersRow();
                        ImGui::TableNextRow();
                        ImGui::TableNextColumn();
                        ImGui::Text("Bodies: %d", world.GetBodyCount());
                        // ImGui::Text("Enabled: %d", enabledBodyCount);
                        ImGui::Text("Colliders: %d", colliderCount);
                        ImGui::Text("Static: %d", staticBodyCount);
                        ImGui::Text("Kinematic: %d", kinematicBodyCount);
                        ImGui::Text("Dynamic: %d", dynamicBodyCount);
                        ImGui::Text("Awake Dynamic: %d", awakeDynamicBodyCount);
                        ImGui::Text("Sleeping Dynamic: %d", world.GetSleepingBodyCount());

                        ImGui::TableNextColumn();
                        ImGui::Text("Contacts: %d", world.GetContactCount());
                        ImGui::Text("Joints: %d", world.GetJointCount());
                        ImGui::Text("Awake Islands: %d", world.GetAwakeIslandCount());
                        ImGui::Text("Steps: %lld", world.GetStepIndex());

                        ImGui::Spacing();
                        ImGui::Text("%d Constraints across %d color batches", totalConstraintCount, colorCount);

                        float barWidth = ImGui::GetContentRegionAvail().x;
                        float barHeight = 2.0f * ImGui::GetFontSize();
                        ImVec2 barMin = ImGui::GetCursorScreenPos();
                        ImGui::InvisibleButton("ConstraintColors", ImVec2{ barWidth, barHeight });

                        ImDrawList* drawList = ImGui::GetWindowDrawList();
                        drawList->AddRectFilled(
                            barMin, ImVec2{ barMin.x + barWidth, barMin.y + barHeight }, IM_COL32(40, 40, 40, 255)
                        );

                        float x = barMin.x;
                        if (normalConstraintCount > 0)
                        {
                            float invTotal = 1.0f / (float)normalConstraintCount;
                            for (int32 i = 0; i < constraint_overflow_index; ++i)
                            {
                                int32 count = constraintCounts[i];
                                if (count == 0)
                                {
                                    continue;
                                }

                                float segmentWidth = barWidth * count * invTotal;
                                uint32 color = color::RGBToHex(g_colors2[i]);
                                ImU32 imColor = IM_COL32((color >> 16) & 0xff, (color >> 8) & 0xff, color & 0xff, 255);
                                drawList->AddRectFilled(
                                    ImVec2{ x, barMin.y }, ImVec2{ x + segmentWidth, barMin.y + barHeight }, imColor
                                );
                                x += segmentWidth;
                            }
                        }

                        if (ImGui::IsItemHovered() && normalConstraintCount > 0)
                        {
                            float mouseX = ImGui::GetIO().MousePos.x;
                            float segmentX = barMin.x;
                            for (int32 i = 0; i < constraint_overflow_index; ++i)
                            {
                                int32 count = constraintCounts[i];
                                float segmentWidth = barWidth * count / (float)normalConstraintCount;
                                if (count > 0 && mouseX < segmentX + segmentWidth)
                                {
                                    ImGui::SetTooltip("Color %d: %d Constraints", i, count);
                                    break;
                                }
                                segmentX += segmentWidth;
                            }
                        }

                        int32 overflowCount = constraintCounts[constraint_overflow_index];
                        float overflowFraction = totalConstraintCount > 0 ? overflowCount / (float)totalConstraintCount : 0.0f;
                        char overflowText[32];
                        std::snprintf(overflowText, sizeof(overflowText), "Overflow %d", overflowCount);
                        ImGui::PushStyleColor(ImGuiCol_PlotHistogram, IM_COL32(220, 60, 60, 255));
                        ImGui::ProgressBar(overflowFraction, ImVec2{ -FLT_MIN, 0.0f }, overflowText);
                        ImGui::PopStyleColor();
                        ImGui::EndTable();
                    }

                    ImGui::EndTabItem();
                }

                ImGui::EndTabBar();
            }
        }
        ImGui::End();
    }

    ImGui::SetNextWindowPos({ 0.0f, Window::Get()->GetWindowSize().y }, ImGuiCond_Always, { 0.0f, 1.0f });
    ImGui::Begin(
        "Body info", NULL,
        ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoInputs | ImGuiWindowFlags_AlwaysAutoResize |
            ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_NoBackground
    );
    Body* targetBody = demo->GetTargetBody();
    if (targetBody)
    {
        ImGui::PushStyleColor(ImGuiCol_Text, ImVec4{ 12 / 255.0f, 11 / 255.0f, 14 / 255.0f, 1.0f });
        ImGui::Text("Mass: %.2f", targetBody->GetMass());
        Vec3 pos = targetBody->GetPosition();
        Vec3 rot = targetBody->GetRotation().ToEuler() * Vec3(inv_pi * 180);
        ImGui::Text("Pos: %.2f, %.2f, %.2f", pos.x, pos.y, pos.z);
        ImGui::Text("Rot: %.2f, %.2f, %.2f", rot.x, rot.y, rot.z);
        ImGui::PopStyleColor();
    }
    ImGui::End();

    demo->UpdateUI();
}

void Game::InitDemo(size_t index)
{
    std::vector<DemoFrame>& demoFrames = GetDemoFrames();

    if (index >= demoFrames.size())
    {
        return;
    }

    bool restoreSettings = demo && demoIndex == index;
    bool restoreCameraPosition = demo && demoIndex == index && !options.reset_camera;
    Camera previousCamera;
    WorldSettings previousSettings;

    if (restoreSettings)
    {
        previousSettings = demo->GetWorldSettings();
        previousSettings.thread_pool = GetThreadPool();
        previousCamera = demo->GetCamera();
    }

    delete demo;
    demo = nullptr;
    renderer.ClearMeshCache();

    time = 0.0f;
    demoIndex = index;
    demo = demoFrames[demoIndex].createFunction(*this);
    demo->GetWorldSettings().thread_pool = GetThreadPool();
    // ClearProfiles();

    if (restoreSettings)
    {
        demo->GetWorldSettings() = previousSettings;
        demo->GetWorldSettings().thread_pool = GetThreadPool();
    }

    if (restoreCameraPosition)
    {
        demo->GetCamera() = previousCamera;
    }

    demo->dt = fixedDeltaTime;
    options.step = false;
}

void Game::ClearProfiles()
{
    std::memset(profiles, 0, profile_capacity * sizeof(WorldProfile));
    profileReadIndex = 0;
    profileWriteIndex = 0;
}

void Game::RecreateThreadPool()
{
    workerCount = std::max(workerCount, 1);
    ThreadPool::global_thread_pool = std::make_unique<ThreadPool>(workerCount);

    if (demo)
    {
        demo->GetWorldSettings().thread_pool = ThreadPool::global_thread_pool.get();
    }
}

} // namespace muli3
