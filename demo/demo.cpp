#include "game.h"
#include "window.h"

namespace muli3
{

Demo::Demo(Game& game)
    : game{ game }
    , renderer{ game.GetRenderer() }
    , options{ game.GetDebugOptions() }
{
    screenBounds = Window::Get()->GetWindowSize();
    settings.thread_pool = game.GetThreadPool();
    settings.world_bounds.min.y = -30;
    world = new World(settings);

    camera.Reset();
    camera.position = Vec3{ 0.0f, 5.0f, 10.0f };
    camera.rotation = Vec3{ DegToRad(-20.0f), 0.0f, 0.0f };
    dt = game.GetFixedDeltaTime();
}

Demo::~Demo()
{
    cursorJoint = nullptr;
    delete world;
}

void Demo::UpdateInput()
{
    FindTargetBody();
    EnableKeyboardShortcut();
    EnableBodyCreate();

    bool handledGrab = EnableBodyGrab();
    if (!handledGrab)
    {
        EnableCameraControl();
    }
}

void Demo::Step()
{
    if (options.pause)
    {
        if (options.step)
        {
            options.step = false;
            world->Step(dt);
        }
    }
    else
    {
        world->Step(dt);
    }
}

void Demo::FindTargetBody()
{
    targetBody = nullptr;
    targetCollider = nullptr;
    targetPoint = Vec3::zero;
    cursorPos = Input::GetMousePosition();
    screenBounds = Window::Get()->GetWindowSize();

    if (Window::Get()->GetCursorHidden() || ImGui::GetIO().WantCaptureMouse)
    {
        return;
    }

    Ray ray = GetMouseRay();
    world->RayCastClosest(ray.o, ray.o + ray.d * 500.0f, 0.0f, [&](Collider* collider, Vec3 point, Vec3 normal, float fraction) {
        MuliNotUsed(normal);
        MuliNotUsed(fraction);

        targetCollider = collider;
        targetBody = collider->GetBody();
        targetPoint = point;
    });
}

void Demo::EnableKeyboardShortcut()
{
    if (ImGui::GetIO().WantCaptureKeyboard || Window::Get()->GetCursorHidden())
    {
        return;
    }

    if (Input::IsKeyPressed(GLFW_KEY_J)) options.draw_joint = !options.draw_joint;
    if (Input::IsKeyPressed(GLFW_KEY_O))
    {
        options.body_draw_mode = (BodyDrawMode)((options.body_draw_mode + 1) % body_draw_mode_count);
    }
    if (Input::IsKeyPressed(GLFW_KEY_L)) options.colorize_island = !options.colorize_island;
    if (Input::IsKeyPressed(GLFW_KEY_B)) options.show_aabb = !options.show_aabb;
    if (Input::IsKeyPressed(GLFW_KEY_V)) options.show_bvh = !options.show_bvh;
    if (Input::IsKeyPressed(GLFW_KEY_P)) options.show_contact_point = !options.show_contact_point;
    if (Input::IsKeyPressed(GLFW_KEY_N)) options.show_contact_normal = !options.show_contact_normal;
    if (Input::IsKeyPressed(GLFW_KEY_C)) options.reset_camera = !options.reset_camera;
    if (Input::IsKeyPressed(GLFW_KEY_F1)) options.show_profiler = !options.show_profiler;
    if (Input::IsKeyPressed(GLFW_KEY_Q)) options.pause = !options.pause;
    if (Input::IsKeyDown(GLFW_KEY_RIGHT) || Input::IsKeyPressed(GLFW_KEY_E)) options.step = true;

    if (Input::IsKeyPressed(GLFW_KEY_G))
    {
        settings.apply_gravity = !settings.apply_gravity;
        world->Awake();
    }
}

void Demo::EnableBodyCreate()
{
    if (ImGui::GetIO().WantCaptureKeyboard)
    {
        throwCooldown = 0.0f;
        return;
    }

    bool shift = Input::IsKeyDown(GLFW_KEY_LEFT_SHIFT);
    bool alt = Input::IsKeyDown(GLFW_KEY_LEFT_ALT);
    bool ctrl = Input::IsKeyDown(GLFW_KEY_LEFT_CONTROL);
    bool createSphere = Input::IsKeyDown(GLFW_KEY_1) || Input::IsKeyDown(GLFW_KEY_KP_1);
    bool createCapsule = Input::IsKeyDown(GLFW_KEY_2) || Input::IsKeyDown(GLFW_KEY_KP_2);
    bool createBox = Input::IsKeyDown(GLFW_KEY_3) || Input::IsKeyDown(GLFW_KEY_KP_3);

    if (!createSphere && !createCapsule && !createBox)
    {
        throwCooldown = 0.0f;
        return;
    }

    if (shift)
    {
        throwCooldown -= dt;
        if (throwCooldown > 0.0f)
        {
            return;
        }
    }
    else if (!Input::IsKeyPressed(GLFW_KEY_1) && !Input::IsKeyPressed(GLFW_KEY_KP_1) && !Input::IsKeyPressed(GLFW_KEY_2) &&
             !Input::IsKeyPressed(GLFW_KEY_KP_2) && !Input::IsKeyPressed(GLFW_KEY_3) && !Input::IsKeyPressed(GLFW_KEY_KP_3))
    {
        return;
    }

    Camera& cam = GetCamera();
    Vec3 forward = cam.GetForward();
    Vec3 position = cam.GetPosition() + forward * 1.4f;
    Transform transform{ position, Quat::FromEuler(cam.rotation) };
    Body* body = nullptr;

    if (createSphere)
    {
        body = world->CreateSphere(0.25f, transform);
    }
    if (createCapsule)
    {
        body = world->CreateCapsule(0.6f, 0.2f, transform);
    }
    if (createBox)
    {
        body = world->CreateBox(0.45f, transform);
    }

    if (body)
    {
        body->SetLinearVelocity(forward * 18.0f);
        body->SetGyroscopicTorqueEnabled(!alt);
    }

    throwCooldown = 0.03f;
}

bool Demo::EnableBodyGrab()
{
    if (Window::Get()->GetCursorHidden() || ImGui::GetIO().WantCaptureMouse)
    {
        if (cursorJoint && Input::IsMouseReleased(GLFW_MOUSE_BUTTON_LEFT))
        {
            world->Destroy(cursorJoint);
            cursorJoint = nullptr;
        }

        return false;
    }

    if (!IsGrabJointActive())
    {
        cursorJoint = nullptr;
    }

    if (targetBody && Input::IsMousePressed(GLFW_MOUSE_BUTTON_LEFT))
    {
        if (targetBody->GetType() == Body::dynamic_body)
        {
            targetBody->Awake();
            cursorJoint = world->CreateGrabJoint(targetBody, targetPoint, targetPoint, 4.0f, 0.5f);
            cursorJoint->OnDestroy = this;
            grabDepth = Dot(targetPoint - camera.GetPosition(), camera.GetForward());
        }
    }
    if (targetBody && (Input::IsMousePressed(GLFW_MOUSE_BUTTON_MIDDLE) || Input::IsKeyPressed(GLFW_KEY_F)))
    {
        targetBody->DestroyCollider(targetCollider);
        targetCollider = nullptr;

        if (targetBody->GetColliderCount() == 0)
        {
            if (cursorJoint && cursorJoint->GetBodyA() == targetBody)
            {
                cursorJoint = nullptr;
            }

            world->Destroy(targetBody);
            targetBody = nullptr;
            return false;
        }
    }

    if (cursorJoint)
    {
        Vec3 target;
        if (GetMouseWorldPointOnGrabPlane(&target))
        {
            cursorJoint->GetBodyA()->Awake();
            cursorJoint->SetTarget(target);
        }

        if (Input::IsMouseReleased(GLFW_MOUSE_BUTTON_LEFT))
        {
            world->Destroy(cursorJoint);
            cursorJoint = nullptr;
        }
        else if (Input::IsMousePressed(GLFW_MOUSE_BUTTON_RIGHT))
        {
            cursorJoint = nullptr;
            return true;
        }
    }

    return cursorJoint != nullptr;
}

void Demo::EnableCameraControl()
{
    Window* window = Window::Get();

    if (!window->GetCursorHidden() && !ImGui::GetIO().WantCaptureMouse && Input::IsMousePressed(GLFW_MOUSE_BUTTON_RIGHT))
    {
        window->SetCursorHidden(true);
        ImGui::GetIO().ConfigFlags |= ImGuiConfigFlags_NoMouse;
    }
    else if (window->GetCursorHidden() && Input::IsMouseReleased(GLFW_MOUSE_BUTTON_RIGHT))
    {
        window->SetCursorHidden(false);
        ImGui::GetIO().ConfigFlags &= ~ImGuiConfigFlags_NoMouse;
    }

    camera.Update(dt, window->GetCursorHidden());
}

bool Demo::IsGrabJointActive() const
{
    if (cursorJoint == nullptr)
    {
        return false;
    }

    for (const Joint* joint = world->GetJoints(); joint; joint = joint->GetNext())
    {
        if (joint == cursorJoint)
        {
            return true;
        }
    }

    return false;
}

Ray Demo::GetMouseRay() const
{
    Vec2 windowSize = Window::Get()->GetWindowSize();
    Vec2 mouse = Input::GetMousePosition();

    float x = 2.0f * mouse.x / windowSize.x - 1.0f;
    float y = 1.0f - 2.0f * mouse.y / windowSize.y;

    Vec4 clipNear{ x, y, -1.0f, 1.0f };
    Vec4 clipFar{ x, y, 1.0f, 1.0f };

    float aspectRatio = windowSize.y > 0.0f ? windowSize.x / windowSize.y : 1.0f;
    Mat4 invViewProjection = (camera.GetProjectionMatrix(aspectRatio) * camera.GetViewMatrix()).GetInverse();

    Vec4 worldNear4 = invViewProjection * clipNear;
    Vec4 worldFar4 = invViewProjection * clipFar;

    Vec3 worldNear = Vec3{ worldNear4.x, worldNear4.y, worldNear4.z } / worldNear4.w;
    Vec3 worldFar = Vec3{ worldFar4.x, worldFar4.y, worldFar4.z } / worldFar4.w;

    return Ray{ worldNear, Normalize(worldFar - worldNear) };
}

bool Demo::GetMouseWorldPointOnGrabPlane(Vec3* point) const
{
    Ray ray = GetMouseRay();

    Vec3 planeNormal = camera.GetForward();
    float denominator = Dot(ray.d, planeNormal);
    if (Abs(denominator) <= epsilon)
    {
        return false;
    }

    Vec3 planePoint = camera.GetPosition() + planeNormal * grabDepth;
    float t = Dot(planePoint - ray.o, planeNormal) / denominator;
    if (t < 0.0f)
    {
        return false;
    }

    *point = ray.o + ray.d * t;
    return true;
}

} // namespace muli3
