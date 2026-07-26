#include "game.h"
#include "input.h"
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

    world = new World(&settings);

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
    if (options.pause && !options.step)
    {
        return;
    }

    options.step = false;
    world->Step(dt);
}

void Demo::FindTargetBody()
{
    targetBody = nullptr;
    targetCollider = nullptr;
    targetPoint = Vec3::zero;
    cursorPos = Input::GetMousePosition();
    screenBounds = Window::Get()->GetWindowSize();

    if (cursorJoint || Window::Get()->GetCursorHidden() || ImGui::GetIO().WantCaptureMouse)
    {
        return;
    }

    Ray ray = GetMouseRay();
    world->RayCastClosest(ray.o, ray.o + ray.d * 500.0f, [&](Collider* collider, Vec3 point, Vec3 normal, float fraction) {
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

    bool repeatCreate = Input::IsKeyDown(GLFW_KEY_LEFT_SHIFT);
    auto isCreateKey = [repeatCreate](int key, int keypad) {
        if (repeatCreate)
        {
            return Input::IsKeyDown(key) || Input::IsKeyDown(keypad);
        }

        return Input::IsKeyPressed(key) || Input::IsKeyPressed(keypad);
    };

    bool createSphere = isCreateKey(GLFW_KEY_1, GLFW_KEY_KP_1);
    bool createCapsule = isCreateKey(GLFW_KEY_2, GLFW_KEY_KP_2);
    bool createBox = isCreateKey(GLFW_KEY_3, GLFW_KEY_KP_3);
    bool createCylinder = isCreateKey(GLFW_KEY_4, GLFW_KEY_KP_4);

    if (!createSphere && !createCapsule && !createBox && !createCylinder)
    {
        throwCooldown = 0.0f;
        return;
    }

    if (repeatCreate)
    {
        throwCooldown -= dt;
        if (throwCooldown > 0.0f)
        {
            return;
        }
    }

    Vec3 forward = camera.GetForward();
    Vec3 position = camera.GetPosition() + forward * 1.4f;
    Vec3 velocity = forward * 18.0f;
    Body* body = nullptr;

    if (createSphere || createCapsule || createBox || createCylinder)
    {
        Transform transform{ position, Quat::FromEuler(camera.rotation) };
        if (createSphere)
        {
            body = world->CreateSphere(0.25f, transform);
        }
        else if (createCapsule)
        {
            body = world->CreateCapsule(0.6f, 0.2f, transform);
        }
        else if (createBox)
        {
            body = world->CreateBox(0.45f, transform);
        }
        else
        {
            body = world->CreateCylinder(0.45f, 0.25f, 0.25f, 16, transform);
        }

        body->SetLinearVelocity(velocity);
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
    bool cursorHidden = window->GetCursorHidden();

    if (!cursorHidden && !ImGui::GetIO().WantCaptureMouse && Input::IsMousePressed(GLFW_MOUSE_BUTTON_RIGHT))
    {
        window->SetCursorHidden(true);
        ImGui::GetIO().ConfigFlags |= ImGuiConfigFlags_NoMouse;
        cursorHidden = true;
    }
    else if (cursorHidden && Input::IsMouseReleased(GLFW_MOUSE_BUTTON_RIGHT))
    {
        window->SetCursorHidden(false);
        ImGui::GetIO().ConfigFlags &= ~ImGuiConfigFlags_NoMouse;
        cursorHidden = false;
    }

    camera.Update(game.GetDeltaTime(), cursorHidden);
}

Ray Demo::GetMouseRay() const
{
    float x = 2.0f * cursorPos.x / screenBounds.x - 1.0f;
    float y = 1.0f - 2.0f * cursorPos.y / screenBounds.y;

    Vec4 clipNear{ x, y, -1.0f, 1.0f };
    Vec4 clipFar{ x, y, 1.0f, 1.0f };

    float aspectRatio = screenBounds.y > 0.0f ? screenBounds.x / screenBounds.y : 1.0f;
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
