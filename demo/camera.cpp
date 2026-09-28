#include "camera.h"
#include "input.h"
#include "window.h"

namespace muli3
{

void Camera::Reset()
{
    position = Vec3::zero;
    rotation = Vec3::zero;
    scale = Vec3(1);
    velocity = Vec3::zero;
}

void Camera::Update(float dt, bool captureMouse)
{
    MuliNotUsed(captureMouse);
    UpdateInput(dt);
}

bool Camera::UpdateInput(float dt)
{
    bool captureMouse = Window::Get()->GetCursorHidden();
    bool moved = false;

    Vec3 accel = Vec3::zero;

    if (Input::IsKeyDown(GLFW_KEY_W)) accel.z -= 1.0f;
    if (Input::IsKeyDown(GLFW_KEY_S)) accel.z += 1.0f;
    if (Input::IsKeyDown(GLFW_KEY_A)) accel.x -= 1.0f;
    if (Input::IsKeyDown(GLFW_KEY_D)) accel.x += 1.0f;
    if (Input::IsKeyDown(GLFW_KEY_SPACE)) accel.y += 1.0f;
    if (Input::IsKeyDown(GLFW_KEY_LEFT_CONTROL)) accel.y -= 1.0f;

    if (Length2(accel) > epsilon)
    {
        accel.Normalize();
    }

    float cameraSpeed = speed;
    if (Input::IsKeyDown(GLFW_KEY_LEFT_SHIFT) || Input::IsKeyDown(GLFW_KEY_RIGHT_SHIFT))
    {
        cameraSpeed *= 3.0f;
    }
    if (Input::IsKeyDown(GLFW_KEY_LEFT_ALT) || Input::IsKeyDown(GLFW_KEY_RIGHT_ALT))
    {
        cameraSpeed *= 0.25f;
    }

    if (captureMouse)
    {
        Vec2 mouseDelta = Input::GetMouseDelta();
        rotation.y -= mouseDelta.x * DegToRad(sensitivity);
        rotation.x -= mouseDelta.y * DegToRad(sensitivity);
        if (mouseDelta.y != 0.0f)
        {
            rotation.x = Clamp(rotation.x, DegToRad(-89.0f), DegToRad(89.0f));
        }
        moved |= mouseDelta != Vec2::zero;
    }

    float cos = std::cos(rotation.y);
    float sin = std::sin(rotation.y);

    Vec3 targetVelocity = {
        cameraSpeed * (accel.x * cos + accel.z * sin),
        cameraSpeed * accel.y,
        cameraSpeed * (accel.x * -sin + accel.z * cos),
    };

    moved |= targetVelocity != Vec3::zero || velocity != Vec3::zero;

    // y = v_target - v and it decays as y * exp(-damping * t).
    // Integrating v(t) = v_target - y * exp(-damping * t) over dt gives
    // dp = v_target * dt - y * blend / damping.

    Vec3 y = targetVelocity - velocity;
    float blend = -std::expm1(-damping * dt); // 1 - exp(-damping * dt)
    velocity += y * blend;
    position += targetVelocity * dt - y * (blend / damping);

    return moved;
}

void Camera::LookAt(const Vec3& newPosition, const Vec3& target)
{
    Vec3 direction = target - newPosition;
    MuliAssert(Length2(direction) > 0.0f);

    float horizontalLength = std::sqrt(direction.x * direction.x + direction.z * direction.z);
    position = newPosition;
    rotation.x = std::atan2(direction.y, horizontalLength);
    if (horizontalLength > 0.0f)
    {
        rotation.y = std::atan2(-direction.x, -direction.z);
    }
    rotation.z = 0.0f;
}

Mat4 Camera::GetViewMatrix() const
{
    return GetCameraMatrix();
}

Mat4 Camera::GetCameraMatrix() const
{
    Mat4 m{ Vec4{ Vec3{ 1.0f, 1.0f, 1.0f } / scale, 1.0f } };
    m = MulT(Mat4(Quat::FromEuler(rotation), Vec3::zero), m);
    m = m.Translate(-position);
    return m;
}

Mat4 Camera::GetProjectionMatrix(float aspectRatio) const
{
    return Mat4::Perspective(DegToRad(fovDegrees), aspectRatio, 0.1f, 500.0f);
}

Vec3 Camera::GetForward() const
{
    Vec3 forward = Quat::FromEuler(rotation).Rotate(-z_axis);
    forward.Normalize();
    return forward;
}

Vec3 Camera::GetRight() const
{
    Vec3 right = Quat::FromEuler(rotation).Rotate(x_axis);
    right.Normalize();
    return right;
}

Vec3 Camera::GetUp() const
{
    Vec3 up = Quat::FromEuler(rotation).Rotate(y_axis);
    up.Normalize();
    return up;
}

} // namespace muli3
