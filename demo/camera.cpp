#include "camera.h"

namespace muli3
{

void Camera::Reset()
{
    position = Vec3{ 0.0f, 3.0f, 8.0f };
    yawDegrees = -90.0f;
    pitchDegrees = -18.0f;
}

void Camera::Update(float dt, bool captureMouse)
{
    if (captureMouse)
    {
        const Vec2 mouseDelta = Input::GetMouseDelta();
        yawDegrees += mouseDelta.x * mouseSensitivity;
        pitchDegrees -= mouseDelta.y * mouseSensitivity;
        pitchDegrees = Clamp(pitchDegrees, -89.0f, 89.0f);
    }

    Vec3 moveDirection{ 0.0f, 0.0f, 0.0f };
    const Vec3 forward = GetForward();
    const Vec3 right = GetRight();
    float speed = moveSpeed;

    if (Input::IsKeyDown(GLFW_KEY_LEFT_SHIFT) || Input::IsKeyDown(GLFW_KEY_RIGHT_SHIFT))
    {
        speed *= 3.0f;
    }
    if (Input::IsKeyDown(GLFW_KEY_LEFT_ALT) || Input::IsKeyDown(GLFW_KEY_RIGHT_ALT))
    {
        speed *= 0.25f;
    }

    if (Input::IsKeyDown(GLFW_KEY_W))
    {
        moveDirection += forward;
    }
    if (Input::IsKeyDown(GLFW_KEY_S))
    {
        moveDirection -= forward;
    }
    if (Input::IsKeyDown(GLFW_KEY_A))
    {
        moveDirection -= right;
    }
    if (Input::IsKeyDown(GLFW_KEY_D))
    {
        moveDirection += right;
    }
    if (Input::IsKeyDown(GLFW_KEY_SPACE))
    {
        moveDirection += worldUp;
    }
    if (Input::IsKeyDown(GLFW_KEY_LEFT_CONTROL))
    {
        moveDirection -= worldUp;
    }

    if (moveDirection.LengthSquared() > epsilon)
    {
        moveDirection.Normalize();
        position += moveDirection * (speed * dt);
    }
}

Mat4 Camera::GetViewMatrix() const
{
    return Mat4::LookAt(position, position + GetForward(), worldUp);
}

Mat4 Camera::GetProjectionMatrix(float aspectRatio) const
{
    return Mat4::Perspective(DegToRad(fovDegrees), aspectRatio, 0.1f, 500.0f);
}

Vec3 Camera::GetForward() const
{
    const float yaw = DegToRad(yawDegrees);
    const float pitch = DegToRad(pitchDegrees);
    Vec3 forward{
        std::cos(yaw) * std::cos(pitch),
        std::sin(pitch),
        std::sin(yaw) * std::cos(pitch),
    };
    forward.Normalize();
    return forward;
}

Vec3 Camera::GetRight() const
{
    Vec3 right = Cross(GetForward(), worldUp);
    right.Normalize();
    return right;
}

Vec3 Camera::GetUp() const
{
    Vec3 up = Cross(GetRight(), GetForward());
    up.Normalize();
    return up;
}

} // namespace muli3
