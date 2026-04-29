#include "camera.h"
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

    Vec3 accel{ 0.0f, 0.0f, 0.0f };

    if (Input::IsKeyDown(GLFW_KEY_W)) accel.z -= 1.0f;
    if (Input::IsKeyDown(GLFW_KEY_S)) accel.z += 1.0f;
    if (Input::IsKeyDown(GLFW_KEY_A)) accel.x -= 1.0f;
    if (Input::IsKeyDown(GLFW_KEY_D)) accel.x += 1.0f;
    if (Input::IsKeyDown(GLFW_KEY_SPACE)) accel.y += 1.0f;
    if (Input::IsKeyDown(GLFW_KEY_LEFT_CONTROL) || Input::IsKeyDown(GLFW_KEY_C)) accel.y -= 1.0f;

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
        rotation.y -= mouseDelta.x * DegToRad(sensitivity) * dt;
        rotation.x -= mouseDelta.y * DegToRad(sensitivity) * dt;
        moved |= mouseDelta != Vec2::zero;
    }

    rotation.x = Clamp(rotation.x, DegToRad(-89.0f), DegToRad(89.0f));

    float cos = std::cos(rotation.y);
    float sin = std::sin(rotation.y);

    velocity.x += cameraSpeed * (accel.x * cos + accel.z * sin);
    velocity.z += cameraSpeed * (accel.x * -sin + accel.z * cos);
    velocity.y += cameraSpeed * accel.y;

    moved |= velocity != Vec3::zero;

    position += velocity * dt;
    velocity *= std::exp(-damping * dt);

    return moved;
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
