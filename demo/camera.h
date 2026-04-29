#pragma once

namespace muli3
{

class Camera
{
public:
    void Reset();
    void Update(float dt, bool captureMouse);
    bool UpdateInput(float dt);

    Mat4 GetViewMatrix() const;
    Mat4 GetCameraMatrix() const;
    Mat4 GetProjectionMatrix(float aspectRatio) const;

    Vec3 GetPosition() const;
    Vec3 GetForward() const;
    Vec3 GetRight() const;
    Vec3 GetUp() const;
    void SetPosition(const Vec3& position);
    void SetRotation(float yaw, float pitch);
    void SetEulerAngles(const Vec3& eulerAngles);

    Vec3 position{ 0.0f, 3.0f, 8.0f };
    Vec3 rotation{ DegToRad(-18.0f), 0.0f, 0.0f };
    Vec3 scale{ 1.0f, 1.0f, 1.0f };

    Vec3 velocity{ 0.0f, 0.0f, 0.0f };
    float speed = 1.0f;
    float sensitivity = 18.0f;
    float damping = 12.0f;
    float fovDegrees = 60.0f;
};

inline Vec3 Camera::GetPosition() const
{
    return position;
}

inline void Camera::SetPosition(const Vec3& newPosition)
{
    position = newPosition;
}

inline void Camera::SetRotation(float yaw, float pitch)
{
    rotation.x = DegToRad(pitch);
    rotation.y = DegToRad(yaw + 90.0f);
    rotation.z = 0.0f;
}

inline void Camera::SetEulerAngles(const Vec3& newEulerAngles)
{
    rotation = newEulerAngles;
}

} // namespace muli3
