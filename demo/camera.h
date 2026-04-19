#pragma once

#include "common.h"
#include "input.h"

namespace muli3
{

class Camera
{
public:
    void Reset();
    void Update(float dt, bool captureMouse);

    Mat4 GetViewMatrix() const;
    Mat4 GetProjectionMatrix(float aspectRatio) const;

    Vec3 GetPosition() const;
    Vec3 GetForward() const;
    Vec3 GetRight() const;
    Vec3 GetUp() const;

private:
    Vec3 position{ 0.0f, 3.0f, 8.0f };
    float yawDegrees = -90.0f;
    float pitchDegrees = -18.0f;
    float moveSpeed = 8.0f;
    float mouseSensitivity = 0.12f;
    float fovDegrees = 60.0f;
    Vec3 worldUp{ 0.0f, 1.0f, 0.0f };
};

inline Vec3 Camera::GetPosition() const
{
    return position;
}

} // namespace muli3
