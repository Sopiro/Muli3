#pragma once

#include "shape.h"

namespace muli3
{

class Sphere final : public Shape
{
public:
    explicit Sphere(float radius);

    const Vec3& GetCenterOfMass() const override;
    Mat3 ComputeLocalInertiaTensor(float mass) const override;
    AABB ComputeAABB(const Transform& transform) const override;

    float GetRadius() const;

private:
    float radius = 0.5f;
    Vec3 centerOfMass{ 0.0f, 0.0f, 0.0f };
};

inline const Vec3& Sphere::GetCenterOfMass() const
{
    return centerOfMass;
}

inline float Sphere::GetRadius() const
{
    return radius;
}

} // namespace muli3
