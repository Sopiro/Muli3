#pragma once

#include "shape.h"

namespace muli3
{

class Sphere final : public Shape
{
public:
    explicit Sphere(float radius, const Transform& transform = identity);
    Sphere(const Sphere& other, const Transform& transform);

    void ComputeMass(float density, MassData* outMassData) const override;
    void ComputeAABB(const Transform& transform, AABB* outAABB) const override;
    Mat3 ComputeLocalInertiaTensor(float mass) const override;

    int32 GetVertexCount() const override;
    Vec3 GetVertex(int32 id) const override;
    int32 GetSupport(const Vec3& localDir) const override;

    bool TestPoint(const Transform& transform, const Vec3& q) const override;
    Vec3 GetClosestPoint(const Transform& transform, const Vec3& q) const override;
};

inline Sphere::Sphere(const Sphere& other, const Transform& transform)
    : Sphere(other.radius, Transform{ Mul(transform, other.center) })
{
}

inline int32 Sphere::GetVertexCount() const
{
    return 1;
}

inline Vec3 Sphere::GetVertex(int32 id) const
{
    MuliAssert(id == 0);
    MuliNotUsed(id);
    return center;
}

inline int32 Sphere::GetSupport(const Vec3& localDir) const
{
    MuliNotUsed(localDir);
    return 0;
}

} // namespace muli3
