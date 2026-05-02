#pragma once

#include "shape.h"

namespace muli3
{

class Sphere : public Shape
{
public:
    explicit Sphere(float radius, const Transform& transform = identity);
    Sphere(const Sphere& other, const Transform& transform);

    void ComputeMass(float density, MassData* outMassData) const override;
    void ComputeAABB(const Transform& transform, AABB* outAABB) const override;

    int32 GetVertexCount() const override;
    Vec3 GetVertex(int32 id) const override;
    int32 GetSupport(const Vec3& localDir) const override;
    Face GetFeaturedFace(const Transform& transform, const Vec3& dir) const override;

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

inline Face Sphere::GetFeaturedFace(const Transform& transform, const Vec3& dir) const
{
    MuliNotUsed(dir);

    Face f;
    f.count = 1;
    f.points[0].p = Mul(transform, center);
    f.points[0].id = 0;
    f.normal = Vec3::zero;
    return f;
}

} // namespace muli3
