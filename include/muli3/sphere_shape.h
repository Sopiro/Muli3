#pragma once

#include "shape.h"

namespace muli3
{

class SphereShape : public Shape
{
public:
    explicit SphereShape(float radius, const Transform& transform = identity);
    SphereShape(const SphereShape& other, const Transform& transform);

    void ComputeMass(float density, MassData* outMassData) const;
    void ComputeAABB(const Transform& transform, AABB* outAABB) const;

    int32 GetVertexCount() const;
    Vec3 GetVertex(int32 id) const;
    int32 GetSupport(const Vec3& localDir) const;
    Face GetFeaturedFace(const Transform& transform, const Vec3& dir) const;

    bool TestPoint(const Transform& transform, const Vec3& q) const;
    Vec3 GetClosestPoint(const Transform& transform, const Vec3& q) const;
    bool RayCast(const Transform& transform, const RayCastInput& input, RayCastOutput* output) const;
};

inline SphereShape::SphereShape(const SphereShape& other, const Transform& transform)
    : SphereShape(other.radius, Transform{ Mul(transform, other.center) })
{
}

inline int32 SphereShape::GetVertexCount() const
{
    return 1;
}

inline Vec3 SphereShape::GetVertex(int32 id) const
{
    MuliAssert(id == 0);
    MuliNotUsed(id);
    return center;
}

inline int32 SphereShape::GetSupport(const Vec3& localDir) const
{
    MuliNotUsed(localDir);
    return 0;
}

inline Face SphereShape::GetFeaturedFace(const Transform& transform, const Vec3& dir) const
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
