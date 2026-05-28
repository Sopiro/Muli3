#pragma once

#include "shape.h"

namespace muli3
{

class CapsuleShape : public Shape
{
public:
    CapsuleShape(float height, float radius, const Transform& transform = identity);
    CapsuleShape(const Vec3& p1, const Vec3& p2, float radius, const Transform& transform = identity);
    CapsuleShape(const CapsuleShape& other, const Transform& transform);

    void ComputeMass(float density, MassData* outMassData) const;
    void ComputeAABB(const Transform& transform, AABB* outAABB) const;

    int32 GetVertexCount() const;
    Vec3 GetVertex(int32 id) const;
    int32 GetSupport(const Vec3& localDir) const;
    Face GetFeaturedFace(const Transform& transform, const Vec3& dir) const;

    bool TestPoint(const Transform& transform, const Vec3& q) const;
    Vec3 GetClosestPoint(const Transform& transform, const Vec3& q) const;
    bool RayCast(const Transform& transform, const RayCastInput& input, RayCastOutput* output) const;

    float GetHeight() const;
    const Vec3& GetVertexA() const;
    const Vec3& GetVertexB() const;

private:
    Vec3 va, vb;
};

inline CapsuleShape::CapsuleShape(const CapsuleShape& other, const Transform& transform)
    : CapsuleShape(other.va, other.vb, other.radius, transform)
{
}

inline int32 CapsuleShape::GetVertexCount() const
{
    return 2;
}

inline Vec3 CapsuleShape::GetVertex(int32 id) const
{
    MuliAssert(id == 0 || id == 1);
    return id == 0 ? va : vb;
}

inline int32 CapsuleShape::GetSupport(const Vec3& localDir) const
{
    Vec3 e = vb - va;
    return Dot(e, localDir) > 0.0f ? 1 : 0;
}

inline float CapsuleShape::GetHeight() const
{
    return Dist(va, vb);
}

inline const Vec3& CapsuleShape::GetVertexA() const
{
    return va;
}

inline const Vec3& CapsuleShape::GetVertexB() const
{
    return vb;
}

} // namespace muli3
