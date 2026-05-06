#pragma once

#include "shape.h"

namespace muli3
{

class Capsule : public Shape
{
public:
    Capsule(float height, float radius, const Transform& transform = identity);
    Capsule(const Vec3& p1, const Vec3& p2, float radius, bool resetPosition = false, const Transform& transform = identity);
    Capsule(const Capsule& other, const Transform& transform);

    void ComputeMass(float density, MassData* outMassData) const override;
    void ComputeAABB(const Transform& transform, AABB* outAABB) const override;

    int32 GetVertexCount() const override;
    Vec3 GetVertex(int32 id) const override;
    int32 GetSupport(const Vec3& localDir) const override;
    Face GetFeaturedFace(const Transform& transform, const Vec3& dir) const override;

    bool TestPoint(const Transform& transform, const Vec3& q) const override;
    Vec3 GetClosestPoint(const Transform& transform, const Vec3& q) const override;
    bool RayCast(const Transform& transform, const RayCastInput& input, RayCastOutput* output) const override;

    float GetHeight() const;
    const Vec3& GetVertexA() const;
    const Vec3& GetVertexB() const;

private:
    Vec3 va, vb;
};

inline Capsule::Capsule(const Capsule& other, const Transform& transform)
    : Capsule(other.va, other.vb, other.radius, false, transform)
{
}

inline int32 Capsule::GetVertexCount() const
{
    return 2;
}

inline Vec3 Capsule::GetVertex(int32 id) const
{
    MuliAssert(id == 0 || id == 1);
    return id == 0 ? va : vb;
}

inline int32 Capsule::GetSupport(const Vec3& localDir) const
{
    Vec3 e = vb - va;
    return Dot(e, localDir) > 0.0f ? 1 : 0;
}

inline float Capsule::GetHeight() const
{
    return Dist(va, vb);
}

inline const Vec3& Capsule::GetVertexA() const
{
    return va;
}

inline const Vec3& Capsule::GetVertexB() const
{
    return vb;
}

} // namespace muli3
