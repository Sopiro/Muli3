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
    Vec3 GetVertex(int32 index) const;
    int32 GetVertexIndex(int32 index) const;
    int32 GetSupport(const Vec3& localDir) const;
    Face GetFeaturedFace(const Transform& transform, const Vec3& dir) const;

    bool TestPoint(const Transform& transform, const Vec3& q) const;
    Vec3 GetClosestPoint(const Transform& transform, const Vec3& q) const;
    bool RayCast(const Transform& transform, const RayCastInput& input, RayCastOutput* output) const;
};

inline int32 SphereShape::GetVertexCount() const
{
    return 1;
}

inline Vec3 SphereShape::GetVertex(int32 index) const
{
    MuliAssert(index == 0);
    MuliNotUsed(index);
    return center;
}

inline int32 SphereShape::GetVertexIndex(int32 index) const
{
    MuliAssert(index == 0);
    MuliNotUsed(index);
    return 0;
}

inline int32 SphereShape::GetSupport(const Vec3& localDir) const
{
    MuliNotUsed(localDir);
    return 0;
}

inline Face SphereShape::GetFeaturedFace(const Transform& transform, const Vec3& dir) const
{
    MuliNotUsed(transform);
    MuliNotUsed(dir);

    Face f{};
    f.vertexStart = 0;
    f.vertexCount = 1;
    f.normal = Vec3::zero;
    return f;
}

} // namespace muli3
