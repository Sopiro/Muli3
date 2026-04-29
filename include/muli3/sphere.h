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

    int32 GetVertexCount() const override;
    Vec3 GetVertex(int32 id) const override;
    int32 GetSupport(const Vec3& localDir) const override;
    bool GetFace(int32 id, const Transform& transform, Face* outFace) const override;
    bool GetFeaturedFace(const Transform& transform, const Vec3& dir, Face* outFace) const override;

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

inline bool Sphere::GetFace(int32 id, const Transform& transform, Face* outFace) const
{
    MuliNotUsed(id);
    MuliNotUsed(transform);
    MuliNotUsed(outFace);
    return false;
}

inline bool Sphere::GetFeaturedFace(const Transform& transform, const Vec3& dir, Face* outFace) const
{
    MuliNotUsed(transform);
    MuliNotUsed(dir);
    MuliNotUsed(outFace);
    return false;
}

} // namespace muli3
