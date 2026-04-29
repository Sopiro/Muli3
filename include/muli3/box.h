#pragma once

#include "settings.h"
#include "shape.h"

namespace muli3
{

class Box final : public Shape
{
public:
    Box(float width, float height, float depth, float radius = default_radius, const Transform& transform = identity);
    Box(const Vec3& size, float radius = default_radius, const Transform& transform = identity);
    Box(float size, float radius = default_radius, const Transform& transform = identity);
    Box(const Box& other, const Transform& transform);

    void ComputeMass(float density, MassData* outMassData) const override;
    void ComputeAABB(const Transform& transform, AABB* outAABB) const override;

    int32 GetVertexCount() const override;
    Vec3 GetVertex(int32 id) const override;

    int32 GetSupport(const Vec3& localDir) const override;
    bool GetFace(int32 id, const Transform& transform, Face* outFace) const override;
    bool GetFeaturedFace(const Transform& transform, const Vec3& dir, Face* outFace) const override;

    bool TestPoint(const Transform& transform, const Vec3& q) const override;
    Vec3 GetClosestPoint(const Transform& transform, const Vec3& q) const override;

    const Vec3& GetHalfExtents() const;

private:
    Vec3 vertices[8];
    Vec3 normals[6];
    Vec3 halfExtents{ 0.5f, 0.5f, 0.5f };
};

inline Box::Box(const Vec3& size, float radius, const Transform& transform)
    : Box(size.x, size.y, size.z, radius, transform)
{
}

inline Box::Box(float size, float radius, const Transform& transform)
    : Box(size, size, size, radius, transform)
{
}

inline int32 Box::GetVertexCount() const
{
    return 8;
}

inline Vec3 Box::GetVertex(int32 id) const
{
    MuliAssert(0 <= id && id < 8);
    return vertices[id];
}

inline const Vec3& Box::GetHalfExtents() const
{
    return halfExtents;
}

} // namespace muli3
