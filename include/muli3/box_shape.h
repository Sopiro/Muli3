#pragma once

#include "settings.h"
#include "shape.h"

namespace muli3
{

class BoxShape : public Shape
{
public:
    BoxShape(float width, float height, float depth, float radius = default_radius, const Transform& transform = identity);
    BoxShape(const Vec3& size, float radius = default_radius, const Transform& transform = identity);
    BoxShape(float size, float radius = default_radius, const Transform& transform = identity);
    BoxShape(const BoxShape& other, const Transform& transform);

    void ComputeMass(float density, MassData* outMassData) const override;
    void ComputeAABB(const Transform& transform, AABB* outAABB) const override;

    int32 GetVertexCount() const override;
    Vec3 GetVertex(int32 id) const override;
    int32 GetSupport(const Vec3& localDir) const override;
    Face GetFeaturedFace(const Transform& transform, const Vec3& dir) const override;

    bool TestPoint(const Transform& transform, const Vec3& q) const override;
    Vec3 GetClosestPoint(const Transform& transform, const Vec3& q) const override;
    bool RayCast(const Transform& transform, const RayCastInput& input, RayCastOutput* output) const override;

    const Vec3& GetHalfExtents() const;
    const Quat& GetRotation() const;

private:
    Vec3 halfExtents;
    Quat rotation;
};

inline BoxShape::BoxShape(const Vec3& size, float radius, const Transform& transform)
    : BoxShape(size.x, size.y, size.z, radius, transform)
{
}

inline BoxShape::BoxShape(float size, float radius, const Transform& transform)
    : BoxShape(size, size, size, radius, transform)
{
}

inline int32 BoxShape::GetVertexCount() const
{
    return 8;
}

inline Vec3 BoxShape::GetVertex(int32 id) const
{
    MuliAssert(0 <= id && id < 8);

    Vec3 localPoint{
        (id & 1) ? halfExtents.x : -halfExtents.x,
        (id & 2) ? halfExtents.y : -halfExtents.y,
        (id & 4) ? halfExtents.z : -halfExtents.z,
    };

    return center + rotation.Rotate(localPoint);
}

inline int32 BoxShape::GetSupport(const Vec3& localDir) const
{
    Vec3 dir = rotation.RotateInv(localDir);

    int32 id = 0;
    if (dir.x > 0.0f)
    {
        id |= 1;
    }
    if (dir.y > 0.0f)
    {
        id |= 2;
    }
    if (dir.z > 0.0f)
    {
        id |= 4;
    }

    return id;
}

inline const Vec3& BoxShape::GetHalfExtents() const
{
    return halfExtents;
}

inline const Quat& BoxShape::GetRotation() const
{
    return rotation;
}

} // namespace muli3
