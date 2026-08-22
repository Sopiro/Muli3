#pragma once

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

    const Vec3& GetHalfExtents() const;
    const Quat& GetRotation() const;

private:
    Vec3 halfExtents;
    Quat rotation;
};

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

inline int32 BoxShape::GetVertexIndex(int32 index) const
{
    constexpr int32 indices[] = {
        0, 4, 6, 2, // -x
        1, 3, 7, 5, // +x
        0, 1, 5, 4, // -y
        2, 6, 7, 3, // +y
        0, 2, 3, 1, // -z
        4, 5, 7, 6, // +z
    };

    MuliAssert(0 <= index && index < 24);
    return indices[index];
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
