#pragma once

#include "shape.h"

namespace muli3
{

class TriangleShape : public Shape
{
public:
    TriangleShape(const Vec3& a, const Vec3& b, const Vec3& c);
    TriangleShape(const Vec3& a, const Vec3& b, const Vec3& c, float radius, const Transform& transform = identity);
    TriangleShape(const TriangleShape& other, const Transform& transform);

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

    const Vec3* GetVertices() const;
    const Vec3& GetNormal() const;

private:
    Vec3 vertices[3];
    Vec3 normal;
};

inline int32 TriangleShape::GetVertexCount() const
{
    return 3;
}

inline Vec3 TriangleShape::GetVertex(int32 index) const
{
    MuliAssert(0 <= index && index < 3);
    return vertices[index];
}

inline int32 TriangleShape::GetVertexIndex(int32 index) const
{
    constexpr int32 indices[] = { 0, 1, 2, 0, 2, 1 };

    MuliAssert(0 <= index && index < 6);
    return indices[index];
}

inline const Vec3* TriangleShape::GetVertices() const
{
    return vertices;
}

inline const Vec3& TriangleShape::GetNormal() const
{
    return normal;
}

} // namespace muli3
