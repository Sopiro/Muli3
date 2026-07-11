#pragma once

#include "shape.h"

namespace muli3
{

// Represents a convex polygon on a plane.
// Vertices must be coplanar and arranged in boundary order. Duplicate vertices,
// collinear consecutive edges, non-convex vertices, and counts that do not fit
// in Face::vertexCount result in undefined behavior.
class PolygonShape : public Shape
{
public:
    PolygonShape(std::span<const Vec3> vertices, float radius = default_radius, const Transform& transform = identity);
    PolygonShape(const PolygonShape& other, const Transform& transform);

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

    std::span<const Vec3> GetVertices() const;
    const Vec3& GetNormal() const;

private:
    std::vector<Vec3> vertices;
    std::vector<int32> indices;
    Vec3 normal;

    Vec3 GetClosestPointLocal(const Vec3& q) const;
};

inline int32 PolygonShape::GetVertexCount() const
{
    return int32(vertices.size());
}

inline Vec3 PolygonShape::GetVertex(int32 index) const
{
    MuliAssert(0 <= index && index < int32(vertices.size()));
    return vertices[index];
}

inline int32 PolygonShape::GetVertexIndex(int32 index) const
{
    MuliAssert(0 <= index && index < int32(indices.size()));
    return indices[index];
}

inline std::span<const Vec3> PolygonShape::GetVertices() const
{
    return vertices;
}

inline const Vec3& PolygonShape::GetNormal() const
{
    return normal;
}

} // namespace muli3
