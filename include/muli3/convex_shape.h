#pragma once

#include "geometry.h"
#include "settings.h"
#include "shape.h"

namespace muli3
{

class ConvexShape : public Shape
{
public:
    ConvexShape(std::span<const Vec3> vertices, float radius = default_radius, const Transform& transform = identity);
    ConvexShape(
        std::span<const Vec3> vertices,
        std::span<const ConvexFace> faces,
        float radius = default_radius,
        const Transform& transform = identity
    );
    ConvexShape(const ConvexShape& other, const Transform& transform);

    void ComputeMass(float density, MassData* outMassData) const;
    void ComputeAABB(const Transform& transform, AABB* outAABB) const;

    int32 GetVertexCount() const;
    Vec3 GetVertex(int32 id) const;
    int32 GetSupport(const Vec3& localDir) const;
    Face GetFeaturedFace(const Transform& transform, const Vec3& dir) const;

    bool TestPoint(const Transform& transform, const Vec3& q) const;
    Vec3 GetClosestPoint(const Transform& transform, const Vec3& q) const;
    bool RayCast(const Transform& transform, const RayCastInput& input, RayCastOutput* output) const;

    std::span<const Vec3> GetVertices() const;
    std::span<const ConvexFace> GetFaces() const;
    std::span<const Vec3> GetFaceNormals() const;

private:
    std::vector<Vec3> vertices;
    std::vector<ConvexFace> faces;
    std::vector<Vec3> normals;

    bool TestPointLocal(const Vec3& q) const;
    Vec3 GetClosestPointLocal(const Vec3& q) const;
};

inline int32 ConvexShape::GetVertexCount() const
{
    return int32(vertices.size());
}

inline Vec3 ConvexShape::GetVertex(int32 id) const
{
    MuliAssert(0 <= id && id < int32(vertices.size()));
    return vertices[id];
}

inline std::span<const Vec3> ConvexShape::GetVertices() const
{
    return vertices;
}

inline std::span<const ConvexFace> ConvexShape::GetFaces() const
{
    return faces;
}

inline std::span<const Vec3> ConvexShape::GetFaceNormals() const
{
    return normals;
}

} // namespace muli3
