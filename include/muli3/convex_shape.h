#pragma once

#include "shape.h"

namespace muli3
{

// Represents a convex polyhedron.
class ConvexShape : public Shape
{
public:
    ConvexShape(std::span<const Vec3> vertices, float radius = default_radius, const Transform& transform = identity);
    ConvexShape(
        std::span<const Vec3> vertices,
        std::span<const int32> indices,
        std::span<const Face> faces,
        float radius = default_radius,
        const Transform& transform = identity
    );
    ConvexShape(const ConvexShape& other, const Transform& transform);

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
    std::span<const int32> GetIndices() const;
    std::span<const Face> GetFaces() const;

    uint64 GetHash() const;

private:
    std::vector<Vec3> vertices;
    std::vector<int32> indices;
    std::vector<Face> faces;

    uint64 hash;

    void Initialize();

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

inline int32 ConvexShape::GetVertexIndex(int32 index) const
{
    MuliAssert(0 <= index && index < int32(indices.size()));
    return indices[index];
}

inline std::span<const Vec3> ConvexShape::GetVertices() const
{
    return vertices;
}

inline std::span<const int32> ConvexShape::GetIndices() const
{
    return indices;
}

inline std::span<const Face> ConvexShape::GetFaces() const
{
    return faces;
}

inline uint64 ConvexShape::GetHash() const
{
    return hash;
}

} // namespace muli3
