#pragma once

#include "shape.h"

namespace muli3
{

class MeshShape : public Shape
{
public:
    MeshShape(std::span<const Vec3> vertices, std::span<const int32> indices, const Transform& transform = identity);
    MeshShape(const MeshShape& other, const Transform& transform);

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
    bool ShapeCast(
        const Transform& transform,
        const Shape* shape,
        const Transform& shapeTransform,
        const Vec3& translation,
        ShapeCastOutput* output
    ) const;

    int32 GetTriangleCount() const;
    std::span<const Vec3> GetVertices() const;
    std::span<const int32> GetIndices() const;
    const AABB& GetLocalBounds() const;
    uint8 GetActiveEdgeBits(int32 triangle) const;
    void GetTriangle(int32 triangle, Vec3* a, Vec3* b, Vec3* c) const;

    template <typename Callback>
    void Query(const AABB& localAABB, Callback&& callback) const;

    uint64 GetHash() const;

private:
    struct BVHPrimitive
    {
        AABB bounds;
        Vec3 centroid;
        int32 triangle;
    };

    struct BVHNode
    {
        AABB bounds;

        union
        {
            int32 child2;
            int32 triangleOffset;
        };

        int32 triangleCount;
        uint8 axis;
    };

    std::vector<Vec3> vertices;
    std::vector<int32> indices;
    std::vector<uint8> activeEdges;
    std::vector<int32> bvhTriangles;
    std::vector<BVHNode> nodes;
    AABB localBounds;

    uint64 hash;

    void Build();
    int32 BuildNode(std::vector<BVHPrimitive>* primitives, int32 begin, int32 end);
};

inline int32 MeshShape::GetVertexCount() const
{
    return int32(vertices.size());
}

inline Vec3 MeshShape::GetVertex(int32 index) const
{
    MuliAssert(0 <= index && index < int32(vertices.size()));
    return vertices[index];
}

inline int32 MeshShape::GetVertexIndex(int32 index) const
{
    MuliNotUsed(index);
    MuliAssert(false);
    return 0;
}

inline int32 MeshShape::GetSupport(const Vec3& localDir) const
{
    MuliNotUsed(localDir);
    MuliAssert(false);
    return 0;
}

inline Face MeshShape::GetFeaturedFace(const Transform& transform, const Vec3& dir) const
{
    MuliNotUsed(transform);
    MuliNotUsed(dir);
    MuliAssert(false);
    return {};
}

inline int32 MeshShape::GetTriangleCount() const
{
    return int32(indices.size()) / 3;
}

inline std::span<const Vec3> MeshShape::GetVertices() const
{
    return vertices;
}

inline std::span<const int32> MeshShape::GetIndices() const
{
    return indices;
}

inline const AABB& MeshShape::GetLocalBounds() const
{
    return localBounds;
}

inline uint8 MeshShape::GetActiveEdgeBits(int32 triangle) const
{
    MuliAssert(0 <= triangle && triangle < GetTriangleCount());
    return activeEdges[triangle];
}

inline void MeshShape::GetTriangle(int32 triangle, Vec3* a, Vec3* b, Vec3* c) const
{
    MuliAssert(0 <= triangle && triangle < GetTriangleCount());
    int32 index = triangle * 3;
    *a = vertices[indices[index]];
    *b = vertices[indices[index + 1]];
    *c = vertices[indices[index + 2]];
}

inline uint64 MeshShape::GetHash() const
{
    return hash;
}

template <typename Callback>
inline void MeshShape::Query(const AABB& localAABB, Callback&& callback) const
{
    // A balanced BVH needs one pending node per level, so this fixed stack covers meshes far larger than practical memory limits.
    int32 stack[64];
    int32 count = 0;
    stack[count++] = 0;

    while (count > 0)
    {
        int32 nodeIndex = stack[--count];
        const BVHNode& node = nodes[nodeIndex];
        if (node.bounds.TestOverlap(localAABB) == false)
        {
            continue;
        }

        if (node.triangleCount > 0)
        {
            for (int32 i = 0; i < node.triangleCount; ++i)
            {
                int32 triangle = bvhTriangles[node.triangleOffset + i];
                Vec3 a, b, c;
                GetTriangle(triangle, &a, &b, &c);
                callback(triangle, a, b, c);
            }
        }
        else
        {
            MuliAssert(count + 2 <= int32(std::size(stack)));
            stack[count++] = node.child2;
            stack[count++] = nodeIndex + 1;
        }
    }
}

} // namespace muli3
