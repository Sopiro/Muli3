// IWYU pragma: always_keep
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

class CapsuleShape : public Shape
{
public:
    CapsuleShape(float height, float radius, const Transform& transform = identity);
    CapsuleShape(const Vec3& p1, const Vec3& p2, float radius, const Transform& transform = identity);
    CapsuleShape(const CapsuleShape& other, const Transform& transform);

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

    float GetHeight() const;
    const Vec3& GetVertexA() const;
    const Vec3& GetVertexB() const;

private:
    Vec3 va, vb;
};

inline int32 CapsuleShape::GetVertexCount() const
{
    return 2;
}

inline Vec3 CapsuleShape::GetVertex(int32 index) const
{
    MuliAssert(index == 0 || index == 1);
    return index == 0 ? va : vb;
}

inline int32 CapsuleShape::GetVertexIndex(int32 index) const
{
    MuliAssert(index == 0 || index == 1);
    return index;
}

inline int32 CapsuleShape::GetSupport(const Vec3& localDir) const
{
    Vec3 e = vb - va;
    return Dot(e, localDir) > 0.0f ? 1 : 0;
}

inline Face CapsuleShape::GetFeaturedFace(const Transform& transform, const Vec3& dir) const
{
    MuliNotUsed(transform);

    Face face{};
    face.vertexStart = 0;
    face.vertexCount = 2;
    face.normal = dir;
    return face;
}

inline float CapsuleShape::GetHeight() const
{
    return Dist(va, vb);
}

inline const Vec3& CapsuleShape::GetVertexA() const
{
    return va;
}

inline const Vec3& CapsuleShape::GetVertexB() const
{
    return vb;
}

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

    uint64 GetHash() const;

private:
    std::vector<Vec3> vertices;
    std::vector<int32> indices;
    Vec3 normal;

    uint64 hash;

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

inline uint64 PolygonShape::GetHash() const
{
    return hash;
}

class HeightFieldShape : public Shape
{
public:
    HeightFieldShape(
        int32 sampleCountX,
        int32 sampleCountZ,
        std::span<const float> heightSamples,
        float cellSizeX = 1.0f,
        float cellSizeZ = 1.0f,
        const Vec3& offset = Vec3::zero,
        int32 blockSize = 4,
        const Transform& transform = identity
    );
    HeightFieldShape(const HeightFieldShape& other, const Transform& transform);

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

    int32 GetSampleCountX() const;
    int32 GetSampleCountZ() const;

    int32 GetCellCountX() const;
    int32 GetCellCountZ() const;
    float GetCellSizeX() const;
    float GetCellSizeZ() const;

    const Vec3& GetOffset() const;
    const AABB& GetLocalBounds() const;

    float GetHeight(int32 x, int32 z) const;
    Vec3 GetPosition(int32 x, int32 z) const;
    uint8 GetActiveEdgeBits(int32 x, int32 z, int32 triangle) const;

    // 00-----10
    // |  \  1 |
    // | 0  \  |
    // 01---- 11
    void GetTriangle(int32 x, int32 z, int32 triangle, Vec3* a, Vec3* b, Vec3* c) const;

    void Query(
        const AABB& localAABB,
        std::function<void(int32 x, int32 z, int32 triangle, const Vec3& a, const Vec3& b, const Vec3& c)> callback
    ) const;

    uint64 GetHash() const;

private:
    int32 sampleCountX;
    int32 sampleCountZ;
    float cellSizeX;
    float cellSizeZ;

    Vec3 offset; // Origin offset

    float minHeight;
    float maxHeight;

    // Height range of a group of cells
    struct Block
    {
        float minHeight;
        float maxHeight;
    };

    int32 blockSize;
    int32 blockCountX;
    int32 blockCountZ;
    std::vector<float> heights;
    std::vector<Block> blocks;

    // Active edge bits
    std::vector<uint8> activeEdges;

    AABB localBounds;

    uint64 hash;

    void Build();
    AABB GetBlockAABB(int32 bx, int32 bz) const;
};

inline int32 HeightFieldShape::GetVertexCount() const
{
    return sampleCountX * sampleCountZ;
}

inline Vec3 HeightFieldShape::GetVertex(int32 index) const
{
    MuliAssert(0 <= index && index < sampleCountX * sampleCountZ);
    int32 x = index % sampleCountX;
    int32 z = index / sampleCountX;
    return GetPosition(x, z);
}

inline int32 HeightFieldShape::GetVertexIndex(int32 index) const
{
    MuliNotUsed(index);
    MuliAssert(false);
    return 0;
}

inline int32 HeightFieldShape::GetSupport(const Vec3& localDir) const
{
    MuliNotUsed(localDir);
    MuliAssert(false);
    return 0;
}

inline Face HeightFieldShape::GetFeaturedFace(const Transform& transform, const Vec3& dir) const
{
    MuliNotUsed(transform);
    MuliNotUsed(dir);
    MuliAssert(false);
    return {};
}

inline int32 HeightFieldShape::GetSampleCountX() const
{
    return sampleCountX;
}

inline int32 HeightFieldShape::GetSampleCountZ() const
{
    return sampleCountZ;
}

inline int32 HeightFieldShape::GetCellCountX() const
{
    return sampleCountX - 1;
}

inline int32 HeightFieldShape::GetCellCountZ() const
{
    return sampleCountZ - 1;
}

inline float HeightFieldShape::GetCellSizeX() const
{
    return cellSizeX;
}

inline float HeightFieldShape::GetCellSizeZ() const
{
    return cellSizeZ;
}

inline const Vec3& HeightFieldShape::GetOffset() const
{
    return offset;
}

inline const AABB& HeightFieldShape::GetLocalBounds() const
{
    return localBounds;
}

inline float HeightFieldShape::GetHeight(int32 x, int32 z) const
{
    MuliAssert(0 <= x && x < sampleCountX);
    MuliAssert(0 <= z && z < sampleCountZ);
    return heights[z * sampleCountX + x];
}

inline Vec3 HeightFieldShape::GetPosition(int32 x, int32 z) const
{
    return offset + Vec3{ x * cellSizeX, GetHeight(x, z), z * cellSizeZ };
}

inline uint8 HeightFieldShape::GetActiveEdgeBits(int32 x, int32 z, int32 triangle) const
{
    MuliAssert(0 <= x && x < GetCellCountX());
    MuliAssert(0 <= z && z < GetCellCountZ());
    MuliAssert(triangle == 0 || triangle == 1);

    return activeEdges[(z * GetCellCountX() + x) * 2 + triangle];
}

inline void HeightFieldShape::GetTriangle(int32 x, int32 z, int32 triangle, Vec3* a, Vec3* b, Vec3* c) const
{
    MuliAssert(0 <= x && x < GetCellCountX());
    MuliAssert(0 <= z && z < GetCellCountZ());
    MuliAssert(triangle == 0 || triangle == 1);

    Vec3 p00 = GetPosition(x, z);
    Vec3 p10 = GetPosition(x + 1, z);
    Vec3 p01 = GetPosition(x, z + 1);
    Vec3 p11 = GetPosition(x + 1, z + 1);

    *a = p00;
    if (triangle == 0)
    {
        *b = p01;
        *c = p11;
    }
    else
    {
        *b = p11;
        *c = p10;
    }
}

inline uint64 HeightFieldShape::GetHash() const
{
    return hash;
}

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

inline void Shape::ComputeMass(float density, MassData* outMassData) const
{
    Dispatch([&](auto shape) { shape->ComputeMass(density, outMassData); });
}

inline void Shape::ComputeAABB(const Transform& transform, AABB* outAABB) const
{
    Dispatch([&](auto shape) { shape->ComputeAABB(transform, outAABB); });
}

inline int32 Shape::GetVertexCount() const
{
    return Dispatch([&](auto shape) { return shape->GetVertexCount(); });
}

inline Vec3 Shape::GetVertex(int32 index) const
{
    return Dispatch([&](auto shape) { return shape->GetVertex(index); });
}

inline int32 Shape::GetVertexIndex(int32 index) const
{
    return Dispatch([&](auto shape) { return shape->GetVertexIndex(index); });
}

inline int32 Shape::GetSupport(const Vec3& localDir) const
{
    return Dispatch([&](auto shape) { return shape->GetSupport(localDir); });
}

inline Face Shape::GetFeaturedFace(const Transform& transform, const Vec3& dir) const
{
    return Dispatch([&](auto shape) { return shape->GetFeaturedFace(transform, dir); });
}

inline bool Shape::TestPoint(const Transform& transform, const Vec3& q) const
{
    return Dispatch([&](auto shape) { return shape->TestPoint(transform, q); });
}

inline Vec3 Shape::GetClosestPoint(const Transform& transform, const Vec3& q) const
{
    return Dispatch([&](auto shape) { return shape->GetClosestPoint(transform, q); });
}

inline bool Shape::RayCast(const Transform& transform, const RayCastInput& input, RayCastOutput* output) const
{
    return Dispatch([&](auto shape) { return shape->RayCast(transform, input, output); });
}

} // namespace muli3
