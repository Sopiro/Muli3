#pragma once

#include "shape.h"

namespace muli3
{

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
    Vec3 GetVertex(int32 id) const;
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

    void Build();
    AABB GetBlockAABB(int32 bx, int32 bz) const;
};

inline int32 HeightFieldShape::GetVertexCount() const
{
    return sampleCountX * sampleCountZ;
}

inline Vec3 HeightFieldShape::GetVertex(int32 id) const
{
    MuliAssert(0 <= id && id < sampleCountX * sampleCountZ);
    int32 x = id % sampleCountX;
    int32 z = id / sampleCountX;
    return GetPosition(x, z);
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

} // namespace muli3
