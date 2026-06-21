#include "muli3/height_field_shape.h"
#include "muli3/distance.h"
#include "muli3/parallel_for.h"
#include "muli3/settings.h"

namespace muli3
{

HeightFieldShape::HeightFieldShape(
    int32 inSampleCountX,
    int32 inSampleCountZ,
    std::span<const float> heightSamples,
    float inCellSizeX,
    float inCellSizeZ,
    const Vec3& inOffset,
    int32 inBlockSize,
    const Transform& transform
)
    : Shape(Shape::height_field, 0.0f)
    , sampleCountX{ inSampleCountX }
    , sampleCountZ{ inSampleCountZ }
    , cellSizeX{ inCellSizeX * Abs(transform.s.x) }
    , cellSizeZ{ inCellSizeZ * Abs(transform.s.z) }
    , offset{ Mul(transform, inOffset) }
    , blockSize{ inBlockSize }
    , heights{ heightSamples.begin(), heightSamples.end() }
{
    MuliAssert(sampleCountX > 1);
    MuliAssert(sampleCountZ > 1);
    MuliAssert(cellSizeX > 0.0f);
    MuliAssert(cellSizeZ > 0.0f);
    MuliAssert(blockSize > 0);
    MuliAssert(int32(heightSamples.size()) == sampleCountX * sampleCountZ);

    for (float& h : heights)
    {
        h *= Abs(transform.s.y);
    }

    center = offset + Vec3(0.5f * (sampleCountX - 1) * cellSizeX, 0.0f, 0.5f * (sampleCountZ - 1) * cellSizeZ);
    volume = 0.0f;

    Build();
}

HeightFieldShape::HeightFieldShape(const HeightFieldShape& other, const Transform& transform)
    : Shape(Shape::height_field, 0.0f)
    , sampleCountX{ other.sampleCountX }
    , sampleCountZ{ other.sampleCountZ }
    , cellSizeX{ other.cellSizeX * Abs(transform.s.x) }
    , cellSizeZ{ other.cellSizeZ * Abs(transform.s.z) }
    , offset{ Mul(transform, other.offset) }
    , blockSize{ other.blockSize }
    , heights{ other.heights }
{
    for (float& h : heights)
    {
        h *= Abs(transform.s.y);
    }

    center = offset + Vec3(0.5f * (sampleCountX - 1) * cellSizeX, 0.0f, 0.5f * (sampleCountZ - 1) * cellSizeZ);
    volume = 0.0f;

    Build();
}

void HeightFieldShape::Build()
{
    // Build a regular grid of height values

    minHeight = max_float;
    maxHeight = -max_float;
    for (float h : heights)
    {
        minHeight = Min(minHeight, h);
        maxHeight = Max(maxHeight, h);
    }

    const int32 cellCountX = sampleCountX - 1;
    const int32 cellCountZ = sampleCountZ - 1;

    blockCountX = (cellCountX + blockSize - 1) / blockSize;
    blockCountZ = (cellCountZ + blockSize - 1) / blockSize;
    blocks.resize(blockCountX * blockCountZ);

    int32 blockCount = blockCountX * blockCountZ;
    ParallelFor(0, blockCount, 32, [&](int32 begin, int32 end) {
        for (int32 i = begin; i < end; ++i)
        {
            int32 bx = i % blockCountX;
            int32 bz = i / blockCountX;

            int32 x0 = bx * blockSize;
            int32 z0 = bz * blockSize;
            int32 x1 = Min(x0 + blockSize, cellCountX);
            int32 z1 = Min(z0 + blockSize, cellCountZ);

            Block& block = blocks[i];
            block.minHeight = max_float;
            block.maxHeight = -max_float;

            for (int32 z = z0; z <= z1; ++z)
            {
                for (int32 x = x0; x <= x1; ++x)
                {
                    float h = GetHeight(x, z);
                    block.minHeight = Min(block.minHeight, h);
                    block.maxHeight = Max(block.maxHeight, h);
                }
            }
        }
    });

    localBounds.min = Vec3{
        offset.x,
        offset.y + minHeight,
        offset.z,
    };
    localBounds.max = Vec3{
        offset.x + (sampleCountX - 1) * cellSizeX,
        offset.y + maxHeight,
        offset.z + (sampleCountZ - 1) * cellSizeZ,
    };
}

void HeightFieldShape::ComputeMass(float density, MassData* outMassData) const
{
    MuliNotUsed(density);
    MuliAssert(outMassData != nullptr);

    outMassData->mass = 0.0f;
    outMassData->inertia = Mat3::zero;
    outMassData->centerOfMass = center;
}

void HeightFieldShape::ComputeAABB(const Transform& transform, AABB* outAABB) const
{
    MuliAssert(outAABB != nullptr);

    Vec3 corners[8] = {
        Mul(transform, Vec3(localBounds.min.x, localBounds.min.y, localBounds.min.z)),
        Mul(transform, Vec3(localBounds.max.x, localBounds.min.y, localBounds.min.z)),
        Mul(transform, Vec3(localBounds.min.x, localBounds.max.y, localBounds.min.z)),
        Mul(transform, Vec3(localBounds.max.x, localBounds.max.y, localBounds.min.z)),
        Mul(transform, Vec3(localBounds.min.x, localBounds.min.y, localBounds.max.z)),
        Mul(transform, Vec3(localBounds.max.x, localBounds.min.y, localBounds.max.z)),
        Mul(transform, Vec3(localBounds.min.x, localBounds.max.y, localBounds.max.z)),
        Mul(transform, Vec3(localBounds.max.x, localBounds.max.y, localBounds.max.z)),
    };

    Vec3 min = corners[0];
    Vec3 max = corners[0];
    for (int32 i = 1; i < 8; ++i)
    {
        min = Min(min, corners[i]);
        max = Max(max, corners[i]);
    }

    *outAABB = AABB{ min, max };
}

AABB HeightFieldShape::GetBlockAABB(int32 bx, int32 bz) const
{
    const Block& block = blocks[bz * blockCountX + bx];

    int32 x0 = bx * blockSize;
    int32 z0 = bz * blockSize;
    int32 x1 = Min(x0 + blockSize, GetCellCountX());
    int32 z1 = Min(z0 + blockSize, GetCellCountZ());

    return AABB{
        offset + Vec3{ x0 * cellSizeX, block.minHeight, z0 * cellSizeZ },
        offset + Vec3{ x1 * cellSizeX, block.maxHeight, z1 * cellSizeZ },
    };
}

void HeightFieldShape::Query(
    const AABB& localAABB,
    std::function<void(int32 x, int32 z, int32 triangle, const Vec3& a, const Vec3& b, const Vec3& c)> callback
) const
{
    const float cellCountX = sampleCountX - 1;
    const float cellCountZ = sampleCountZ - 1;

    // Clamp the query bounds to the height field in XZ
    AABB bounds = localAABB;
    bounds.min.x = Clamp(bounds.min.x, offset.x, offset.x + cellCountX * cellSizeX);
    bounds.max.x = Clamp(bounds.max.x, offset.x, offset.x + cellCountX * cellSizeX);
    bounds.min.z = Clamp(bounds.min.z, offset.z, offset.z + cellCountZ * cellSizeZ);
    bounds.max.z = Clamp(bounds.max.z, offset.z, offset.z + cellCountZ * cellSizeZ);

    // Convert the query bounds to cell and block ranges
    int32 minCellX = Clamp(int32(std::floor((bounds.min.x - offset.x) / cellSizeX)), 0, cellCountX - 1);
    int32 maxCellX = Clamp(int32(std::floor((bounds.max.x - offset.x) / cellSizeX)), 0, cellCountX - 1);
    int32 minCellZ = Clamp(int32(std::floor((bounds.min.z - offset.z) / cellSizeZ)), 0, cellCountZ - 1);
    int32 maxCellZ = Clamp(int32(std::floor((bounds.max.z - offset.z) / cellSizeZ)), 0, cellCountZ - 1);

    int32 minBlockX = minCellX / blockSize;
    int32 maxBlockX = maxCellX / blockSize;
    int32 minBlockZ = minCellZ / blockSize;
    int32 maxBlockZ = maxCellZ / blockSize;

    // Note it must be inclusive
    for (int32 bz = minBlockZ; bz <= maxBlockZ; ++bz)
    {
        for (int32 bx = minBlockX; bx <= maxBlockX; ++bx)
        {
            // Reject whole blocks whose height range misses the query
            if (GetBlockAABB(bx, bz).TestOverlap(localAABB) == false)
            {
                continue;
            }

            // Visit only the cells shared by the block and query range
            int32 x0 = Max(minCellX, bx * blockSize);
            int32 z0 = Max(minCellZ, bz * blockSize);
            int32 x1 = Min(maxCellX, Min((bx + 1) * blockSize - 1, cellCountX - 1));
            int32 z1 = Min(maxCellZ, Min((bz + 1) * blockSize - 1, cellCountZ - 1));

            for (int32 z = z0; z <= z1; ++z)
            {
                for (int32 x = x0; x <= x1; ++x)
                {
                    Vec3 a, b, c;
                    GetTriangle(x, z, 0, &a, &b, &c);
                    callback(x, z, 0, a, b, c);
                    GetTriangle(x, z, 1, &a, &b, &c);
                    callback(x, z, 1, a, b, c);
                }
            }
        }
    }
}

bool HeightFieldShape::TestPoint(const Transform& transform, const Vec3& q) const
{
    Vec3 localQ = MulT(transform, q);
    Vec3 closest = GetClosestPoint(transform, q);

    return Dist2(closest, q) <= Sqr(linear_slop) && localQ.y <= MulT(transform, closest).y + epsilon;
}

Vec3 HeightFieldShape::GetClosestPoint(const Transform& transform, const Vec3& q) const
{
    Vec3 localQ = MulT(transform, q);
    AABB localAABB{
        Vec3{ localQ.x - cellSizeX, offset.y + minHeight - 1.0f, localQ.z - cellSizeZ },
        Vec3{ localQ.x + cellSizeX, offset.y + maxHeight + 1.0f, localQ.z + cellSizeZ },
    };

    Vec3 closest = localQ;
    float minDistance2 = max_float;

    Query(localAABB, [&](int32, int32, int32, const Vec3& a, const Vec3& b, const Vec3& c) {
        Vec3 p = ClosestPointVsTriangle(localQ, a, b, c);
        float distance2 = Dist2(localQ, p);
        if (distance2 < minDistance2)
        {
            minDistance2 = distance2;
            closest = p;
        }
    });

    if (minDistance2 == max_float)
    {
        closest = Clamp(localQ, localBounds.min, localBounds.max);
    }

    return Mul(transform, closest);
}

bool HeightFieldShape::RayCast(const Transform& transform, const RayCastInput& input, RayCastOutput* output) const
{
    RayCastInput localInput = input;
    localInput.from = MulT(transform, input.from);
    localInput.to = MulT(transform, input.to);

    AABB localAABB{ Min(localInput.from, localInput.to), Max(localInput.from, localInput.to) };
    localAABB.min -= Vec3{ input.radius, input.radius, input.radius };
    localAABB.max += Vec3{ input.radius, input.radius, input.radius };

    bool hit = false;
    float bestFraction = input.maxFraction;
    Vec3 bestNormal = y_axis;

    Query(localAABB, [&](int32, int32, int32, const Vec3& a, const Vec3& b, const Vec3& c) {
        RayCastOutput triangleOutput;
        RayCastInput triangleInput = localInput;
        triangleInput.maxFraction = bestFraction;
        if (RayCastTriangle(a, b, c, triangleInput, &triangleOutput))
        {
            hit = true;
            bestFraction = triangleOutput.fraction;
            bestNormal = triangleOutput.normal;
        }
    });

    if (hit == false)
    {
        return false;
    }

    output->fraction = bestFraction;
    output->normal = transform.q.Rotate(bestNormal);
    return true;
}

} // namespace muli3
