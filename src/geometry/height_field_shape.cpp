#include "muli3/height_field_shape.h"
#include "muli3/distance.h"
#include "muli3/parallel_for.h"
#include "muli3/settings.h"
#include "muli3/shapes.h"

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

    // Build active edge flag bits
    //
    // 00-----10
    // |  \  1 |
    // | 0  \  |
    // 01---- 11
    //
    // For triangle 0:
    // edge 0 = p00 -> p01 : left cell edge
    // edge 1 = p01 -> p11 : lower cell edge
    // edge 2 = p11 -> p00 : diagonal edge
    //
    // For triangle 1:
    // edge 0 = p00 -> p11 : diagonal edge
    // edge 1 = p11 -> p10 : right cell edge
    // edge 2 = p10 -> p00 : upper cell edge

    int32 triangleCount = cellCountX * cellCountZ * 2;
    activeEdges.assign(triangleCount, 0);

    // Cache vertices and normals
    std::vector<Vec3> normals(triangleCount);
    std::vector<Vec3> vertices(triangleCount * 3);

    for (int32 z = 0; z < cellCountZ; ++z)
    {
        for (int32 x = 0; x < cellCountX; ++x)
        {
            for (int32 triangle = 0; triangle < 2; ++triangle)
            {
                int32 id = (z * cellCountX + x) * 2 + triangle;

                Vec3* v = vertices.data() + id * 3;
                GetTriangle(x, z, triangle, v, v + 1, v + 2);

                normals[id] = Normalize(Cross(v[1] - v[0], v[2] - v[0]));
            }
        }
    }

    constexpr float cosThreshold = 0.996195f; // cos(5 degrees)

    const auto ProcessEdge = [&](int32 x, int32 z, int32 triangle, int32 edge, int32 neighborX, int32 neighborZ,
                                 int32 neighborTriangle) {
        int32 id = (z * cellCountX + x) * 2 + triangle;

        const Vec3* v = vertices.data() + id * 3;
        Vec3 edgeDirection = v[(edge + 1) % 3] - v[edge];

        // Always active if the neighbor is outside the height field
        bool active = neighborX < 0 || neighborX >= cellCountX || neighborZ < 0 || neighborZ >= cellCountZ;
        if (!active)
        {
            int32 neighborId = (neighborZ * cellCountX + neighborX) * 2 + neighborTriangle;

            // Concave edges are internal to the terrain surface.
            if (Dot(Cross(normals[id], normals[neighborId]), edgeDirection) < 0.0f)
            {
                active = false;
            }
            else
            {
                float cosNormal = Dot(normals[id], normals[neighborId]);
                active = cosNormal < cosThreshold;
            }
        }

        if (active)
        {
            activeEdges[id] |= uint8(1 << edge);
        }
    };

    for (int32 z = 0; z < cellCountZ; ++z)
    {
        for (int32 x = 0; x < cellCountX; ++x)
        {
            // Triangle 0: p00, p01, p11
            ProcessEdge(x, z, 0, 0, x - 1, z, 1);
            ProcessEdge(x, z, 0, 1, x, z + 1, 1);
            ProcessEdge(x, z, 0, 2, x, z, 1);

            // Triangle 1: p00, p11, p10
            ProcessEdge(x, z, 1, 0, x, z, 0);
            ProcessEdge(x, z, 1, 1, x + 1, z, 0);
            ProcessEdge(x, z, 1, 2, x, z - 1, 0);
        }
    }
}

static Vec3 GetBarycentric(const Vec3& p, const Vec3& a, const Vec3& b, const Vec3& c)
{
    Vec3 v0 = b - a;
    Vec3 v1 = c - a;
    Vec3 v2 = p - a;

    float d00 = Dot(v0, v0);
    float d01 = Dot(v0, v1);
    float d11 = Dot(v1, v1);
    float d20 = Dot(v2, v0);
    float d21 = Dot(v2, v1);
    float denom = d00 * d11 - d01 * d01;
    if (Abs(denom) <= epsilon)
    {
        return Vec3{ 1.0f, 0.0f, 0.0f };
    }

    float v = (d11 * d20 - d01 * d21) / denom;
    float w = (d00 * d21 - d01 * d20) / denom;
    return Vec3{ 1.0f - v - w, v, w };
}

// Jolt style ghost collision resolution: JPH::ActiveEdges::FixNormal
Vec3 HeightFieldShape::FixNormal(
    int32 x, int32 z, int32 triangle, const Transform& transform, const Vec3& point, const Vec3& normal, const Vec3& translation
) const
{
    uint8 activeEdges = GetActiveEdgeBits(x, z, triangle);
    if (activeEdges == 0b111)
    {
        // Every edge is active
        return normal;
    }

    Vec3 a, b, c;
    GetTriangle(x, z, triangle, &a, &b, &c);

    a = Mul(transform, a);
    b = Mul(transform, b);
    c = Mul(transform, c);

    Vec3 faceNormal = Cross(b - a, c - a);
    if (faceNormal.Normalize() == 0.0f)
    {
        return normal;
    }

    if (Dot(faceNormal, normal) < 0.0f)
    {
        faceNormal = -faceNormal;
    }

    // For casts, keep the original normal if it blocks the sweep less than the face normal.
    // This avoids replacing a grazing side hit with a stronger terrain-normal hit.
    if (Length2(translation) > epsilon && Dot(translation, normal) < Dot(translation, faceNormal))
    {
        return normal;
    }

    // Almost a face hit.
    if (Dot(faceNormal, normal) > 0.999848f) // cos 179
    {
        return normal;
    }

    constexpr float epsilon0 = 1.0e-4f;
    constexpr float epsilon1 = 1.0f - epsilon0;

    Vec3 bary = GetBarycentric(point, a, b, c);
    uint8 collidingEdge = 0;

    // Build edge bits for the feature nearest the collision point.
    if (bary.x > epsilon1)
    {
        // Collision is near vertex 0, so edge 0 or 2 needs to be tested.
        collidingEdge = 0b101;
    }
    else if (bary.y > epsilon1)
    {
        // Collision is near vertex 1, so edge 0 or 1 needs to be tested.
        collidingEdge = 0b011;
    }
    else if (bary.z > epsilon1)
    {
        // Collision is near vertex 2, so edge 1 or 2 needs to be tested.
        collidingEdge = 0b110;
    }
    else if (bary.x < epsilon0)
    {
        // Collision is near edge 1.
        collidingEdge = 0b010;
    }
    else if (bary.y < epsilon0)
    {
        // Collision is near edge 2.
        collidingEdge = 0b100;
    }
    else if (bary.z < epsilon0)
    {
        // Collision is near edge 0.
        collidingEdge = 0b001;
    }
    else
    {
        // Interior hit.
        return faceNormal;
    }

    // Keep the original normal if the feature includes an active edge, otherwise use the face normal.
    return (activeEdges & collidingEdge) != 0 ? normal : faceNormal;
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

bool HeightFieldShape::ShapeCast(
    const Transform& transform,
    const Shape* shape,
    const Transform& shapeTransform,
    const Vec3& translation,
    ShapeCastOutput* output
) const
{
    MuliAssert(output != nullptr);
    MuliAssert(shape->GetType() != Shape::height_field);

    AABB worldAABB;
    shape->ComputeAABB(shapeTransform, &worldAABB);

    // Transform all corners to the height field space.
    Vec3 corners[8] = {
        MulT(transform, Vec3{ worldAABB.min.x, worldAABB.min.y, worldAABB.min.z }),
        MulT(transform, Vec3{ worldAABB.max.x, worldAABB.min.y, worldAABB.min.z }),
        MulT(transform, Vec3{ worldAABB.min.x, worldAABB.max.y, worldAABB.min.z }),
        MulT(transform, Vec3{ worldAABB.max.x, worldAABB.max.y, worldAABB.min.z }),
        MulT(transform, Vec3{ worldAABB.min.x, worldAABB.min.y, worldAABB.max.z }),
        MulT(transform, Vec3{ worldAABB.max.x, worldAABB.min.y, worldAABB.max.z }),
        MulT(transform, Vec3{ worldAABB.min.x, worldAABB.max.y, worldAABB.max.z }),
        MulT(transform, Vec3{ worldAABB.max.x, worldAABB.max.y, worldAABB.max.z }),
    };

    AABB localAABB{ corners[0], corners[0] };
    for (int32 i = 1; i < 8; ++i)
    {
        localAABB = AABB::Union(localAABB, corners[i]);
    }

    // Shape cast keeps the orientation fixed, so the local AABB only translates.
    Vec3 localTranslation = transform.q.RotateInv(translation) / transform.s;
    if (Length2(localTranslation) <= epsilon)
    {
        return false;
    }

    float t0 = 0.0f;
    float t1 = 1.0f;

    // Clip the swept AABB to the height field bounds before walking the grid.
    for (int32 axis = 0; axis < 3; ++axis)
    {
        float minA = localAABB.min[axis];
        float maxA = localAABB.max[axis];
        float dir = localTranslation[axis];
        float minB = localBounds.min[axis];
        float maxB = localBounds.max[axis];

        if (Abs(dir) <= epsilon)
        {
            if (maxA < minB || minA > maxB)
            {
                return false;
            }
            continue;
        }

        float enter = (minB - maxA) / dir;
        float exit = (maxB - minA) / dir;
        if (enter > exit)
        {
            std::swap(enter, exit);
        }

        t0 = Max(t0, enter);
        t1 = Min(t1, exit);
        if (t0 > t1)
        {
            return false;
        }
    }

    int32 cellCountX = GetCellCountX();
    int32 cellCountZ = GetCellCountZ();
    float gridMaxX = offset.x + cellCountX * cellSizeX;
    float gridMaxZ = offset.z + cellCountZ * cellSizeZ;

    // Bias the leading face slightly so boundary hits enter the next cell.
    float sweepEpsilon = 1e-4f;

    // Start from the footprint clipped into the height field.
    float minX = localAABB.min.x + localTranslation.x * t0;
    float maxX = localAABB.max.x + localTranslation.x * t0;
    float minZ = localAABB.min.z + localTranslation.z * t0;
    float maxZ = localAABB.max.z + localTranslation.z * t0;
    if (localTranslation.x < 0.0f)
    {
        minX -= sweepEpsilon;
    }
    else if (localTranslation.x > 0.0f)
    {
        maxX += sweepEpsilon;
    }
    if (localTranslation.z < 0.0f)
    {
        minZ -= sweepEpsilon;
    }
    else if (localTranslation.z > 0.0f)
    {
        maxZ += sweepEpsilon;
    }

    // Current cell range indices of AABB
    int32 currentMin[2] = {
        Clamp(int32(std::floor((Clamp(minX, offset.x, gridMaxX) - offset.x) / cellSizeX)), 0, cellCountX - 1),
        Clamp(int32(std::floor((Clamp(minZ, offset.z, gridMaxZ) - offset.z) / cellSizeZ)), 0, cellCountZ - 1),
    };
    int32 currentMax[2] = {
        Clamp(int32(std::floor((Clamp(maxX, offset.x, gridMaxX) - offset.x) / cellSizeX)), 0, cellCountX - 1),
        Clamp(int32(std::floor((Clamp(maxZ, offset.z, gridMaxZ) - offset.z) / cellSizeZ)), 0, cellCountZ - 1),
    };

    bool hit = false;
    float bestFraction = 1.0f;
    ShapeCastOutput bestOutput;

    // Test every cell overlapped by the initial AABB footprint.
    for (int32 z = currentMin[1]; z <= currentMax[1]; ++z)
    {
        for (int32 x = currentMin[0]; x <= currentMax[0]; ++x)
        {
            float h00 = GetHeight(x, z);
            float h10 = GetHeight(x + 1, z);
            float h01 = GetHeight(x, z + 1);
            float h11 = GetHeight(x + 1, z + 1);
            float cellMinY = offset.y + Min(Min(h00, h10), Min(h01, h11));
            float cellMaxY = offset.y + Max(Max(h00, h10), Max(h01, h11));
            float sweptMinY = Min(localAABB.min.y, localAABB.min.y + localTranslation.y * bestFraction);
            float sweptMaxY = Max(localAABB.max.y, localAABB.max.y + localTranslation.y * bestFraction);

            // Reject cells whose height range cannot touch the swept AABB.
            if (sweptMaxY < cellMinY || sweptMinY > cellMaxY)
            {
                continue;
            }

            for (int32 triangle = 0; triangle < 2; ++triangle)
            {
                Vec3 a, b, c;
                GetTriangle(x, z, triangle, &a, &b, &c);
                TriangleShape triangleShape{ a, b, c };

                ShapeCastOutput candidate;
                if (muli3::ShapeCast(
                        shape, shapeTransform, &triangleShape, transform, translation * bestFraction, Vec3::zero, &candidate
                    ))
                {
                    candidate.t *= bestFraction;
                    candidate.normal = FixNormal(x, z, triangle, transform, candidate.point, candidate.normal, translation);
                    if (candidate.t <= bestFraction)
                    {
                        hit = true;
                        bestFraction = candidate.t;
                        bestOutput = candidate;
                    }
                }
            }
        }
    }

    float nextT[2] = { max_float, max_float };
    float deltaT[2] = { max_float, max_float };

    // DDA tracks the leading AABB face: max face for positive motion, min face for negative motion.
    if (localTranslation.x > epsilon)
    {
        nextT[0] = (offset.x + (currentMax[0] + 1) * cellSizeX - localAABB.max.x) / localTranslation.x;
        deltaT[0] = cellSizeX / localTranslation.x;
    }
    else if (localTranslation.x < -epsilon)
    {
        nextT[0] = (offset.x + currentMin[0] * cellSizeX - localAABB.min.x) / localTranslation.x;
        deltaT[0] = -cellSizeX / localTranslation.x;
    }

    if (localTranslation.z > epsilon)
    {
        nextT[1] = (offset.z + (currentMax[1] + 1) * cellSizeZ - localAABB.max.z) / localTranslation.z;
        deltaT[1] = cellSizeZ / localTranslation.z;
    }
    else if (localTranslation.z < -epsilon)
    {
        nextT[1] = (offset.z + currentMin[1] * cellSizeZ - localAABB.min.z) / localTranslation.z;
        deltaT[1] = -cellSizeZ / localTranslation.z;
    }

    // The initial footprint has already been visited, so move to the next crossing after t0.
    for (int32 axis = 0; axis < 2; ++axis)
    {
        while (nextT[axis] <= t0 + epsilon)
        {
            nextT[axis] += deltaT[axis];
        }
    }

    while (true)
    {
        int32 stepAxis = nextT[0] <= nextT[1] ? 0 : 1;
        float t = nextT[stepAxis];
        if (t > t1 || t > bestFraction)
        {
            break;
        }

        int32 oldMin[2] = { currentMin[0], currentMin[1] };
        int32 oldMax[2] = { currentMax[0], currentMax[1] };

        // Update the AABB footprint at this crossing and bias the leading face to include boundary-touching cells.
        minX = localAABB.min.x + localTranslation.x * t;
        maxX = localAABB.max.x + localTranslation.x * t;
        minZ = localAABB.min.z + localTranslation.z * t;
        maxZ = localAABB.max.z + localTranslation.z * t;
        if (localTranslation.x < 0.0f)
        {
            minX -= sweepEpsilon;
        }
        else if (localTranslation.x > 0.0f)
        {
            maxX += sweepEpsilon;
        }
        if (localTranslation.z < 0.0f)
        {
            minZ -= sweepEpsilon;
        }
        else if (localTranslation.z > 0.0f)
        {
            maxZ += sweepEpsilon;
        }

        currentMin[0] = Clamp(int32(std::floor((Clamp(minX, offset.x, gridMaxX) - offset.x) / cellSizeX)), 0, cellCountX - 1);
        currentMax[0] = Clamp(int32(std::floor((Clamp(maxX, offset.x, gridMaxX) - offset.x) / cellSizeX)), 0, cellCountX - 1);
        currentMin[1] = Clamp(int32(std::floor((Clamp(minZ, offset.z, gridMaxZ) - offset.z) / cellSizeZ)), 0, cellCountZ - 1);
        currentMax[1] = Clamp(int32(std::floor((Clamp(maxZ, offset.z, gridMaxZ) - offset.z) / cellSizeZ)), 0, cellCountZ - 1);

        // Only the newly covered row or column can contain new candidate triangles.
        for (int32 z = currentMin[1]; z <= currentMax[1]; ++z)
        {
            for (int32 x = currentMin[0]; x <= currentMax[0]; ++x)
            {
                if (oldMin[0] <= x && x <= oldMax[0] && oldMin[1] <= z && z <= oldMax[1])
                {
                    continue;
                }

                float h00 = GetHeight(x, z);
                float h10 = GetHeight(x + 1, z);
                float h01 = GetHeight(x, z + 1);
                float h11 = GetHeight(x + 1, z + 1);
                float cellMinY = offset.y + Min(Min(h00, h10), Min(h01, h11));
                float cellMaxY = offset.y + Max(Max(h00, h10), Max(h01, h11));
                float sweptMinY = Min(localAABB.min.y, localAABB.min.y + localTranslation.y * bestFraction);
                float sweptMaxY = Max(localAABB.max.y, localAABB.max.y + localTranslation.y * bestFraction);

                // Reuse the broad Y range of the current best sweep before the exact cast.
                if (sweptMaxY < cellMinY || sweptMinY > cellMaxY)
                {
                    continue;
                }

                for (int32 triangle = 0; triangle < 2; ++triangle)
                {
                    Vec3 a, b, c;
                    GetTriangle(x, z, triangle, &a, &b, &c);
                    TriangleShape triangleShape{ a, b, c };

                    ShapeCastOutput candidate;
                    if (muli3::ShapeCast(
                            shape, shapeTransform, &triangleShape, transform, translation * bestFraction, Vec3::zero, &candidate
                        ))
                    {
                        candidate.t *= bestFraction;
                        candidate.normal = FixNormal(x, z, triangle, transform, candidate.point, candidate.normal, translation);
                        if (candidate.t <= bestFraction)
                        {
                            hit = true;
                            bestFraction = candidate.t;
                            bestOutput = candidate;
                        }
                    }
                }
            }
        }

        nextT[stepAxis] += deltaT[stepAxis];
    }

    if (hit == false)
    {
        return false;
    }

    *output = bestOutput;
    return true;
}

bool HeightFieldShape::RayCast(const Transform& transform, const RayCastInput& input, RayCastOutput* output) const
{
    // Transform ray to the height field local space
    RayCastInput localInput = input;
    localInput.from = MulT(transform, input.from);
    localInput.to = MulT(transform, input.to);

    Vec3 d = localInput.to - localInput.from;
    if (Length2(d) <= epsilon)
    {
        return false;
    }

    // Clip ray to the height field bounds
    float t0 = 0.0f;
    float t1 = input.maxFraction;

    for (int32 axis = 0; axis < 3; ++axis)
    {
        float origin = localInput.from[axis];
        float dir = d[axis];
        float min = localBounds.min[axis];
        float max = localBounds.max[axis];

        if (Abs(dir) <= epsilon)
        {
            if (origin < min || origin > max)
            {
                return false;
            }
            continue;
        }

        float invDir = 1.0f / dir;
        float enter = (min - origin) * invDir;
        float exit = (max - origin) * invDir;
        if (enter > exit)
        {
            std::swap(enter, exit);
        }

        t0 = Max(t0, enter);
        t1 = Min(t1, exit);
        if (t0 > t1)
        {
            return false;
        }
    }

    bool hit = false;
    float bestFraction = input.maxFraction;
    Vec3 bestNormal = y_axis;

    int32 cellCountX = GetCellCountX();
    int32 cellCountZ = GetCellCountZ();

    // Find the cell containing the clipped ray start
    Vec3 p0 = localInput.from + d * t0;
    int32 cell[2] = {
        Clamp(int32(std::floor((p0.x - offset.x) / cellSizeX)), 0, cellCountX - 1),
        Clamp(int32(std::floor((p0.z - offset.z) / cellSizeZ)), 0, cellCountZ - 1),
    };

    int32 step[2] = { 0, 0 };
    int32 cellEnd[2] = { 0, 0 };
    float nextT[2] = { max_float, max_float };
    float deltaT[2] = { max_float, max_float };

    float gridOrigin[2] = { localInput.from.x, localInput.from.z };
    float gridDir[2] = { d.x, d.z };
    float gridOffset[2] = { offset.x, offset.z };
    float gridCellSize[2] = { cellSizeX, cellSizeZ };
    int32 gridCellCount[2] = { cellCountX, cellCountZ };

    // Precompute 2D DDA stepping over the XZ grid
    for (int32 axis = 0; axis < 2; ++axis)
    {
        if (Abs(gridDir[axis]) <= epsilon)
        {
            continue;
        }

        if (gridDir[axis] > 0.0f)
        {
            step[axis] = 1;
            cellEnd[axis] = gridCellCount[axis];
            nextT[axis] = (gridOffset[axis] + (cell[axis] + 1) * gridCellSize[axis] - gridOrigin[axis]) / gridDir[axis];
        }
        else
        {
            step[axis] = -1;
            cellEnd[axis] = -1;
            nextT[axis] = (gridOffset[axis] + cell[axis] * gridCellSize[axis] - gridOrigin[axis]) / gridDir[axis];
        }
        deltaT[axis] = gridCellSize[axis] / Abs(gridDir[axis]);
    }

    while (t0 <= t1)
    {
        // Skip cells outside the ray height range
        float h00 = GetHeight(cell[0], cell[1]);
        float h10 = GetHeight(cell[0] + 1, cell[1]);
        float h01 = GetHeight(cell[0], cell[1] + 1);
        float h11 = GetHeight(cell[0] + 1, cell[1] + 1);
        float minHeight = offset.y + Min(Min(h00, h10), Min(h01, h11));
        float maxHeight = offset.y + Max(Max(h00, h10), Max(h01, h11));

        float rayHeight = localInput.from.y + d.y * bestFraction;

        if (Max(localInput.from.y, rayHeight) >= minHeight && Min(localInput.from.y, rayHeight) <= maxHeight)
        {
            RayCastInput triangleInput = localInput;

            // Test the two triangles in the current cell
            for (int32 i = 0; i < 2; ++i)
            {
                triangleInput.maxFraction = bestFraction;

                Vec3 a, b, c;
                GetTriangle(cell[0], cell[1], i, &a, &b, &c);

                RayCastOutput candidate;
                if (RayCastTriangle(a, b, c, triangleInput, &candidate))
                {
                    if (candidate.fraction <= bestFraction)
                    {
                        hit = true;
                        bestFraction = candidate.fraction;
                        t1 = Min(t1, bestFraction);
                        bestNormal = candidate.normal;
                    }
                }
            }
        }

        // Advance to the next X or Z cell boundary
        int32 stepAxis = nextT[0] <= nextT[1] ? 0 : 1;
        if (nextT[stepAxis] > t1)
        {
            break;
        }

        cell[stepAxis] += step[stepAxis];
        if (cell[stepAxis] == cellEnd[stepAxis])
        {
            break;
        }

        t0 = nextT[stepAxis];
        nextT[stepAxis] += deltaT[stepAxis];
    }

    if (hit == false)
    {
        return false;
    }

    output->fraction = bestFraction;
    output->normal = transform.q.Rotate(bestNormal);
    return true;
}

} // namespace muli3
