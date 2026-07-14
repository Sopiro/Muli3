#include "muli3/mesh_shape.h"
#include "muli3/distance.h"
#include "muli3/ghost.h"
#include "muli3/shapes.h"

namespace muli3
{

MeshShape::MeshShape(std::span<const Vec3> inVertices, std::span<const int32> inIndices, const Transform& transform)
    : Shape(Shape::mesh, 0.0f)
    , vertices{ inVertices.begin(), inVertices.end() }
    , indices{ inIndices.begin(), inIndices.end() }
{
    MuliAssert(vertices.size() >= 3);
    MuliAssert(indices.size() >= 3 && indices.size() % 3 == 0);

    // Shape-local transforms are baked into the vertices before building the acceleration data.
    if (transform != identity)
    {
        for (Vec3& vertex : vertices)
        {
            vertex = Mul(transform, vertex);
        }
    }

    Build();
}

MeshShape::MeshShape(const MeshShape& other, const Transform& transform)
    : Shape(Shape::mesh, 0.0f)
    , vertices{ other.vertices }
    , indices{ other.indices }
{
    if (transform != identity)
    {
        for (Vec3& vertex : vertices)
        {
            vertex = Mul(transform, vertex);
        }
    }

    Build();
}

int32 MeshShape::BuildNode(std::vector<BVHPrimitive>* primitives, int32 begin, int32 end)
{
    // Reserve the parent first so the left child is always stored at nodeIndex + 1.
    int32 nodeIndex = int32(nodes.size());
    nodes.emplace_back();

    // Bounds drive traversal while centroid bounds define the SAH bucket coordinates.
    AABB bounds;
    AABB centroidBounds;
    for (int32 i = begin; i < end; ++i)
    {
        bounds = AABB::Union(bounds, (*primitives)[i].bounds);
        centroidBounds = AABB::Union(centroidBounds, (*primitives)[i].centroid);
    }

    int32 primitiveCount = end - begin;
    if (primitiveCount == 1)
    {
        int32 triangleOffset = int32(bvhTriangles.size());
        bvhTriangles.push_back((*primitives)[begin].triangle);
        nodes[nodeIndex].bounds = bounds;
        nodes[nodeIndex].triangleOffset = triangleOffset;
        nodes[nodeIndex].triangleCount = 1;
        nodes[nodeIndex].axis = 0;
        return nodeIndex;
    }

    // Bin triangle bounds along the longest centroid axis and minimize the surface area heuristic.
    Vec3 extent = centroidBounds.GetExtents();
    int32 axis = extent.y > extent.x ? 1 : 0;
    if (extent.z > extent[axis])
    {
        axis = 2;
    }

    int32 middle = -1;
    if (primitiveCount <= 2)
    {
        middle = (begin + end) / 2;
        std::nth_element(
            primitives->begin() + begin, primitives->begin() + middle, primitives->begin() + end,
            [axis](const BVHPrimitive& a, const BVHPrimitive& b) { return a.centroid[axis] < b.centroid[axis]; }
        );
    }
    else if (extent[axis] > epsilon)
    {
        struct Bucket
        {
            AABB bounds;
            int32 count = 0;
        };

        constexpr int32 bucketCount = 12;
        constexpr int32 splitCount = bucketCount - 1;
        Bucket buckets[bucketCount];
        float scale = bucketCount * (1.0f - epsilon) / extent[axis];

        // Accumulate primitive bounds into a small fixed number of spatial buckets.
        for (int32 i = begin; i < end; ++i)
        {
            BVHPrimitive& primitive = (*primitives)[i];
            int32 bucket = int32((primitive.centroid[axis] - centroidBounds.min[axis]) * scale);
            buckets[bucket].count++;
            buckets[bucket].bounds = AABB::Union(buckets[bucket].bounds, primitive.bounds);
        }

        int32 leftCount[splitCount];
        int32 rightCount[splitCount];
        float leftArea[splitCount];
        float rightArea[splitCount];
        AABB leftBounds;
        AABB rightBounds;
        int32 leftSum = 0;
        int32 rightSum = 0;

        for (int32 i = 0; i < splitCount; ++i)
        {
            leftSum += buckets[i].count;
            leftCount[i] = leftSum;
            leftBounds = AABB::Union(leftBounds, buckets[i].bounds);
            leftArea[i] = leftBounds.GetSurfaceArea();

            int32 rightBucket = bucketCount - 1 - i;
            rightSum += buckets[rightBucket].count;
            rightCount[splitCount - 1 - i] = rightSum;
            rightBounds = AABB::Union(rightBounds, buckets[rightBucket].bounds);
            rightArea[splitCount - 1 - i] = rightBounds.GetSurfaceArea();
        }

        int32 splitBucket = -1;
        float splitCost = max_float;
        for (int32 i = 0; i < splitCount; ++i)
        {
            if (leftCount[i] == 0 || rightCount[i] == 0)
            {
                continue;
            }

            float cost = leftCount[i] * leftArea[i] + rightCount[i] * rightArea[i];
            if (cost < splitCost)
            {
                splitBucket = i;
                splitCost = cost;
            }
        }

        // A leaf is cheaper when its triangles overlap too much to justify another traversal step.
        constexpr float traversalCost = 0.5f;
        float leafCost = float(primitiveCount);
        if (splitBucket >= 0 && traversalCost + splitCost / bounds.GetSurfaceArea() < leafCost)
        {
            auto middleIt =
                std::partition(primitives->begin() + begin, primitives->begin() + end, [&](const BVHPrimitive& primitive) {
                    int32 bucket = int32((primitive.centroid[axis] - centroidBounds.min[axis]) * scale);
                    return bucket <= splitBucket;
                });
            middle = int32(middleIt - primitives->begin());
        }
    }

    if (middle == -1)
    {
        // Keep leaf triangles contiguous so traversal only stores an offset and count in each node.
        int32 triangleOffset = int32(bvhTriangles.size());
        for (int32 i = begin; i < end; ++i)
        {
            bvhTriangles.push_back((*primitives)[i].triangle);
        }
        nodes[nodeIndex].bounds = bounds;
        nodes[nodeIndex].triangleOffset = triangleOffset;
        nodes[nodeIndex].triangleCount = primitiveCount;
        nodes[nodeIndex].axis = uint8(axis);
        return nodeIndex;
    }

    int32 child1 = BuildNode(primitives, begin, middle);
    int32 child2 = BuildNode(primitives, middle, end);
    MuliNotUsed(child1);
    MuliAssert(child1 == nodeIndex + 1);

    // Only the second child needs an explicit index in the depth-first node layout.
    nodes[nodeIndex].bounds = bounds;
    nodes[nodeIndex].child2 = child2;
    nodes[nodeIndex].triangleCount = 0;
    nodes[nodeIndex].axis = uint8(axis);
    return nodeIndex;
}

void MeshShape::Build()
{
    const int32 triangleCount = GetTriangleCount();
    activeEdges.assign(triangleCount, 0b111);

    std::vector<Vec3> normals(triangleCount);
    for (int32 triangle = 0; triangle < triangleCount; ++triangle)
    {
        int32 i = triangle * 3;
        MuliNotUsed(i);
        MuliAssert(0 <= indices[i] && indices[i] < int32(vertices.size()));
        MuliAssert(0 <= indices[i + 1] && indices[i + 1] < int32(vertices.size()));
        MuliAssert(0 <= indices[i + 2] && indices[i + 2] < int32(vertices.size()));

        Vec3 a, b, c;
        GetTriangle(triangle, &a, &b, &c);
        normals[triangle] = Cross(b - a, c - a);
        MuliAssert(normals[triangle].Normalize() > epsilon);
    }

    // Match shared index edges and deactivate coplanar and concave internal edges.
    // Boundary and sharp convex edges stay active so their feature normals remain valid.
    struct Edge
    {
        int32 triangle;
        int32 edge;
    };

    std::unordered_map<uint64, Edge> edgeMap;
    edgeMap.reserve(indices.size());
    constexpr float cosThreshold = 0.996195f; // cos 5 degrees

    for (int32 triangle = 0; triangle < triangleCount; ++triangle)
    {
        int32 base = triangle * 3;
        for (int32 edge = 0; edge < 3; ++edge)
        {
            int32 i0 = indices[base + edge];
            int32 i1 = indices[base + (edge + 1) % 3];

            uint32 minIndex = uint32(Min(i0, i1));
            uint32 maxIndex = uint32(Max(i0, i1));

            uint64 key = (uint64(minIndex) << 32) | maxIndex;

            auto [it, inserted] = edgeMap.emplace(key, Edge{ triangle, edge });
            if (inserted)
            {
                continue;
            }

            Edge neighbor = it->second;
            Vec3 edgeDirection = vertices[i1] - vertices[i0];

            bool concave = Dot(Cross(normals[triangle], normals[neighbor.triangle]), edgeDirection) < 0.0f;
            bool smooth = Dot(normals[triangle], normals[neighbor.triangle]) >= cosThreshold;
            if (concave || smooth)
            {
                activeEdges[triangle] &= uint8(~(1 << edge));
                activeEdges[neighbor.triangle] &= uint8(~(1 << neighbor.edge));
            }
        }
    }

    std::vector<BVHPrimitive> primitives(triangleCount);
    for (int32 triangle = 0; triangle < triangleCount; ++triangle)
    {
        Vec3 a, b, c;
        GetTriangle(triangle, &a, &b, &c);
        AABB bounds{ a, a };
        bounds = AABB::Union(bounds, b);
        bounds = AABB::Union(bounds, c);
        primitives[triangle] = BVHPrimitive{ bounds, bounds.GetCenter(), triangle };
    }

    // Store depth-first nodes and leaf triangles contiguously for cache-friendly traversal.
    bvhTriangles.clear();
    bvhTriangles.reserve(triangleCount);
    nodes.clear();
    nodes.reserve(triangleCount * 2 - 1);
    BuildNode(&primitives, 0, triangleCount);
    localBounds = nodes[0].bounds;
    center = localBounds.GetCenter();
    volume = 0.0f;
}

void MeshShape::ComputeMass(float density, MassData* outMassData) const
{
    MuliNotUsed(density);
    MuliAssert(outMassData != nullptr);
    outMassData->mass = 0.0f;
    outMassData->inertia = Mat3::zero;
    outMassData->centerOfMass = center;
}

void MeshShape::ComputeAABB(const Transform& transform, AABB* outAABB) const
{
    MuliAssert(outAABB != nullptr);

    Vec3 corners[8] = {
        Mul(transform, Vec3{ localBounds.min.x, localBounds.min.y, localBounds.min.z }),
        Mul(transform, Vec3{ localBounds.max.x, localBounds.min.y, localBounds.min.z }),
        Mul(transform, Vec3{ localBounds.min.x, localBounds.max.y, localBounds.min.z }),
        Mul(transform, Vec3{ localBounds.max.x, localBounds.max.y, localBounds.min.z }),
        Mul(transform, Vec3{ localBounds.min.x, localBounds.min.y, localBounds.max.z }),
        Mul(transform, Vec3{ localBounds.max.x, localBounds.min.y, localBounds.max.z }),
        Mul(transform, Vec3{ localBounds.min.x, localBounds.max.y, localBounds.max.z }),
        Mul(transform, Vec3{ localBounds.max.x, localBounds.max.y, localBounds.max.z }),
    };

    *outAABB = AABB{ corners[0], corners[0] };
    for (int32 i = 1; i < 8; ++i)
    {
        *outAABB = AABB::Union(*outAABB, corners[i]);
    }
}

bool MeshShape::TestPoint(const Transform& transform, const Vec3& q) const
{
    return Dist2(GetClosestPoint(transform, q), q) <= Sqr(linear_slop);
}

Vec3 MeshShape::GetClosestPoint(const Transform& transform, const Vec3& q) const
{
    Vec3 localQ = MulT(transform, q);
    Vec3 closest = localBounds.GetCenter();
    float minDistance2 = max_float;

    // Closest-point queries are uncommon for static meshes.
    // Walk the BVH without allocating scratch memory.
    int32 stack[64];
    int32 count = 0;

    stack[count++] = 0;
    while (count > 0)
    {
        int32 nodeIndex = stack[--count];
        const BVHNode& node = nodes[nodeIndex];
        Vec3 boxPoint = Clamp(localQ, node.bounds.min, node.bounds.max);
        if (Dist2(localQ, boxPoint) >= minDistance2)
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
                Vec3 point = ClosestPointVsTriangle(localQ, a, b, c);
                float distance2 = Dist2(localQ, point);
                if (distance2 < minDistance2)
                {
                    minDistance2 = distance2;
                    closest = point;
                }
            }
        }
        else
        {
            MuliAssert(count + 2 <= int32(std::size(stack)));
            int32 child1 = nodeIndex + 1;
            int32 child2 = node.child2;
            Vec3 point1 = Clamp(localQ, nodes[child1].bounds.min, nodes[child1].bounds.max);
            Vec3 point2 = Clamp(localQ, nodes[child2].bounds.min, nodes[child2].bounds.max);

            // Visit the closer child first so a tighter distance rejects the far subtree sooner.
            if (Dist2(localQ, point1) < Dist2(localQ, point2))
            {
                stack[count++] = child2;
                stack[count++] = child1;
            }
            else
            {
                stack[count++] = child1;
                stack[count++] = child2;
            }
        }
    }

    return Mul(transform, closest);
}

bool MeshShape::RayCast(const Transform& transform, const RayCastInput& input, RayCastOutput* output) const
{
    MuliAssert(output != nullptr);

    RayCastInput localInput = input;
    localInput.from = MulT(transform, input.from);
    localInput.to = MulT(transform, input.to);
    Ray ray{ localInput.from, localInput.to - localInput.from };

    // Reuse inverse directions for every node slab test during traversal.
    Vec3 invDir{ 1.0f / ray.d.x, 1.0f / ray.d.y, 1.0f / ray.d.z };
    int32 isDirNegative[3] = { int32(invDir.x < 0.0f), int32(invDir.y < 0.0f), int32(invDir.z < 0.0f) };

    bool hit = false;
    float bestFraction = input.maxFraction;
    Vec3 bestNormal = Vec3::zero;

    int32 stack[64];
    int32 count = 0;

    stack[count++] = 0;
    while (count > 0)
    {
        int32 nodeIndex = stack[--count];
        const BVHNode& node = nodes[nodeIndex];
        if (node.bounds.TestRay(ray.o, 0.0f, bestFraction, invDir, isDirNegative) == false)
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
                RayCastOutput candidate;
                localInput.maxFraction = bestFraction;
                if (RayCastTriangle(a, b, c, localInput, &candidate))
                {
                    hit = true;
                    bestFraction = candidate.fraction;
                    bestNormal = candidate.normal;
                }
            }
        }
        else
        {
            MuliAssert(count + 2 <= int32(std::size(stack)));
            int32 child1 = nodeIndex + 1;
            int32 child2 = node.child2;

            // Put the far child on the stack first so the near child shortens the ray sooner.
            if (isDirNegative[node.axis])
            {
                stack[count++] = child1;
                stack[count++] = child2;
            }
            else
            {
                stack[count++] = child2;
                stack[count++] = child1;
            }
        }
    }

    if (hit == false)
    {
        return false;
    }

    output->fraction = bestFraction;
    output->normal = transform.q.Rotate(bestNormal);
    return true;
}

bool MeshShape::ShapeCast(
    const Transform& transform,
    const Shape* shape,
    const Transform& shapeTransform,
    const Vec3& translation,
    ShapeCastOutput* output
) const
{
    MuliAssert(output != nullptr);
    MuliAssert(shape->GetType() < Shape::height_field);

    // Cast the center of the shape AABB against BVH bounds enlarged by its extents.
    Transform localTransform = MulT(transform, shapeTransform);

    AABB shapeBounds;
    shape->ComputeAABB(localTransform, &shapeBounds);

    Vec3 shapeCenter = shapeBounds.GetCenter();
    Vec3 shapeExtent = shapeBounds.GetExtents() * 0.5f;
    Vec3 localTranslation = transform.q.RotateInv(translation);

    Ray ray{ shapeCenter, localTranslation };
    Vec3 invDir{ 1.0f / ray.d.x, 1.0f / ray.d.y, 1.0f / ray.d.z };
    int32 isDirNegative[3] = { int32(invDir.x < 0.0f), int32(invDir.y < 0.0f), int32(invDir.z < 0.0f) };

    bool hit = false;
    float bestFraction = 1.0f;
    ShapeCastOutput bestOutput;

    int32 stack[64];
    int32 count = 0;

    stack[count++] = 0;
    while (count > 0)
    {
        int32 nodeIndex = stack[--count];
        const BVHNode& node = nodes[nodeIndex];

        AABB bounds{ node.bounds.min - shapeExtent, node.bounds.max + shapeExtent };
        if (bounds.TestRay(ray.o, 0.0f, bestFraction, invDir, isDirNegative) == false)
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

                // Reject leaf triangles before running the more expensive convex cast.
                AABB triangleBounds{ a, a };
                triangleBounds = AABB::Union(triangleBounds, b);
                triangleBounds = AABB::Union(triangleBounds, c);
                triangleBounds.min -= shapeExtent;
                triangleBounds.max += shapeExtent;
                if (triangleBounds.TestRay(ray.o, 0.0f, bestFraction, invDir, isDirNegative) == false)
                {
                    continue;
                }

                TriangleShape triangleShape{ a, b, c };
                ShapeCastOutput candidate;

                // Shorten later casts to the closest fraction found so far.
                if (muli3::ShapeCast(
                        shape, shapeTransform, &triangleShape, transform, translation * bestFraction, Vec3::zero, &candidate
                    ))
                {
                    // ShapeCast returns a fraction relative to the shortened translation.
                    candidate.t *= bestFraction;
                    candidate.normal = ResolveGhostNormal(
                        GetActiveEdgeBits(triangle), triangleShape, transform, candidate.point, candidate.normal, translation
                    );
                    if (candidate.t <= bestFraction)
                    {
                        hit = true;
                        bestFraction = candidate.t;
                        bestOutput = candidate;
                    }
                }
            }
        }
        else
        {
            MuliAssert(count + 2 <= int32(std::size(stack)));
            int32 child1 = nodeIndex + 1;
            int32 child2 = node.child2;

            // Put the far child on the stack first to find a close hit earlier.
            if (isDirNegative[node.axis])
            {
                stack[count++] = child1;
                stack[count++] = child2;
            }
            else
            {
                stack[count++] = child2;
                stack[count++] = child1;
            }
        }
    }

    if (hit)
    {
        *output = bestOutput;
    }
    return hit;
}

} // namespace muli3
