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

    for (Vec3& vertex : vertices)
    {
        vertex = Mul(transform, vertex);
    }

    Build();
}

MeshShape::MeshShape(const MeshShape& other, const Transform& transform)
    : Shape(Shape::mesh, 0.0f)
    , vertices{ other.vertices }
    , indices{ other.indices }
{
    for (Vec3& vertex : vertices)
    {
        vertex = Mul(transform, vertex);
    }

    Build();
}

int32 MeshShape::BuildNode(std::vector<int32>* triangles, int32 begin, int32 end)
{
    int32 nodeIndex = int32(nodes.size());
    nodes.emplace_back();

    AABB bounds;
    AABB centroidBounds;
    for (int32 i = begin; i < end; ++i)
    {
        Vec3 a, b, c;
        GetTriangle((*triangles)[i], &a, &b, &c);
        bounds = AABB::Union(bounds, a);
        bounds = AABB::Union(bounds, b);
        bounds = AABB::Union(bounds, c);
        centroidBounds = AABB::Union(centroidBounds, (a + b + c) / 3.0f);
    }

    if (end - begin == 1)
    {
        nodes[nodeIndex] = BVHNode{ bounds, -1, -1, (*triangles)[begin] };
        return nodeIndex;
    }

    // Split the longest centroid axis to keep the binary tree balanced and spatially coherent.
    Vec3 extent = centroidBounds.GetExtents();
    int32 axis = extent.y > extent.x ? 1 : 0;
    if (extent.z > extent[axis])
    {
        axis = 2;
    }

    int32 middle = (begin + end) / 2;
    std::nth_element(triangles->begin() + begin, triangles->begin() + middle, triangles->begin() + end, [&](int32 a, int32 b) {
        int32 ia = a * 3;
        int32 ib = b * 3;
        float ca = (vertices[indices[ia]][axis] + vertices[indices[ia + 1]][axis] + vertices[indices[ia + 2]][axis]) / 3.0f;
        float cb = (vertices[indices[ib]][axis] + vertices[indices[ib + 1]][axis] + vertices[indices[ib + 2]][axis]) / 3.0f;
        return ca < cb;
    });

    int32 child1 = BuildNode(triangles, begin, middle);
    int32 child2 = BuildNode(triangles, middle, end);
    nodes[nodeIndex] = BVHNode{ bounds, child1, child2, -1 };
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

    std::vector<int32> triangles(triangleCount);
    for (int32 i = 0; i < triangleCount; ++i)
    {
        triangles[i] = i;
    }

    // Store the BVH in one contiguous array for cache-friendly traversal.
    nodes.clear();
    nodes.reserve(triangleCount * 2 - 1);
    BuildNode(&triangles, 0, triangleCount);
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
        const BVHNode& node = nodes[stack[--count]];
        Vec3 boxPoint = Clamp(localQ, node.bounds.min, node.bounds.max);
        if (Dist2(localQ, boxPoint) >= minDistance2)
        {
            continue;
        }

        if (node.triangle != -1)
        {
            Vec3 a, b, c;
            GetTriangle(node.triangle, &a, &b, &c);
            Vec3 point = ClosestPointVsTriangle(localQ, a, b, c);
            float distance2 = Dist2(localQ, point);
            if (distance2 < minDistance2)
            {
                minDistance2 = distance2;
                closest = point;
            }
        }
        else
        {
            MuliAssert(count + 2 <= int32(std::size(stack)));
            stack[count++] = node.child1;
            stack[count++] = node.child2;
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

    bool hit = false;
    float bestFraction = input.maxFraction;
    Vec3 bestNormal = Vec3::zero;

    int32 stack[64];
    int32 count = 0;

    stack[count++] = 0;
    while (count > 0)
    {
        const BVHNode& node = nodes[stack[--count]];
        if (node.bounds.TestRay(ray, 0.0f, bestFraction) == false)
        {
            continue;
        }

        if (node.triangle != -1)
        {
            Vec3 a, b, c;
            GetTriangle(node.triangle, &a, &b, &c);
            RayCastOutput candidate;
            localInput.maxFraction = bestFraction;
            if (RayCastTriangle(a, b, c, localInput, &candidate))
            {
                hit = true;
                bestFraction = candidate.fraction;
                bestNormal = candidate.normal;
            }
        }
        else
        {
            MuliAssert(count + 2 <= int32(std::size(stack)));
            stack[count++] = node.child1;
            stack[count++] = node.child2;
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

    AABB worldAABB;
    shape->ComputeAABB(shapeTransform, &worldAABB);
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

    AABB sweptAABB{ corners[0], corners[0] };
    for (int32 i = 1; i < 8; ++i)
    {
        sweptAABB = AABB::Union(sweptAABB, corners[i]);
    }
    Vec3 localTranslation = transform.q.RotateInv(translation);
    sweptAABB = AABB::Union(sweptAABB, AABB{ sweptAABB.min + localTranslation, sweptAABB.max + localTranslation });

    bool hit = false;
    float bestFraction = 1.0f;
    ShapeCastOutput bestOutput;
    Query(sweptAABB, [&](int32 triangle, const Vec3& a, const Vec3& b, const Vec3& c) {
        TriangleShape triangleShape{ a, b, c };
        ShapeCastOutput candidate;
        if (muli3::ShapeCast(
                shape, shapeTransform, &triangleShape, transform, translation * bestFraction, Vec3::zero, &candidate
            ))
        {
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
    });

    if (hit)
    {
        *output = bestOutput;
    }
    return hit;
}

} // namespace muli3
