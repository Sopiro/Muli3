#include "muli3/geometry.h"
#include "muli3/shape.h"

namespace muli3
{

struct HullFace
{
    int32 indices[3];
    Vec3 normal;
};

struct HullEdge
{
    int32 a;
    int32 b;
};

static bool Contains(std::span<const int32> indices, int32 value)
{
    for (int32 index : indices)
    {
        if (index == value)
        {
            return true;
        }
    }

    return false;
}

static void AddBoundaryEdge(std::vector<HullEdge>* edges, int32 a, int32 b)
{
    // An edge shared by two visible faces appears twice with opposite direction.
    // Remove those internal edges and keep only the patch boundary.
    for (int32 i = 0; i < int32(edges->size()); ++i)
    {
        if ((*edges)[i].a == b && (*edges)[i].b == a)
        {
            edges->erase(edges->begin() + i);
            return;
        }
    }

    edges->push_back(HullEdge{ a, b });
}

static bool HasPoint(std::span<const Vec3> points, const Vec3& p, float tolerance2)
{
    // Treat nearly coincident points as one point so the hull does not get tiny
    // duplicate faces from noisy input.
    for (const Vec3& point : points)
    {
        if (Dist2(point, p) <= tolerance2)
        {
            return true;
        }
    }

    return false;
}

static float DistanceToFace(std::span<const Vec3> points, const HullFace& face, int32 index)
{
    // Positive distance means the point is outside this outward-facing face.
    return Dot(face.normal, points[index] - points[face.indices[0]]);
}

static bool CreateFace(
    std::span<const Vec3> points, int32 a, int32 b, int32 c, const Vec3& inside, float tolerance, HullFace* outFace
)
{
    // Build an outward-facing triangle. The point inside the hull is used only to
    // decide whether the winding needs to be flipped.
    Vec3 normal = Cross(points[b] - points[a], points[c] - points[a]);
    float length = normal.Normalize();
    if (length <= tolerance)
    {
        return false;
    }

    if (Dot(normal, inside - points[a]) > 0.0f)
    {
        std::swap(b, c);
        normal = -normal;
    }

    outFace->indices[0] = a;
    outFace->indices[1] = b;
    outFace->indices[2] = c;
    outFace->normal = normal;
    return true;
}

static bool FindInitialSimplex(std::span<const Vec3> points, float tolerance, int32* outIndices)
{
    // Pick a stable initial tetrahedron by maximizing separation step by step:
    // point, line, triangle, then volume.
    Vec3 center = Vec3::zero;
    for (const Vec3& point : points)
    {
        center += point;
    }
    center *= 1.0f / points.size();

    int32 a = 0;
    float maxDistance2 = 0.0f;

    // Start with a point far from the average position.
    for (int32 i = 0; i < int32(points.size()); ++i)
    {
        float distance2 = Dist2(center, points[i]);
        if (distance2 > maxDistance2)
        {
            a = i;
            maxDistance2 = distance2;
        }
    }

    int32 b = a;
    maxDistance2 = 0.0f;
    // Pick the farthest point from the first point to form a long base edge.
    for (int32 i = 0; i < int32(points.size()); ++i)
    {
        float distance2 = Dist2(points[a], points[i]);
        if (distance2 > maxDistance2)
        {
            b = i;
            maxDistance2 = distance2;
        }
    }

    float edgeLength = std::sqrt(maxDistance2);
    if (edgeLength <= tolerance)
    {
        return false;
    }

    int32 c = a;
    float maxLineDistance = 0.0f;
    Vec3 ab = points[b] - points[a];
    // Pick the point farthest from the base edge to form a large triangle.
    for (int32 i = 0; i < int32(points.size()); ++i)
    {
        float distance = Length(Cross(ab, points[i] - points[a])) / edgeLength;
        if (distance > maxLineDistance)
        {
            c = i;
            maxLineDistance = distance;
        }
    }

    if (maxLineDistance <= tolerance)
    {
        return false;
    }

    Vec3 normal = Cross(points[b] - points[a], points[c] - points[a]);
    normal.Normalize();

    int32 d = a;
    float maxPlaneDistance = 0.0f;

    // Pick the point farthest from the triangle plane to give the tetrahedron volume.
    for (int32 i = 0; i < int32(points.size()); ++i)
    {
        float distance = Abs(Dot(normal, points[i] - points[a]));
        if (distance > maxPlaneDistance)
        {
            d = i;
            maxPlaneDistance = distance;
        }
    }

    if (maxPlaneDistance <= tolerance)
    {
        return false;
    }

    outIndices[0] = a;
    outIndices[1] = b;
    outIndices[2] = c;
    outIndices[3] = d;
    return true;
}

void ComputeConvexHull(
    std::span<const Vec3> points, std::vector<Vec3>* outVertices, std::vector<int32>* outIndices, std::vector<Face>* outFaces
)
{
    MuliAssert(outVertices != nullptr);
    MuliAssert(outFaces != nullptr);
    MuliAssert(outIndices != nullptr);

    outVertices->clear();
    outIndices->clear();
    outFaces->clear();

    float tolerance = 1e-4f;
    float tolerance2 = tolerance * tolerance;

    std::vector<Vec3> uniquePoints;
    uniquePoints.reserve(points.size());

    // The hull builder works on a deduplicated point set so nearly coincident input
    // points do not create duplicate faces or degenerate triangles.
    for (const Vec3& point : points)
    {
        if (!HasPoint(uniquePoints, point, tolerance2))
        {
            uniquePoints.push_back(point);
        }
    }

    if (uniquePoints.size() < 4)
    {
        return;
    }

    int32 count = int32(uniquePoints.size());

    // Start from a well separated tetrahedron, then expand it with the farthest point outside the current hull.
    int32 simplex[4];
    if (!FindInitialSimplex(uniquePoints, tolerance, simplex))
    {
        return;
    }

    Vec3 center = Vec3::zero;
    for (int32 index : simplex)
    {
        center += uniquePoints[index];
    }
    center *= 0.25f;

    // Create the initial hull from the tetrahedron faces.
    std::vector<HullFace> hullFaces;
    hullFaces.reserve(uniquePoints.size() * 2);

    HullFace face;
    if (CreateFace(uniquePoints, simplex[0], simplex[1], simplex[2], center, tolerance, &face))
    {
        hullFaces.push_back(face);
    }
    if (CreateFace(uniquePoints, simplex[0], simplex[3], simplex[1], center, tolerance, &face))
    {
        hullFaces.push_back(face);
    }
    if (CreateFace(uniquePoints, simplex[1], simplex[3], simplex[2], center, tolerance, &face))
    {
        hullFaces.push_back(face);
    }
    if (CreateFace(uniquePoints, simplex[2], simplex[3], simplex[0], center, tolerance, &face))
    {
        hullFaces.push_back(face);
    }

    std::vector<bool> isHullVertex(count, false);
    for (int32 index : simplex)
    {
        isHullVertex[index] = true;
    }

    while (true)
    {
        int32 bestFace = -1;
        int32 bestPoint = -1;
        float bestDistance = tolerance;

        // QuickHull expands by the point farthest outside any current face.
        for (int32 i = 0; i < int32(hullFaces.size()); ++i)
        {
            for (int32 j = 0; j < count; ++j)
            {
                if (isHullVertex[j])
                {
                    continue;
                }

                float distance = DistanceToFace(uniquePoints, hullFaces[i], j);
                if (distance > bestDistance)
                {
                    bestFace = i;
                    bestPoint = j;
                    bestDistance = distance;
                }
            }
        }

        if (bestPoint < 0)
        {
            break;
        }

        // Remove every face visible from the new point.
        // The guaranteed best face is marked visible even if it sits exactly on the tolerance boundary.
        std::vector<bool> visible(hullFaces.size(), false);
        for (int32 i = 0; i < int32(hullFaces.size()); ++i)
        {
            visible[i] = DistanceToFace(uniquePoints, hullFaces[i], bestPoint) > tolerance;
        }
        visible[bestFace] = true;

        // Opposite directed edges are internal to the removed patch.
        // The remaining boundary edges form the horizon where the new point is connected.
        std::vector<HullEdge> edges;
        for (int32 i = 0; i < int32(hullFaces.size()); ++i)
        {
            if (!visible[i])
            {
                continue;
            }

            const HullFace& hullFace = hullFaces[i];
            for (int32 j = 0; j < 3; ++j)
            {
                int32 a = hullFace.indices[j];
                int32 b = hullFace.indices[(j + 1) % 3];

                AddBoundaryEdge(&edges, a, b);
            }
        }

        // Keep the hidden faces and cap the open horizon with triangles to the new point.
        std::vector<HullFace> newFaces;
        newFaces.reserve(hullFaces.size() + edges.size());
        for (int32 i = 0; i < int32(hullFaces.size()); ++i)
        {
            if (!visible[i])
            {
                newFaces.push_back(hullFaces[i]);
            }
        }

        for (const HullEdge& edge : edges)
        {
            if (CreateFace(uniquePoints, edge.a, edge.b, bestPoint, center, tolerance, &face))
            {
                newFaces.push_back(face);
            }
        }

        hullFaces = std::move(newFaces);
        isHullVertex[bestPoint] = true;
    }

    if (hullFaces.empty())
    {
        return;
    }

    std::vector<int32> usedIndices;
    usedIndices.reserve(uniquePoints.size());

    // Discard input points that never became hull vertices and compact the indices.
    for (const HullFace& face : hullFaces)
    {
        for (int32 index : face.indices)
        {
            if (!Contains(usedIndices, index))
            {
                usedIndices.push_back(index);
            }
        }
    }

    std::sort(usedIndices.begin(), usedIndices.end());

    std::vector<int32> remap(count, -1);
    outVertices->reserve(usedIndices.size());
    for (int32 index : usedIndices)
    {
        remap[index] = int32(outVertices->size());
        outVertices->push_back(uniquePoints[index]);
    }

    // The hull is built from triangles because triangles make horizon updates simple.
    // Convert the finished triangle mesh into polygon faces
    // so clipping can use the complete flat surface instead of an arbitrary triangle on it.
    std::vector<int32> faceGroups(hullFaces.size(), -1);
    std::vector<int32> pendingFaces;
    int32 groupCount = 0;

    // Flood-fill each connected coplanar region.
    // Faces on the same plane but not connected by an edge must remain separate groups.
    for (int32 root = 0; root < int32(hullFaces.size()); ++root)
    {
        if (faceGroups[root] >= 0)
        {
            continue;
        }

        const HullFace& rootFace = hullFaces[root];
        const Vec3& planePoint = uniquePoints[rootFace.indices[0]];
        faceGroups[root] = groupCount;
        pendingFaces.clear();
        pendingFaces.push_back(root);

        // Always compare candidates against the root plane.
        // Comparing only with the current neighbor could merge a chain of gradually bending faces.
        while (!pendingFaces.empty())
        {
            int32 currentIndex = pendingFaces.back();
            pendingFaces.pop_back();
            const HullFace& currentFace = hullFaces[currentIndex];

            for (int32 candidateIndex = 0; candidateIndex < int32(hullFaces.size()); ++candidateIndex)
            {
                if (faceGroups[candidateIndex] >= 0)
                {
                    continue;
                }

                const HullFace& candidate = hullFaces[candidateIndex];

                // Coplanar faces have parallel outward normals. This also rejects
                // coincident faces with opposite winding.
                if (Dot(rootFace.normal, candidate.normal) < 1.0f - tolerance)
                {
                    continue;
                }

                // A matching normal alone is not enough because parallel faces can lie on different planes.
                // All three vertices must be within the hull tolerance of the root plane.
                bool coplanar = true;
                for (int32 index : candidate.indices)
                {
                    if (Abs(Dot(rootFace.normal, uniquePoints[index] - planePoint)) > tolerance)
                    {
                        coplanar = false;
                        break;
                    }
                }

                if (!coplanar)
                {
                    continue;
                }

                // Consistently wound adjacent triangles traverse their shared edge in opposite directions:
                // current a -> b, candidate b -> a.
                bool adjacent = false;
                for (int32 i = 0; i < 3 && !adjacent; ++i)
                {
                    int32 a = currentFace.indices[i];
                    int32 b = currentFace.indices[(i + 1) % 3];
                    for (int32 j = 0; j < 3; ++j)
                    {
                        if (candidate.indices[j] == b && candidate.indices[(j + 1) % 3] == a)
                        {
                            adjacent = true;
                            break;
                        }
                    }
                }

                if (adjacent)
                {
                    // Continue from the newly found face so the entire connected
                    // region is collected even when it does not touch the root.
                    faceGroups[candidateIndex] = groupCount;
                    pendingFaces.push_back(candidateIndex);
                }
            }
        }

        ++groupCount;
    }

    std::vector<HullEdge> boundaryEdges;
    std::vector<bool> usedEdges;

    for (int32 group = 0; group < groupCount; ++group)
    {
        boundaryEdges.clear();
        int32 firstFace = -1;

        // AddBoundaryEdge cancels opposite directed pairs.
        // Every triangulation edge occurs twice, while every polygon boundary edge occurs once,
        // so only one directed boundary loop remains for this group.
        for (int32 i = 0; i < int32(hullFaces.size()); ++i)
        {
            if (faceGroups[i] != group)
            {
                continue;
            }

            if (firstFace < 0)
            {
                firstFace = i;
            }

            const HullFace& hullFace = hullFaces[i];
            for (int32 j = 0; j < 3; ++j)
            {
                AddBoundaryEdge(&boundaryEdges, hullFace.indices[j], hullFace.indices[(j + 1) % 3]);
            }
        }

        Face face;
        face.vertexStart = uint16(outIndices->size());
        face.vertexCount = uint16(boundaryEdges.size());
        face.normal = hullFaces[firstFace].normal;

        // AddBoundaryEdge does not preserve loop order.
        // Follow matching endpoints to emit the polygon vertices in the original outward winding.
        usedEdges.assign(boundaryEdges.size(), false);
        int32 edgeIndex = 0;
        for (int32 i = 0; i < int32(boundaryEdges.size()); ++i)
        {
            const HullEdge& edge = boundaryEdges[edgeIndex];
            outIndices->push_back(remap[edge.a]);
            usedEdges[edgeIndex] = true;

            if (i + 1 == int32(boundaryEdges.size()))
            {
                // A closed convex face must end at the first boundary vertex.
                MuliAssert(edge.b == boundaryEdges[0].a);
                break;
            }

            int32 nextEdge = -1;
            for (int32 j = 0; j < int32(boundaryEdges.size()); ++j)
            {
                if (!usedEdges[j] && boundaryEdges[j].a == edge.b)
                {
                    nextEdge = j;
                    break;
                }
            }

            MuliAssert(nextEdge >= 0);
            edgeIndex = nextEdge;
        }

        outFaces->push_back(face);
    }
}

void ComputeConvexHull(std::span<const Vec2> vertices, std::vector<Vec2>* outVertices)
{
    MuliAssert(outVertices != nullptr);

    outVertices->clear();

    int32 vertexCount = int32(vertices.size());
    if (vertexCount == 0)
    {
        return;
    }

    std::vector<Vec2> sorted(vertices.begin(), vertices.end());
    std::sort(sorted.begin(), sorted.end(), [](const Vec2& a, const Vec2& b) {
        if (a.x == b.x)
        {
            return a.y < b.y;
        }
        return a.x < b.x;
    });

    sorted.erase(std::unique(sorted.begin(), sorted.end()), sorted.end());
    vertexCount = int32(sorted.size());
    if (vertexCount < 3)
    {
        *outVertices = sorted;
        return;
    }

    std::vector<Vec2> hull;
    hull.reserve(vertexCount * 2);

    for (int32 i = 0; i < vertexCount; ++i)
    {
        const Vec2& v = sorted[i];
        while (hull.size() >= 2)
        {
            Vec2 d1 = hull[hull.size() - 1] - hull[hull.size() - 2];
            Vec2 d2 = v - hull[hull.size() - 1];
            if (Cross(d1, d2) > 0.0f)
            {
                break;
            }
            hull.pop_back();
        }
        hull.push_back(v);
    }

    size_t lowerCount = hull.size();
    for (int32 i = vertexCount - 2; i >= 0; --i)
    {
        const Vec2& v = sorted[i];
        while (hull.size() > lowerCount)
        {
            Vec2 d1 = hull[hull.size() - 1] - hull[hull.size() - 2];
            Vec2 d2 = v - hull[hull.size() - 1];
            if (Cross(d1, d2) > 0.0f)
            {
                break;
            }
            hull.pop_back();
        }
        hull.push_back(v);
    }

    if (hull.size() > 1)
    {
        hull.pop_back();
    }

    *outVertices = std::move(hull);
}

} // namespace muli3
