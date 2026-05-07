#include "muli3/geometry.h"
#include "muli3/frame.h"
#include "muli3/settings.h"

namespace muli3
{

struct HullFace
{
    std::vector<int32> indices;
    Vec3 normal;
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

static bool HasFace(std::span<const HullFace> faces, std::span<const int32> indices)
{
    for (const HullFace& face : faces)
    {
        if (face.indices.size() != indices.size())
        {
            continue;
        }

        bool same = true;
        for (size_t i = 0; i < indices.size(); ++i)
        {
            if (face.indices[i] != indices[i])
            {
                same = false;
                break;
            }
        }

        if (same)
        {
            return true;
        }
    }

    return false;
}

static bool HasPoint(std::span<const Vec3> points, const Vec3& p, float tolerance2)
{
    for (const Vec3& point : points)
    {
        if (Dist2(point, p) <= tolerance2)
        {
            return true;
        }
    }

    return false;
}

static void OrderFace(std::span<const Vec3> points, std::vector<int32>* indices, const Vec3& normal)
{
    // Collapse a coplanar point set into a stable polygon loop so the face can be
    // triangulated later without relying on the input order.
    Vec3 center = Vec3::zero;
    for (int32 index : *indices)
    {
        center += points[index];
    }
    center *= 1.0f / indices->size();

    Vec3 tangent;
    Vec3 bitangent;
    CoordinateSystem(normal, &tangent, &bitangent);

    std::sort(indices->begin(), indices->end(), [&](int32 a, int32 b) {
        Vec3 da = points[a] - center;
        Vec3 db = points[b] - center;

        float angleA = std::atan2(Dot(da, bitangent), Dot(da, tangent));
        float angleB = std::atan2(Dot(db, bitangent), Dot(db, tangent));
        return angleA < angleB;
    });

    if (indices->size() >= 3)
    {
        Vec3 a = points[(*indices)[0]];
        Vec3 b = points[(*indices)[1]];
        Vec3 c = points[(*indices)[2]];

        if (Dot(Cross(b - a, c - a), normal) < 0.0f)
        {
            std::reverse(indices->begin(), indices->end());
        }
    }
}

void ComputeConvexHull(std::span<const Vec3> points, std::vector<Vec3>* outVertices, std::vector<ConvexFace>* outFaces)
{
    MuliAssert(outVertices != nullptr);
    MuliAssert(outFaces != nullptr);

    outVertices->clear();
    outFaces->clear();

    float tolerance = linear_slop * 0.1f;
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

    std::vector<HullFace> hullFaces;

    int32 count = int32(uniquePoints.size());
    for (int32 i = 0; i < count - 2; ++i)
    {
        for (int32 j = i + 1; j < count - 1; ++j)
        {
            for (int32 k = j + 1; k < count; ++k)
            {
                Vec3 a = uniquePoints[i];
                Vec3 b = uniquePoints[j];
                Vec3 c = uniquePoints[k];

                Vec3 normal = Cross(b - a, c - a);
                float normalLength = normal.Normalize();
                if (normalLength <= tolerance)
                {
                    continue;
                }

                int32 sign = 0;
                std::vector<int32> faceIndices;
                faceIndices.reserve(uniquePoints.size());

                // A valid hull face has all non-coplanar points strictly on one side
                // of the candidate plane, while coplanar points belong to the face.
                for (int32 l = 0; l < count; ++l)
                {
                    float distance = Dot(normal, uniquePoints[l] - a);

                    if (distance > tolerance)
                    {
                        if (sign < 0)
                        {
                            sign = 0;
                            break;
                        }

                        sign = 1;
                    }
                    else if (distance < -tolerance)
                    {
                        if (sign > 0)
                        {
                            sign = 0;
                            break;
                        }

                        sign = -1;
                    }
                    else
                    {
                        faceIndices.push_back(l);
                    }
                }

                if (sign == 0)
                {
                    continue;
                }

                if (sign > 0)
                {
                    normal = -normal;
                }

                // The same coplanar face can be discovered from many different point
                // triplets, so reject duplicates before ordering the polygon loop.
                std::sort(faceIndices.begin(), faceIndices.end());
                if (HasFace(hullFaces, faceIndices))
                {
                    continue;
                }

                OrderFace(uniquePoints, &faceIndices, normal);
                hullFaces.push_back(HullFace{ std::move(faceIndices), normal });
            }
        }
    }

    if (hullFaces.empty())
    {
        return;
    }

    std::vector<int32> usedIndices;
    usedIndices.reserve(uniquePoints.size());

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

    // The rest of the engine currently expects triangle faces, so polygonal hull
    // faces are emitted as a simple triangle fan.
    for (const HullFace& hullFace : hullFaces)
    {
        int32 faceVertexCount = int32(hullFace.indices.size());
        MuliAssert(faceVertexCount >= 3);

        for (int32 i = 1; i + 1 < faceVertexCount; ++i)
        {
            ConvexFace face;
            face.count = 3;
            face.indices[0] = remap[hullFace.indices[0]];
            face.indices[1] = remap[hullFace.indices[i]];
            face.indices[2] = remap[hullFace.indices[i + 1]];
            outFaces->push_back(face);
        }
    }
}

} // namespace muli3
