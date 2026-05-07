#include "muli3/convex_shape.h"

namespace muli3
{

static Vec3 ClosestPointOnTriangle(const Vec3& a, const Vec3& b, const Vec3& c, const Vec3& p)
{
    // Voronoi-region based closest-point query used by point tests and
    // closest-point queries against triangulated hull faces.
    Vec3 ab = b - a;
    Vec3 ac = c - a;
    Vec3 ap = p - a;

    float d1 = Dot(ab, ap);
    float d2 = Dot(ac, ap);
    if (d1 <= 0.0f && d2 <= 0.0f)
    {
        return a;
    }

    Vec3 bp = p - b;
    float d3 = Dot(ab, bp);
    float d4 = Dot(ac, bp);
    if (d3 >= 0.0f && d4 <= d3)
    {
        return b;
    }

    float vc = d1 * d4 - d3 * d2;
    if (vc <= 0.0f && d1 >= 0.0f && d3 <= 0.0f)
    {
        float v = d1 / (d1 - d3);
        return a + ab * v;
    }

    Vec3 cp = p - c;
    float d5 = Dot(ab, cp);
    float d6 = Dot(ac, cp);
    if (d6 >= 0.0f && d5 <= d6)
    {
        return c;
    }

    float vb = d5 * d2 - d1 * d6;
    if (vb <= 0.0f && d2 >= 0.0f && d6 <= 0.0f)
    {
        float w = d2 / (d2 - d6);
        return a + ac * w;
    }

    float va = d3 * d6 - d5 * d4;
    if (va <= 0.0f && d4 - d3 >= 0.0f && d5 - d6 >= 0.0f)
    {
        Vec3 bc = c - b;
        float w = (d4 - d3) / ((d4 - d3) + (d5 - d6));
        return b + bc * w;
    }

    float denom = 1.0f / (va + vb + vc);
    float v = vb * denom;
    float w = vc * denom;
    return a + ab * v + ac * w;
}

static void FixFaceWinding(std::span<const Vec3> vertices, std::vector<ConvexFace>* faces)
{
    // After an arbitrary transform, especially one with negative scale, the face
    // orientation can flip. Re-orient all triangles so their normals point outward.
    Vec3 inside = Vec3::zero;
    for (const Vec3& vertex : vertices)
    {
        inside += vertex;
    }
    inside *= 1.0f / vertices.size();

    for (ConvexFace& face : *faces)
    {
        Vec3 a = vertices[face.indices[0]];
        Vec3 b = vertices[face.indices[1]];
        Vec3 c = vertices[face.indices[2]];

        Vec3 normal = Cross(b - a, c - a);
        if (Dot(normal, inside - a) > 0.0f)
        {
            std::reverse(face.indices, face.indices + face.count);
        }
    }
}

static void ComputeFaceNormals(std::span<const Vec3> vertices, std::span<const ConvexFace> faces, std::vector<Vec3>* normals)
{
    normals->resize(faces.size());

    // ConvexShape stores triangle faces, so each face normal is just the normalized
    // cross product of one triangle.
    for (size_t i = 0; i < faces.size(); ++i)
    {
        const ConvexFace& face = faces[i];
        Vec3 a = vertices[face.indices[0]];
        Vec3 b = vertices[face.indices[1]];
        Vec3 c = vertices[face.indices[2]];

        Vec3 normal = Cross(b - a, c - a);
        normal.Normalize();
        (*normals)[i] = normal;
    }
}

static void ComputeMassProperties(
    std::span<const Vec3> vertices, std::span<const ConvexFace> faces, float* outVolume, Vec3* outCenter, Mat3* outInertia
)
{
    // Integrate the closed hull as a set of signed tetrahedra against the origin.
    // The resulting inertia tensor is expressed about the local origin and later
    // shifted by the rigid body mass aggregation path if needed.
    float volume = 0.0f;
    Vec3 center = Vec3::zero;

    float xx = 0.0f;
    float yy = 0.0f;
    float zz = 0.0f;
    float xy = 0.0f;
    float yz = 0.0f;
    float zx = 0.0f;

    for (const ConvexFace& face : faces)
    {
        const Vec3& a = vertices[face.indices[0]];

        for (int32 i = 1; i + 1 < face.count; ++i)
        {
            const Vec3& b = vertices[face.indices[i]];
            const Vec3& c = vertices[face.indices[i + 1]];

            float det = Dot(a, Cross(b, c));
            float tetraVolume = det / 6.0f;

            volume += tetraVolume;
            center += tetraVolume * (a + b + c);

            xx += det * (a.x * a.x + b.x * b.x + c.x * c.x + a.x * b.x + b.x * c.x + c.x * a.x) / 60.0f;
            yy += det * (a.y * a.y + b.y * b.y + c.y * c.y + a.y * b.y + b.y * c.y + c.y * a.y) / 60.0f;
            zz += det * (a.z * a.z + b.z * b.z + c.z * c.z + a.z * b.z + b.z * c.z + c.z * a.z) / 60.0f;

            xy += det *
                  (2.0f * (a.x * a.y + b.x * b.y + c.x * c.y) + a.x * b.y + a.y * b.x + a.x * c.y + a.y * c.x + b.x * c.y +
                   b.y * c.x) /
                  120.0f;
            yz += det *
                  (2.0f * (a.y * a.z + b.y * b.z + c.y * c.z) + a.y * b.z + a.z * b.y + a.y * c.z + a.z * c.y + b.y * c.z +
                   b.z * c.y) /
                  120.0f;
            zx += det *
                  (2.0f * (a.z * a.x + b.z * b.x + c.z * c.x) + a.z * b.x + a.x * b.z + a.z * c.x + a.x * c.z + b.z * c.x +
                   b.x * c.z) /
                  120.0f;
        }
    }

    if (volume < 0.0f)
    {
        volume = -volume;
        center = -center;
        xx = -xx;
        yy = -yy;
        zz = -zz;
        xy = -xy;
        yz = -yz;
        zx = -zx;
    }

    if (volume <= epsilon)
    {
        *outVolume = 0.0f;
        *outCenter = Vec3::zero;
        *outInertia = Mat3::zero;
        return;
    }

    center *= 1.0f / (4.0f * volume);

    *outVolume = volume;
    *outCenter = center;
    *outInertia = Mat3(Vec3{ yy + zz, -xy, -zx }, Vec3{ -xy, xx + zz, -yz }, Vec3{ -zx, -yz, xx + yy });
}

ConvexShape::ConvexShape(std::span<const Vec3> inVertices, float inRadius, const Transform& transform)
    : Shape{ Shape::convex, inRadius }
    , inertia{ 0.0f }
{
    ComputeConvexHull(inVertices, &vertices, &faces);

    MuliAssert(vertices.size() >= 4);
    MuliAssert(faces.size() > 0);

    for (Vec3& vertex : vertices)
    {
        vertex = Mul(transform, vertex);
    }

    FixFaceWinding(vertices, &faces);
    ComputeFaceNormals(vertices, faces, &normals);
    ComputeMassProperties(vertices, faces, &volume, &center, &inertia);
}

ConvexShape::ConvexShape(const ConvexShape& other, const Transform& transform)
    : ConvexShape(other.vertices, other.radius, transform)
{
}

void ConvexShape::ComputeMass(float density, MassData* outMassData) const
{
    MuliAssert(outMassData != nullptr);

    outMassData->mass = density * volume;
    outMassData->centerOfMass = center;
    outMassData->inertia = Mat3(inertia.ex * density, inertia.ey * density, inertia.ez * density);
}

void ConvexShape::ComputeAABB(const Transform& transform, AABB* outAABB) const
{
    MuliAssert(outAABB != nullptr);

    Vec3 min = Mul(transform, vertices[0]);
    Vec3 max = min;

    for (int32 i = 1; i < int32(vertices.size()); ++i)
    {
        Vec3 v = Mul(transform, vertices[i]);
        min = Min(min, v);
        max = Max(max, v);
    }

    float scaledRadius = radius * Max(Abs(transform.s.x), Max(Abs(transform.s.y), Abs(transform.s.z)));
    Vec3 r{ scaledRadius, scaledRadius, scaledRadius };
    *outAABB = AABB{ min - r, max + r };
}

int32 ConvexShape::GetSupport(const Vec3& localDir) const
{
    // The hull is stored explicitly as vertices, so support is a simple max dot scan.
    int32 index = 0;
    float maxValue = Dot(localDir, vertices[0]);

    for (int32 i = 1; i < int32(vertices.size()); ++i)
    {
        float value = Dot(localDir, vertices[i]);
        if (value > maxValue)
        {
            index = i;
            maxValue = value;
        }
    }

    return index;
}

Face ConvexShape::GetFeaturedFace(const Transform& transform, const Vec3& dir) const
{
    // Contact clipping wants the face whose normal is most aligned with the query
    // direction in local space.
    int32 index = 0;
    float maxValue = Dot(normals[0], transform.q.RotateInv(dir));

    for (int32 i = 1; i < int32(faces.size()); ++i)
    {
        float value = Dot(normals[i], transform.q.RotateInv(dir));
        if (value > maxValue)
        {
            index = i;
            maxValue = value;
        }
    }

    Face face;
    face.count = faces[index].count;
    face.normal = transform.q.Rotate(normals[index]);

    for (int32 i = 0; i < face.count; ++i)
    {
        int32 vertexIndex = faces[index].indices[i];
        face.points[i].id = vertexIndex;
        face.points[i].p = Mul(transform, vertices[vertexIndex]);
    }

    return face;
}

bool ConvexShape::TestPointLocal(const Vec3& q) const
{
    // For a closed convex polyhedron, the point is inside iff it lies behind every
    // outward face plane.
    for (int32 i = 0; i < int32(faces.size()); ++i)
    {
        if (Dot(normals[i], q - vertices[faces[i].indices[0]]) > 0.0f)
        {
            return false;
        }
    }

    return true;
}

bool ConvexShape::TestPoint(const Transform& transform, const Vec3& q) const
{
    Vec3 localQ = MulT(transform, q);
    if (TestPointLocal(localQ))
    {
        return true;
    }

    Vec3 closest = GetClosestPointLocal(localQ);
    return Dist2(localQ, closest) <= radius * radius;
}

Vec3 ConvexShape::GetClosestPointLocal(const Vec3& q) const
{
    if (TestPointLocal(q))
    {
        return q;
    }

    // Outside the hull, the closest point must lie on one of the surface triangles.
    Vec3 closest = vertices[0];
    float minDistance2 = max_float;

    for (const ConvexFace& face : faces)
    {
        const Vec3& a = vertices[face.indices[0]];

        for (int32 i = 1; i + 1 < face.count; ++i)
        {
            const Vec3& b = vertices[face.indices[i]];
            const Vec3& c = vertices[face.indices[i + 1]];

            Vec3 p = ClosestPointOnTriangle(a, b, c, q);
            float distance2 = Dist2(q, p);
            if (distance2 < minDistance2)
            {
                closest = p;
                minDistance2 = distance2;
            }
        }
    }

    return closest;
}

Vec3 ConvexShape::GetClosestPoint(const Transform& transform, const Vec3& q) const
{
    Vec3 localQ = MulT(transform, q);
    Vec3 closest = GetClosestPointLocal(localQ);
    Vec3 delta = localQ - closest;

    float distance = delta.Normalize();
    if (distance <= radius)
    {
        return q;
    }

    return Mul(transform, closest + delta * radius);
}

bool ConvexShape::RayCast(const Transform& transform, const RayCastInput& input, RayCastOutput* output) const
{
    RayCastInput localInput = input;
    localInput.from = MulT(transform, input.from);
    localInput.to = MulT(transform, input.to);

    // Convex ray casting is a slab-style intersection against all face planes,
    // expanded by the shape radius and the swept sphere radius.
    float radii = radius + input.radius;
    Vec3 p1 = localInput.from;
    Vec3 closest = GetClosestPointLocal(p1);
    if (TestPointLocal(p1) || Dist2(p1, closest) <= radii * radii)
    {
        return false;
    }

    Vec3 d = localInput.to - localInput.from;

    float near = 0.0f;
    float far = input.maxFraction;
    int32 index = -1;

    for (int32 i = 0; i < int32(faces.size()); ++i)
    {
        Vec3 normal = normals[i];
        Vec3 v = vertices[faces[i].indices[0]] + normal * radii;

        float numerator = Dot(normal, v - p1);
        float denominator = Dot(normal, d);

        if (Abs(denominator) <= epsilon)
        {
            if (numerator < 0.0f)
            {
                return false;
            }
        }
        else
        {
            if (denominator < 0.0f && numerator < near * denominator)
            {
                near = numerator / denominator;
                index = i;
            }
            else if (denominator > 0.0f && numerator < far * denominator)
            {
                far = numerator / denominator;
            }
        }

        if (far < near)
        {
            return false;
        }
    }

    if (index < 0 || near < 0.0f || near > input.maxFraction)
    {
        return false;
    }

    output->fraction = near;
    output->normal = transform.q.Rotate(normals[index]);
    return true;
}

} // namespace muli3
