#include "muli3/polygon_shape.h"
#include "muli3/distance.h"
#include "muli3/hash.h"
#include "muli3/settings.h"

namespace muli3
{

PolygonShape::PolygonShape(std::span<const Vec3> inVertices, float inRadius, const Transform& transform)
    : Shape(Shape::polygon, inRadius)
{
    MuliAssert(inVertices.size() >= 3);
    MuliAssert(inVertices.size() <= std::numeric_limits<uint16>::max());

    vertices.reserve(inVertices.size());
    for (const Vec3& vertex : inVertices)
    {
        vertices.push_back(Mul(transform, vertex));
    }

    normal = Vec3::zero;
    for (int32 i = 1; i + 1 < int32(vertices.size()); ++i)
    {
        normal = Cross(vertices[i] - vertices[0], vertices[i + 1] - vertices[0]);
        if (normal.Normalize() > epsilon)
        {
            break;
        }
    }

    int32 count = int32(vertices.size());
    indices.reserve(2 * count);
    for (int32 i = 0; i < count; ++i)
    {
        indices.push_back(i);
    }
    indices.push_back(0);
    for (int32 i = count - 1; i > 0; --i)
    {
        indices.push_back(i);
    }

    if (radius == 0.0f)
    {
        float area = 0.0f;
        Vec3 first = Vec3::zero;
        for (int32 i = 1; i + 1 < count; ++i)
        {
            float triangleArea = 0.5f * Length(Cross(vertices[i] - vertices[0], vertices[i + 1] - vertices[0]));
            area += triangleArea;
            first += (vertices[0] + vertices[i] + vertices[i + 1]) * (triangleArea / 3.0f);
        }

        center = area > epsilon ? first / area : vertices[0];
        volume = 0.0f;
    }
    else
    {
        MassData massData;
        ComputeMass(1.0f, &massData);

        center = massData.centerOfMass;
        volume = massData.mass;
    }

    // Compute geometry hash
    hash = Hash(uint32(Shape::polygon), uint32(vertices.size()));
    hash = HashBuffer(vertices.data(), vertices.size() * sizeof(Vec3), hash);
}

PolygonShape::PolygonShape(const PolygonShape& other, const Transform& transform)
    : PolygonShape(other.vertices, other.radius, transform)
{
}

void PolygonShape::ComputeMass(float density, MassData* outMassData) const
{
    MuliAssert(outMassData != nullptr);

    // Rounded polygon is the Minkowski sum of the core polygon P and a sphere of radius r:
    //
    //   Q = P (+) B_r = { p + q | p in P, |q| <= r }.
    //
    // For a convex flat polygon:
    //
    //   V = 2*r*A + (pi*r^2/2)*L + (4*pi*r^3/3)
    //
    // A is the polygon area and L is the perimeter.  The terms are the face slab,
    // edge half-cylinders, and vertex spherical sectors.

    const auto Outer = [](const Vec3& a, const Vec3& b) { return Mat3(a * b.x, a * b.y, a * b.z); };
    const auto Mul = [](const Mat3& m, float s) { return Mat3(m.ex * s, m.ey * s, m.ez * s); };
    const auto Add = [](Mat3* a, const Mat3& b) { *a = *a + b; };

    Vec3 n = normal;
    if (Length2(n) <= epsilon)
    {
        outMassData->mass = 0.0f;
        outMassData->inertia = Mat3::zero;
        outMassData->centerOfMass = vertices[0];
        return;
    }

    int32 count = int32(vertices.size());
    float r = radius;
    float r2 = r * r;
    float r3 = r2 * r;
    float r4 = r2 * r2;
    float r5 = r4 * r;

    float mass = 0.0f;
    Vec3 first = Vec3::zero;
    Mat3 second = Mat3::zero;

    for (int32 i = 1; i + 1 < count; ++i)
    {
        Vec3 a = vertices[0];
        Vec3 b = vertices[i];
        Vec3 c = vertices[i + 1];
        float area = 0.5f * Length(Cross(b - a, c - a));
        if (area <= epsilon)
        {
            continue;
        }

        // Face slab.  The polygon face is triangulated, then each triangle is
        // extruded along the polygon normal by [-r, r].
        //
        // M1 = int_triangle int_-r^r (p + t*n) dt dA
        //    = 2r * A * (a+b+c)/3
        //
        // M2 = 2r * int_triangle p*p^T dA + (2r^3/3) * A * n*n^T
        // int_triangle p*p^T dA = A/12 * (aa^T + bb^T + cc^T + ss^T), s=a+b+c
        Vec3 sum = a + b + c;
        Mat3 triangleSecond = Mul(Outer(a, a) + Outer(b, b) + Outer(c, c) + Outer(sum, sum), area / 12.0f);

        mass += 2.0f * r * area;
        first += sum * (area * 2.0f * r / 3.0f);
        Add(&second, Mul(triangleSecond, 2.0f * r));
        Add(&second, Mul(Outer(n, n), area * 2.0f * r3 / 3.0f));
    }

    for (int32 i = 0; i < count; ++i)
    {
        Vec3 p0 = vertices[i];
        Vec3 p1 = vertices[(i + 1) % count];
        Vec3 edge = p1 - p0;

        float L = edge.Normalize();
        if (L == 0.0f)
        {
            continue;
        }

        Vec3 o = Cross(edge, n);

        // Edge half-cylinder. For edge direction e and outward in-plane normal o:
        //
        // x = p(s) + y*o + z*n, p(s)=p0+e*s, s in [0,L]
        // D = { (y,z) | y >= 0, y^2+z^2 <= r^2 }
        //
        // int_D dA      = pi*r^2/2
        // int_D y dA    = 2*r^3/3
        // int_D y^2 dA  = int_D z^2 dA = pi*r^4/8
        //
        // M1 = A_hd * int_0^L p(s) ds + o * L * int_D y dA
        //
        // M2 = A_hd * int_0^L pp^T ds
        //    + (int_D y dA) * (segmentFirst*o^T + o*segmentFirst^T)
        //    + L*(int_D y^2 dA)*o*o^T + L*(int_D z^2 dA)*n*n^T
        float halfDiskArea = 0.5f * pi * r2;
        float halfDiskFirst = 2.0f * r3 / 3.0f;
        float halfDiskSecond = pi * r4 / 8.0f;

        Vec3 segmentFirst = (p0 + p1) * (0.5f * L);
        Mat3 segmentSecond = Mul(Outer(p0, p0) + Outer(p1, p1), L / 3.0f) + Mul(Outer(p0, p1) + Outer(p1, p0), L / 6.0f);

        mass += L * halfDiskArea;
        first += segmentFirst * halfDiskArea + o * (L * halfDiskFirst);

        Add(&second, Mul(segmentSecond, halfDiskArea));
        Add(&second, Mul(Outer(segmentFirst, o) + Outer(o, segmentFirst), halfDiskFirst));
        Add(&second, Mul(Outer(o, o), L * halfDiskSecond));
        Add(&second, Mul(Outer(n, n), L * halfDiskSecond));
    }

    for (int32 i = 0; i < count; ++i)
    {
        Vec3 prev = vertices[(i + count - 1) % count];
        Vec3 p = vertices[i];
        Vec3 next = vertices[(i + 1) % count];

        Vec3 edge0 = p - prev;
        Vec3 edge1 = next - p;
        if (edge0.Normalize() == 0.0f || edge1.Normalize() == 0.0f)
        {
            continue;
        }

        Vec3 outward0 = Cross(edge0, n);
        Vec3 outward1 = Cross(edge1, n);
        float angle = std::acos(Clamp(Dot(outward0, outward1), -1.0f, 1.0f));
        if (angle <= epsilon)
        {
            continue;
        }

        Vec3 bisector = NormalizeSafe(outward0 + outward1);
        Vec3 tangent = Cross(n, bisector);
        if (Length2(bisector) <= epsilon)
        {
            bisector = outward0;
            tangent = Cross(n, bisector);
        }

        // Vertex spherical sector.  The outside angle between adjacent edge
        // half-cylinders defines a sector in the local frame {b,t,n}:
        //
        // q = u*b + v*t + w*n, x = p + q
        // phi in [-angle/2, angle/2], u=s*cos(phi), v=s*sin(phi), s^2+w^2 <= r^2
        //
        // int dV   = 2*angle*r^3/3
        // int u dV = pi*r^4*sin(angle/2)/4
        //
        // int u^2 dV = 2*r^5*(angle+sin(angle))/15
        // int v^2 dV = 2*r^5*(angle-sin(angle))/15
        // int w^2 dV = 2*angle*r^5/15
        //
        // M2 is translated from the vertex frame by x = p + q.
        float sectorVolume = 2.0f * angle * r3 / 3.0f;
        float sectorFirst = pi * r4 * std::sin(angle * 0.5f) / 4.0f;

        float i0 = 2.0f * r5 * (angle + std::sin(angle)) / 15.0f;
        float i1 = 2.0f * r5 * (angle - std::sin(angle)) / 15.0f;
        float i2 = 2.0f * angle * r5 / 15.0f;

        Vec3 relativeFirst = bisector * sectorFirst;
        Mat3 relativeSecond = Mul(Outer(bisector, bisector), i0) + Mul(Outer(tangent, tangent), i1) + Mul(Outer(n, n), i2);

        mass += sectorVolume;
        first += p * sectorVolume + relativeFirst;
        Add(&second, Mul(Outer(p, p), sectorVolume));
        Add(&second, Outer(p, relativeFirst) + Outer(relativeFirst, p));
        Add(&second, relativeSecond);
    }

    mass *= density;
    first *= density;
    second.ex *= density;
    second.ey *= density;
    second.ez *= density;

    outMassData->mass = mass;
    outMassData->centerOfMass = mass > epsilon ? first / mass : vertices[0];

    float trace = second.Trace();
    outMassData->inertia = Mat3{
        Vec3{ trace - second.ex.x, -second.ex.y, -second.ex.z },
        Vec3{ -second.ey.x, trace - second.ey.y, -second.ey.z },
        Vec3{ -second.ez.x, -second.ez.y, trace - second.ez.z },
    };
}

void PolygonShape::ComputeAABB(const Transform& transform, AABB* outAABB) const
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

    *outAABB = AABB{ min - radius, max + radius };
}

int32 PolygonShape::GetSupport(const Vec3& localDir) const
{
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

Face PolygonShape::GetFeaturedFace(const Transform& transform, const Vec3& dir) const
{
    Face face{};
    face.vertexCount = int32(vertices.size());

    Vec3 worldNormal = transform.q.Rotate(normal);
    if (Dot(worldNormal, dir) >= 0.0f)
    {
        face.vertexStart = 0;
        face.normal = worldNormal;
    }
    else
    {
        face.vertexStart = int32(vertices.size());
        face.normal = -worldNormal;
    }

    return face;
}

bool PolygonShape::TestPoint(const Transform& transform, const Vec3& q) const
{
    Vec3 closest = GetClosestPoint(transform, q);
    return Dist2(closest, q) <= Sqr(linear_slop);
}

Vec3 PolygonShape::GetClosestPointLocal(const Vec3& q) const
{
    Vec3 closest = vertices[0];
    float minDistance2 = max_float;

    for (int32 i = 1; i + 1 < int32(vertices.size()); ++i)
    {
        Vec3 p = ClosestPointVsTriangle(q, vertices[0], vertices[i], vertices[i + 1]);
        float distance2 = Dist2(q, p);
        if (distance2 < minDistance2)
        {
            closest = p;
            minDistance2 = distance2;
        }
    }

    return closest;
}

Vec3 PolygonShape::GetClosestPoint(const Transform& transform, const Vec3& q) const
{
    Vec3 localQ = MulT(transform, q);
    Vec3 closest = GetClosestPointLocal(localQ);

    Vec3 normal = localQ - closest;
    float distance = normal.Normalize();
    if (distance <= radius)
    {
        return q;
    }

    return Mul(transform, closest + normal * radius);
}

bool PolygonShape::RayCast(const Transform& transform, const RayCastInput& input, RayCastOutput* output) const
{
    bool hit = false;
    RayCastInput rayInput = input;
    RayCastOutput candidate;

    Vec3 a = Mul(transform, vertices[0]);
    for (int32 i = 1; i + 1 < int32(vertices.size()); ++i)
    {
        Vec3 b = Mul(transform, vertices[i]);
        Vec3 c = Mul(transform, vertices[i + 1]);

        if (RayCastTriangle(a, b, c, rayInput, &candidate))
        {
            *output = candidate;
            rayInput.maxFraction = output->fraction;
            hit = true;
        }
    }

    return hit;
}

} // namespace muli3
