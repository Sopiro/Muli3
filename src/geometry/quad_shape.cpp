#include "muli3/quad_shape.h"
#include "muli3/distance.h"
#include "muli3/settings.h"

namespace muli3
{

QuadShape::QuadShape(const Vec3& a, const Vec3& b, const Vec3& c, const Vec3& d, float inRadius, const Transform& transform)
    : Shape(Shape::quad, inRadius)
{
    vertices[0] = Mul(transform, a);
    vertices[1] = Mul(transform, b);
    vertices[2] = Mul(transform, c);
    vertices[3] = Mul(transform, d);

    normal = Cross(vertices[1] - vertices[0], vertices[2] - vertices[0]);
    normal.Normalize();

    if (radius == 0.0f)
    {
        Vec3 areaNormal0 = Cross(vertices[1] - vertices[0], vertices[2] - vertices[0]);
        Vec3 areaNormal1 = Cross(vertices[2] - vertices[0], vertices[3] - vertices[0]);

        float area0 = 0.5f * Length(areaNormal0);
        float area1 = 0.5f * Length(areaNormal1);
        float area = area0 + area1;

        center = area > epsilon ? ((vertices[0] + vertices[1] + vertices[2]) * (area0 / 3.0f) +
                                   (vertices[0] + vertices[2] + vertices[3]) * (area1 / 3.0f)) /
                                      area
                                : (vertices[0] + vertices[1] + vertices[2] + vertices[3]) * 0.25f;
        volume = 0.0f;
    }
    else
    {
        MassData massData;
        ComputeMass(1.0f, &massData);

        center = massData.centerOfMass;
        volume = massData.mass;
    }
}

QuadShape::QuadShape(const QuadShape& other, const Transform& transform)
    : QuadShape(other.vertices[0], other.vertices[1], other.vertices[2], other.vertices[3], other.radius, transform)
{
}

void QuadShape::ComputeMass(float density, MassData* outMassData) const
{
    MuliAssert(outMassData != nullptr);

    // Rounded quad is the Minkowski sum of the core quadrilateral P and a sphere of radius r:
    //
    //   Q = P (+) B_r = { p + q | p in P, |q| <= r }.
    //
    // For a convex flat polygon:
    //
    //   V = 2*r*A + (pi*r^2/2)*L + (4*pi*r^3/3)
    //
    // A is the quad area and L is the perimeter.  The terms are the face slab,
    // edge half-cylinders, and vertex spherical sectors.  The implementation
    // accumulates:
    //
    //   M0 = int dV
    //   M1 = int x dV
    //   M2 = int x*x^T dV
    //
    // and converts M2 to inertia at the end:
    //
    //   I = int (dot(x,x)*Identity - x*x^T) dm = tr(M2)*Identity - M2.

    const auto Outer = [](const Vec3& a, const Vec3& b) { return Mat3(a * b.x, a * b.y, a * b.z); };
    const auto Mul = [](const Mat3& m, float s) { return Mat3(m.ex * s, m.ey * s, m.ez * s); };
    const auto Add = [](Mat3* a, const Mat3& b) { *a = *a + b; };

    Vec3 n = normal;
    if (Length2(n) <= epsilon)
    {
        outMassData->mass = 0.0f;
        outMassData->inertia = Mat3::zero;
        outMassData->centerOfMass = (vertices[0] + vertices[1] + vertices[2] + vertices[3]) * 0.25f;
        return;
    }

    float r = radius;
    float r2 = r * r;
    float r3 = r2 * r;
    float r4 = r2 * r2;
    float r5 = r4 * r;

    float mass = 0.0f;
    Vec3 first = Vec3::zero;
    Mat3 second = Mat3::zero;

    Vec3 triangleVertices[2][3] = {
        { vertices[0], vertices[1], vertices[2] },
        { vertices[0], vertices[2], vertices[3] },
    };

    for (int32 i = 0; i < 2; ++i)
    {
        Vec3 a = triangleVertices[i][0];
        Vec3 b = triangleVertices[i][1];
        Vec3 c = triangleVertices[i][2];
        float area = 0.5f * Length(Cross(b - a, c - a));
        if (area <= epsilon)
        {
            continue;
        }

        // Face slab.  The quad face is triangulated, then each triangle is
        // extruded along the quad normal by [-r, r].
        //
        // M1 = int_triangle int_-r^r (p + t*n) dt dA
        //    = 2r * A * (a+b+c)/3
        //
        // M2 = 2r * int_triangle p*p^T dA + (2r^3/3) * A * n*n^T
        // int_triangle p*p^T dA = A/12 * (aa^T + bb^T + cc^T + ss^T), s=a+b+c
        Vec3 sum = a + b + c;
        Mat3 triangleSecond = Mul(Outer(a, a) + Outer(b, b) + Outer(c, c) + Outer(sum, sum), area / 12.0f);

        mass += 2.0f * r * area;
        first = first + sum * (area * 2.0f * r / 3.0f);
        Add(&second, Mul(triangleSecond, 2.0f * r));
        Add(&second, Mul(Outer(n, n), area * 2.0f * r3 / 3.0f));
    }

    for (int32 i = 0; i < 4; ++i)
    {
        Vec3 p0 = vertices[i];
        Vec3 p1 = vertices[(i + 1) % 4];
        Vec3 edge = p1 - p0;

        float L = edge.Normalize();
        if (L == 0.0f)
        {
            continue;
        }

        Vec3 o = Cross(edge, n);

        // Edge half-cylinder.  For edge direction e and outward in-plane normal o:
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
        first = first + segmentFirst * halfDiskArea + o * (L * halfDiskFirst);

        Add(&second, Mul(segmentSecond, halfDiskArea));
        Add(&second, Mul(Outer(segmentFirst, o) + Outer(o, segmentFirst), halfDiskFirst));
        Add(&second, Mul(Outer(o, o), L * halfDiskSecond));
        Add(&second, Mul(Outer(n, n), L * halfDiskSecond));
    }

    for (int32 i = 0; i < 4; ++i)
    {
        Vec3 prev = vertices[(i + 3) % 4];
        Vec3 p = vertices[i];
        Vec3 next = vertices[(i + 1) % 4];

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
        first = first + p * sectorVolume + relativeFirst;
        Add(&second, Mul(Outer(p, p), sectorVolume));
        Add(&second, Outer(p, relativeFirst) + Outer(relativeFirst, p));
        Add(&second, relativeSecond);
    }

    mass *= density;
    first = first * density;
    second.ex = second.ex * density;
    second.ey = second.ey * density;
    second.ez = second.ez * density;

    outMassData->mass = mass;
    outMassData->centerOfMass = mass > epsilon ? first / mass : (vertices[0] + vertices[1] + vertices[2] + vertices[3]) * 0.25f;

    float trace = second.Trace();
    outMassData->inertia = Mat3{
        Vec3{ trace - second.ex.x, -second.ex.y, -second.ex.z },
        Vec3{ -second.ey.x, trace - second.ey.y, -second.ey.z },
        Vec3{ -second.ez.x, -second.ez.y, trace - second.ez.z },
    };
}

void QuadShape::ComputeAABB(const Transform& transform, AABB* outAABB) const
{
    MuliAssert(outAABB != nullptr);

    Vec3 min = Mul(transform, vertices[0]);
    Vec3 max = min;

    for (int32 i = 1; i < 4; ++i)
    {
        Vec3 v = Mul(transform, vertices[i]);
        min = Min(min, v);
        max = Max(max, v);
    }

    float scaledRadius = radius * Max(Abs(transform.s.x), Max(Abs(transform.s.y), Abs(transform.s.z)));
    Vec3 r{ scaledRadius, scaledRadius, scaledRadius };
    *outAABB = AABB{ min - r, max + r };
}

int32 QuadShape::GetSupport(const Vec3& localDir) const
{
    int32 index = 0;
    float maxValue = Dot(localDir, vertices[0]);

    for (int32 i = 1; i < 4; ++i)
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

Face QuadShape::GetFeaturedFace(const Transform& transform, const Vec3& dir) const
{
    Face face{};
    face.count = 4;

    Vec3 worldNormal = transform.q.Rotate(normal);
    if (Dot(worldNormal, dir) >= 0.0f)
    {
        face.normal = worldNormal;

        for (int32 i = 0; i < 4; ++i)
        {
            face.points[i].id = i;
            face.points[i].p = Mul(transform, vertices[i]);
        }
    }
    else
    {
        face.normal = -worldNormal;

        for (int32 i = 0; i < 4; ++i)
        {
            int32 index = i == 0 ? 0 : 4 - i;
            face.points[i].id = index;
            face.points[i].p = Mul(transform, vertices[index]);
        }
    }

    return face;
}

bool QuadShape::TestPoint(const Transform& transform, const Vec3& q) const
{
    Vec3 closest = GetClosestPoint(transform, q);
    return Dist2(closest, q) <= Sqr(linear_slop);
}

Vec3 QuadShape::GetClosestPoint(const Transform& transform, const Vec3& q) const
{
    Vec3 localQ = MulT(transform, q);
    Vec3 closest0 = ClosestPointVsTriangle(localQ, vertices[0], vertices[1], vertices[2]);
    Vec3 closest1 = ClosestPointVsTriangle(localQ, vertices[0], vertices[2], vertices[3]);
    Vec3 closest = Dist2(localQ, closest0) <= Dist2(localQ, closest1) ? closest0 : closest1;

    Vec3 normal = localQ - closest;
    float distance = normal.Normalize();
    if (distance <= radius)
    {
        return q;
    }

    return Mul(transform, closest + normal * radius);
}

bool QuadShape::RayCast(const Transform& transform, const RayCastInput& input, RayCastOutput* output) const
{
    Vec3 a = Mul(transform, vertices[0]);
    Vec3 b = Mul(transform, vertices[1]);
    Vec3 c = Mul(transform, vertices[2]);
    Vec3 d = Mul(transform, vertices[3]);
    Vec3 n = transform.q.Rotate(normal);
    float radii = radius + input.radius;

    RayCastOutput bestOutput;
    bestOutput.fraction = input.maxFraction;
    bool hit = false;

    auto KeepBest = [&](const RayCastOutput& candidate) {
        if (candidate.fraction <= bestOutput.fraction)
        {
            bestOutput = candidate;
            hit = true;
        }
    };

    RayCastInput rayInput = input;
    rayInput.radius = 0.0f;

    RayCastOutput candidate;
    if (RayCastTriangle(a + n * radii, b + n * radii, c + n * radii, rayInput, &candidate))
    {
        candidate.normal = n;
        KeepBest(candidate);
    }
    if (RayCastTriangle(a + n * radii, c + n * radii, d + n * radii, rayInput, &candidate))
    {
        candidate.normal = n;
        KeepBest(candidate);
    }
    if (RayCastTriangle(a - n * radii, c - n * radii, b - n * radii, rayInput, &candidate))
    {
        candidate.normal = -n;
        KeepBest(candidate);
    }
    if (RayCastTriangle(a - n * radii, d - n * radii, c - n * radii, rayInput, &candidate))
    {
        candidate.normal = -n;
        KeepBest(candidate);
    }

    RayCastInput capsuleInput = input;
    capsuleInput.radius = 0.0f;
    if (RayCastCapsule(a, b, radii, capsuleInput, &candidate))
    {
        KeepBest(candidate);
    }
    if (RayCastCapsule(b, c, radii, capsuleInput, &candidate))
    {
        KeepBest(candidate);
    }
    if (RayCastCapsule(c, d, radii, capsuleInput, &candidate))
    {
        KeepBest(candidate);
    }
    if (RayCastCapsule(d, a, radii, capsuleInput, &candidate))
    {
        KeepBest(candidate);
    }

    if (hit)
    {
        *output = bestOutput;
    }
    return hit;
}

} // namespace muli3
