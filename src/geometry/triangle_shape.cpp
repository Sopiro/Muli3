#include "muli3/triangle_shape.h"
#include "muli3/distance.h"
#include "muli3/settings.h"

namespace muli3
{

TriangleShape::TriangleShape(const Vec3& a, const Vec3& b, const Vec3& c, float inRadius, const Transform& transform)
    : Shape(Shape::triangle, inRadius)
{
    vertices[0] = Mul(transform, a);
    vertices[1] = Mul(transform, b);
    vertices[2] = Mul(transform, c);

    normal = Cross(vertices[1] - vertices[0], vertices[2] - vertices[0]);
    normal.Normalize();

    if (radius == 0.0f)
    {
        center = (vertices[0] + vertices[1] + vertices[2]) / 3.0f;
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

TriangleShape::TriangleShape(const TriangleShape& other, const Transform& transform)
    : TriangleShape(other.vertices[0], other.vertices[1], other.vertices[2], other.radius, transform)
{
}

void TriangleShape::ComputeMass(float density, MassData* outMassData) const
{
    MuliAssert(outMassData != nullptr);

    // Rounded triangle is the Minkowski sum of the core triangle and a sphere of radius r.
    //
    // Its volume is integrated as:
    // V = 2 * r * area + 0.5 * pi * r^2 * perimeter + 4 / 3 * pi * r^3
    //
    // The three terms are the face slab, three edge half-cylinders, and sphere.
    //
    // Accumulate \int dV, \int x dV, and \int x * x^T dV, then convert the second moment to inertia at the end:
    //
    // I = \int x \cdot x * Identity - x * x^T dm

    // Outer(Dyad) product: a * b^T
    const auto Outer = [](const Vec3& a, const Vec3& b) { return Mat3(a * b.x, a * b.y, a * b.z); };
    const auto Mul = [](const Mat3& m, float s) { return Mat3(m.ex * s, m.ey * s, m.ez * s); };
    const auto Add = [](Mat3* a, const Mat3& b) { *a = *a + b; };

    Vec3 a = vertices[0];
    Vec3 b = vertices[1];
    Vec3 c = vertices[2];

    Vec3 ab = b - a;
    Vec3 ac = c - a;
    Vec3 areaNormal = Cross(ab, ac);
    float area2 = areaNormal.Normalize();
    float area = area2 * 0.5f;

    if (area <= epsilon)
    {
        outMassData->mass = 0.0f;
        outMassData->inertia = Mat3::zero;
        outMassData->centerOfMass = (a + b + c) / 3.0f;
        return;
    }

    Vec3 n = areaNormal;
    float r = radius;
    float r2 = r * r;
    float r3 = r2 * r;
    float r4 = r2 * r2;
    float r5 = r4 * r;

    float mass = 0.0f;
    Vec3 first = Vec3::zero;
    Mat3 second = Mat3::zero;

    // Face slab.  Extrude the triangle along the normal by [-r, r].
    //
    // M1 = \int_triangle \int_{-r}^r (x + t n) dt dA
    //    = 2r * \int_triangle x dA
    //    = 2r * A * (a + b + c) / 3
    //
    // M2 = \int_triangle \int_{-r}^r (p + t n)(p + t n)^T dt dA
    //
    // with \int_{-r}^r dt = 2r and \int_{-r}^r t^2 dt = 2r^3 / 3
    //
    // M2 = 2r * \int_triangle pp^T dA + (2r^3 / 3) * A * nn^T
    // \int_triangle pp^T dA = A / 12 * (aa^T + bb^T + cc^T + ss^T), where s = a+b+c

    Vec3 sum = a + b + c;
    Mat3 triangleSecond = Mul(Outer(a, a) + Outer(b, b) + Outer(c, c) + Outer(sum, sum), area / 12.0f);

    float faceVolume = 2.0f * r * area;
    mass += faceVolume;
    first = first + sum * (area * 2.0f * r / 3.0f);
    Add(&second, Mul(triangleSecond, 2.0f * r));
    Add(&second, Mul(Outer(n, n), area * 2.0f * r3 / 3.0f));

    for (int32 i = 0; i < 3; ++i)
    {
        Vec3 p0 = vertices[i];
        Vec3 p1 = vertices[(i + 1) % 3];
        Vec3 edge = p1 - p0;

        float L = edge.Normalize();
        if (L == 0.0f)
        {
            continue;
        }

        Vec3 o = Cross(edge, n);

        // Edge half-cylinder.  The cross section is the outside half-disk in
        // the outward/normal plane, swept along the edge segment.
        //
        // x = p(s) + y * o + z * n
        //
        // s in [0, L]
        // p(s) = p0 + e * s
        // (y, z) in halfdisk D, where D = { (y, z) | y >= 0, y^2 + z^2 <= r^2 }
        //
        // A = pi * r^2 / 2
        // \int_D y dA = 2 * r^3 / 3 : halfDiskFirst
        // \int_D z dA = 0
        //
        // \int_D y^2 dA = \int_D z^2 dA = pi * r^4 / 8 : halfDiskSecond
        // \int_D yz dA = 0
        //
        // M1 = \int_0^L \int_D p(s) + y*o + z*n dA ds
        //    = \int_0^L \int_D p(s) dA ds
        //      + \int_0^L \int_D y*o dA ds
        //      + \int_0^L \int_D z*n dA ds
        //
        // \int_D p(s) dA = p(s) * A_hd
        // \int_D y*o dA = o * \int_D y dA
        // \int_D z*n dA = n * \int_D z dA = 0
        //
        // M1 = A_hd * \int_0^L p(s) ds  +  L * o * \int_D y dA
        //
        // Segment integrals:
        // \int_0^L p(s) ds = L * (p0 + p1) / 2 : segmetFirst
        // \int_0^L p(s) * p(s)^T ds = L / 3 * (p0p0^T + p1p1^T) + L / 6 * (p0p1^T + p1p0^T) :segmentSecond
        //
        // M2 = \int x x^T dV
        //
        // x x^T = (p + y*o + z*n)(p + y*o + z*n)^T
        //       = p p^T + y p o^T + y o p^T + z p n^T + z n p^T + y^2 o o^T + z^2 n n^T + yz o n^T + yz n o^T
        // Since the halfdisk is symmetric in the z-direction, the integrands with odd parity in z vanish.
        //
        // M2 = A_hd * \int_0^L p p^T ds
        //      + (\int_D y dA) * [ (\int_0^L p ds) o^T + o (\int_0^L p ds)^T ]
        //      + L * (\int_D y^2 dA) * o o^T
        //      + L * (\int_D z^2 dA) * n n^T
        //
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

    for (int32 i = 0; i < 3; ++i)
    {
        Vec3 prev = vertices[(i + 2) % 3];
        Vec3 p = vertices[i];
        Vec3 next = vertices[(i + 1) % 3];

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

        // Vertex spherical sector.  The sector angle is the outside angle between
        // adjacent edge half-cylinders.
        //
        // Use the local frame { b, t, n }, where b is the angle bisector and t lies in the triangle plane.
        // The sector is centered at vertex p.
        //
        // q = u*b + v*t + w*n
        // x = p + q
        //
        // In polar coordinates in the b/t plane:
        // u = s * cos(phi)
        // v = s * sin(phi)
        //
        // phi in [-angle/2, angle/2] and s^2 + w^2 <= r^2
        //
        // \int_sector dV = 2 * angle * r^3 / 3 : sectorVolume
        //
        // The sector is symmetric around b and n, so the first moment has only the bisector component:
        // \int_sector q dV = b * \int_sector u dV : relativeFirst
        // \int_sector u dV = pi * r^4 * sin(angle / 2) / 4 : sectorFirst
        //
        // Cross terms in \int q q^T dV vanish by symmetry:
        // \int uv dV = 0, \int uw dV = 0, \int vw dV = 0
        //
        // Relative second moment in the local frame:
        // M2_rel = \int q q^T dV
        //
        // The uv term contains cos(phi) * sin(phi), which is odd around phi = 0,
        // so left/right symmetry cancels it.  The uw and vw terms contain w,
        // which is odd across the triangle plane, so normal-direction symmetry cancels them.
        //
        //
        // M2_rel = (\int u^2 dV) * bb^T
        //        + (\int v^2 dV) * tt^T
        //        + (\int w^2 dV) * nn^T
        //
        // \int u^2 dV = 2 * r^5 * (angle + sin(angle)) / 15 :i0
        // \int v^2 dV = 2 * r^5 * (angle - sin(angle)) / 15 :i1
        // \int w^2 dV = 2 * angle * r^5 / 15                :i2
        //
        // Translate the relative moments from vertex p to the shape origin:
        //
        // M1 = \int_sector x dV = \int_sector p+q dV = p * \int_sector dV + \int_sector q dV
        //
        // M2 = \int x x^T dV
        //    = \int (p + q)(p + q)^T dV
        //    = pp^T * \int_sector dV
        //      + p * (\int_sector q dV)^T
        //      + (\int_sector q dV) * p^T
        //      + M2_rel
        //
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
    outMassData->centerOfMass = mass > epsilon ? first / mass : (a + b + c) / 3.0f;

    // I = \int (dot(x,x) Identity - x x^T) dm = tr(M2) * Identity - M2.
    float trace = second.Trace();
    outMassData->inertia = Mat3{
        Vec3{ trace - second.ex.x, -second.ex.y, -second.ex.z },
        Vec3{ -second.ey.x, trace - second.ey.y, -second.ey.z },
        Vec3{ -second.ez.x, -second.ez.y, trace - second.ez.z },
    };
}

void TriangleShape::ComputeAABB(const Transform& transform, AABB* outAABB) const
{
    MuliAssert(outAABB != nullptr);

    Vec3 min = Mul(transform, vertices[0]);
    Vec3 max = min;

    for (int32 i = 1; i < 3; ++i)
    {
        Vec3 v = Mul(transform, vertices[i]);
        min = Min(min, v);
        max = Max(max, v);
    }

    float scaledRadius = radius * Max(Abs(transform.s.x), Max(Abs(transform.s.y), Abs(transform.s.z)));
    Vec3 r{ scaledRadius, scaledRadius, scaledRadius };
    *outAABB = AABB{ min - r, max + r };
}

int32 TriangleShape::GetSupport(const Vec3& localDir) const
{
    int32 index = 0;
    float maxValue = Dot(localDir, vertices[0]);

    for (int32 i = 1; i < 3; ++i)
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

Face TriangleShape::GetFeaturedFace(const Transform& transform, const Vec3& dir) const
{
    Face face{};
    face.count = 3;

    Vec3 worldNormal = transform.q.Rotate(normal);
    if (Dot(worldNormal, dir) >= 0.0f)
    {
        face.normal = worldNormal;

        for (int32 i = 0; i < 3; ++i)
        {
            face.points[i].id = i;
            face.points[i].p = Mul(transform, vertices[i]);
        }
    }
    else
    {
        // Reverse the winding so the triangle can be clipped as a solid two-sided face.
        face.normal = -worldNormal;

        for (int32 i = 0; i < 3; ++i)
        {
            int32 index = i == 0 ? 0 : 3 - i;
            face.points[i].id = index;
            face.points[i].p = Mul(transform, vertices[index]);
        }
    }

    return face;
}

bool TriangleShape::TestPoint(const Transform& transform, const Vec3& q) const
{
    Vec3 closest = GetClosestPoint(transform, q);
    return Dist2(closest, q) <= Sqr(linear_slop);
}

Vec3 TriangleShape::GetClosestPoint(const Transform& transform, const Vec3& q) const
{
    Vec3 localQ = MulT(transform, q);
    Vec3 closest = ClosestPointVsTriangle(localQ, vertices[0], vertices[1], vertices[2]);
    Vec3 normal = localQ - closest;
    float distance = normal.Normalize();
    if (distance <= radius)
    {
        return q;
    }

    return Mul(transform, closest + normal * radius);
}

bool TriangleShape::RayCast(const Transform& transform, const RayCastInput& input, RayCastOutput* output) const
{
    Vec3 a = Mul(transform, vertices[0]);
    Vec3 b = Mul(transform, vertices[1]);
    Vec3 c = Mul(transform, vertices[2]);
    return RayCastTriangle(a, b, c, input, output);
}

} // namespace muli3
