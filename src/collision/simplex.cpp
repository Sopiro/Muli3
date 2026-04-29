#include "muli3/collision.h"

namespace muli3
{

Vec3 Simplex::GetSearchDirection() const
{
    switch (count)
    {
    case 1:
        return -vertices[0].point;

    case 2:
    {
        Vec3 a = vertices[0].point;
        Vec3 b = vertices[1].point;

        Vec3 ab = b - a;
        Vec3 ao = -a;

        // Triple product:
        // (ab x ao) x ab = ao * Dot(ab, ab) - ab * Dot(ab, ao)
        float d1 = Dot(ab, ab);
        float d2 = Dot(ab, ao);

        return ao * d1 - ab * d2;
    }

    case 3:
    {
        Vec3 a = vertices[0].point;
        Vec3 b = vertices[1].point;
        Vec3 c = vertices[2].point;

        Vec3 ab = b - a;
        Vec3 ac = c - a;

        Vec3 n = Cross(ab, ac);

        if (Dot(n, n) <= epsilon)
        {
            return -GetClosestPoint();
        }

        // Choose the face normal that points toward the origin.
        return Dot(n, -a) > 0.0f ? n : -n;
    }

    default:
        MuliAssert(false);
        return Vec3::zero;
    }
}

Vec3 Simplex::GetClosestPoint() const
{
    float d = 1.0f / divisor;

    switch (count)
    {
    case 1:
        return vertices[0].point;

    case 2:
        return (d * vertices[0].weight) * vertices[0].point + (d * vertices[1].weight) * vertices[1].point;

    case 3:
        return (d * vertices[0].weight) * vertices[0].point + (d * vertices[1].weight) * vertices[1].point +
               (d * vertices[2].weight) * vertices[2].point;

    case 4:
        return (d * vertices[0].weight) * vertices[0].point + (d * vertices[1].weight) * vertices[1].point +
               (d * vertices[2].weight) * vertices[2].point + (d * vertices[3].weight) * vertices[3].point;

    default:
        MuliAssert(false);
        return Vec3::zero;
    }
}

void Simplex::GetWitnessPoint(Vec3* pointA, Vec3* pointB) const
{
    float d = 1.0f / divisor;

    switch (count)
    {
    case 1:
    {
        *pointA = vertices[0].pointA.p;
        *pointB = vertices[0].pointB.p;
        return;
    }

    case 2:
    {
        *pointA = (d * vertices[0].weight) * vertices[0].pointA.p + (d * vertices[1].weight) * vertices[1].pointA.p;

        *pointB = (d * vertices[0].weight) * vertices[0].pointB.p + (d * vertices[1].weight) * vertices[1].pointB.p;
        return;
    }

    case 3:
    {
        *pointA = (d * vertices[0].weight) * vertices[0].pointA.p + (d * vertices[1].weight) * vertices[1].pointA.p +
                  (d * vertices[2].weight) * vertices[2].pointA.p;

        *pointB = (d * vertices[0].weight) * vertices[0].pointB.p + (d * vertices[1].weight) * vertices[1].pointB.p +
                  (d * vertices[2].weight) * vertices[2].pointB.p;
        return;
    }

    case 4:
    {
        // The origin is inside the tetrahedron.
        // In the Minkowski difference, sum(w * (A - B)) = 0,
        // so the two witness points can be represented as the same point.
        *pointA = (d * vertices[0].weight) * vertices[0].pointA.p + (d * vertices[1].weight) * vertices[1].pointA.p +
                  (d * vertices[2].weight) * vertices[2].pointA.p + (d * vertices[3].weight) * vertices[3].pointA.p;

        *pointB = *pointA;
        return;
    }

    default:
        MuliAssert(false);
    }
}

void Simplex::Advance(const Vec3& q)
{
    switch (count)
    {
    case 1:
        vertices[0].weight = 1.0f;
        divisor = 1.0f;
        return;

    case 2:
        SolveSegment(q);
        return;

    case 3:
        SolveTriangle(q);
        return;

    case 4:
        SolveTetrahedron(q);
        return;

    default:
        MuliAssert(false);
        return;
    }
}

void Simplex::SolveSegment(const Vec3& q)
{
    SupportPoint va = vertices[0];
    SupportPoint vb = vertices[1];

    Vec3 a = va.point;
    Vec3 b = vb.point;
    Vec3 ab = b - a;

    float denom = Dot(ab, ab);

    if (denom <= epsilon)
    {
        count = 1;
        vertices[0] = va;
        vertices[0].weight = 1.0f;
        divisor = 1.0f;
        return;
    }

    float t = Dot(q - a, ab) / denom;

    // Region A
    if (t <= 0.0f)
    {
        count = 1;
        vertices[0] = va;
        vertices[0].weight = 1.0f;
        divisor = 1.0f;
        return;
    }

    // Region B
    if (t >= 1.0f)
    {
        count = 1;
        vertices[0] = vb;
        vertices[0].weight = 1.0f;
        divisor = 1.0f;
        return;
    }

    // Region AB
    count = 2;
    vertices[0] = va;
    vertices[1] = vb;

    vertices[0].weight = 1.0f - t;
    vertices[1].weight = t;

    divisor = 1.0f;
}

void Simplex::SolveTriangle(const Vec3& q)
{
    SupportPoint va = vertices[0];
    SupportPoint vb = vertices[1];
    SupportPoint vc = vertices[2];

    Vec3 a = va.point;
    Vec3 b = vb.point;
    Vec3 c = vc.point;

    Vec3 ab = b - a;
    Vec3 ac = c - a;

    Vec3 n = Cross(ab, ac);

    // Degenerate triangle: reduce it to the closest edge.
    if (Dot(n, n) <= epsilon)
    {
        Simplex best;
        float bestDist2 = FLT_MAX;

        auto TestEdge = [&](const SupportPoint& p0, const SupportPoint& p1) {
            Simplex s;
            s.count = 2;
            s.vertices[0] = p0;
            s.vertices[1] = p1;
            s.SolveSegment(q);

            Vec3 p = s.GetClosestPoint();
            Vec3 r = p - q;
            float dist2 = Dot(r, r);

            if (dist2 < bestDist2)
            {
                bestDist2 = dist2;
                best = s;
            }
        };

        TestEdge(va, vb);
        TestEdge(vb, vc);
        TestEdge(vc, va);

        *this = best;
        return;
    }

    // Triangle Voronoi region test.
    // Based on the closest point on triangle method from Real-Time Collision Detection.

    Vec3 ap = q - a;
    float d1 = Dot(ab, ap);
    float d2 = Dot(ac, ap);

    // Region A
    if (d1 <= 0.0f && d2 <= 0.0f)
    {
        count = 1;
        vertices[0] = va;
        vertices[0].weight = 1.0f;
        divisor = 1.0f;
        return;
    }

    Vec3 bp = q - b;
    float d3 = Dot(ab, bp);
    float d4 = Dot(ac, bp);

    // Region B
    if (d3 >= 0.0f && d4 <= d3)
    {
        count = 1;
        vertices[0] = vb;
        vertices[0].weight = 1.0f;
        divisor = 1.0f;
        return;
    }

    // Region AB
    float vcArea = d1 * d4 - d3 * d2;
    if (vcArea <= 0.0f && d1 >= 0.0f && d3 <= 0.0f)
    {
        float t = d1 / (d1 - d3);

        count = 2;
        vertices[0] = va;
        vertices[1] = vb;

        vertices[0].weight = 1.0f - t;
        vertices[1].weight = t;

        divisor = 1.0f;
        return;
    }

    Vec3 cp = q - c;
    float d5 = Dot(ab, cp);
    float d6 = Dot(ac, cp);

    // Region C
    if (d6 >= 0.0f && d5 <= d6)
    {
        count = 1;
        vertices[0] = vc;
        vertices[0].weight = 1.0f;
        divisor = 1.0f;
        return;
    }

    // Region AC
    float vbArea = d5 * d2 - d1 * d6;
    if (vbArea <= 0.0f && d2 >= 0.0f && d6 <= 0.0f)
    {
        float t = d2 / (d2 - d6);

        count = 2;
        vertices[0] = va;
        vertices[1] = vc;

        vertices[0].weight = 1.0f - t;
        vertices[1].weight = t;

        divisor = 1.0f;
        return;
    }

    // Region BC
    float vaArea = d3 * d6 - d5 * d4;
    if (vaArea <= 0.0f && (d4 - d3) >= 0.0f && (d5 - d6) >= 0.0f)
    {
        float t = (d4 - d3) / ((d4 - d3) + (d5 - d6));

        count = 2;
        vertices[0] = vb;
        vertices[1] = vc;

        vertices[0].weight = 1.0f - t;
        vertices[1].weight = t;

        divisor = 1.0f;
        return;
    }

    // Region ABC
    float denom = 1.0f / (vaArea + vbArea + vcArea);

    float v = vbArea * denom;
    float w = vcArea * denom;
    float u = 1.0f - v - w;

    count = 3;
    vertices[0] = va;
    vertices[1] = vb;
    vertices[2] = vc;

    vertices[0].weight = u;
    vertices[1].weight = v;
    vertices[2].weight = w;

    divisor = 1.0f;
}

void Simplex::SolveTetrahedron(const Vec3& q)
{
    SupportPoint va = vertices[0];
    SupportPoint vb = vertices[1];
    SupportPoint vc = vertices[2];
    SupportPoint vd = vertices[3];

    Vec3 a = va.point;
    Vec3 b = vb.point;
    Vec3 c = vc.point;
    Vec3 d = vd.point;

    float volume = Dot(b - a, Cross(c - a, d - a));

    // Check whether q lies inside the tetrahedron using barycentric coordinates.
    if (std::fabs(volume) > epsilon)
    {
        float invVolume = 1.0f / volume;

        float wb = Dot(q - a, Cross(c - a, d - a)) * invVolume;
        float wc = Dot(b - a, Cross(q - a, d - a)) * invVolume;
        float wd = Dot(b - a, Cross(c - a, q - a)) * invVolume;
        float wa = 1.0f - wb - wc - wd;

        if (wa >= -epsilon && wb >= -epsilon && wc >= -epsilon && wd >= -epsilon)
        {
            // Region ABCD: q is inside the tetrahedron.
            count = 4;

            vertices[0] = va;
            vertices[1] = vb;
            vertices[2] = vc;
            vertices[3] = vd;

            vertices[0].weight = wa;
            vertices[1].weight = wb;
            vertices[2].weight = wc;
            vertices[3].weight = wd;

            divisor = 1.0f;
            return;
        }
    }

    // If q is outside the tetrahedron, choose the closest face simplex.
    Simplex best;
    float bestDist2 = FLT_MAX;

    SupportPoint old[4] = { va, vb, vc, vd };

    const auto TestFace = [&](int32 i0, int32 i1, int32 i2) {
        Simplex s;
        s.count = 3;
        s.vertices[0] = old[i0];
        s.vertices[1] = old[i1];
        s.vertices[2] = old[i2];

        s.SolveTriangle(q);

        Vec3 p = s.GetClosestPoint();
        Vec3 r = p - q;
        float dist2 = Dot(r, r);

        if (dist2 < bestDist2)
        {
            bestDist2 = dist2;
            best = s;
        }
    };

    TestFace(0, 1, 2); // ABC
    TestFace(0, 1, 3); // ABD
    TestFace(0, 2, 3); // ACD
    TestFace(1, 2, 3); // BCD

    *this = best;
}

} // namespace muli3
