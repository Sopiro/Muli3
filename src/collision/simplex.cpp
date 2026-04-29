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
        else
        {
            // Choose the face normal that points toward the origin.
            return Dot(n, -a) > 0.0f ? n : -n;
        }
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
    Vec3 e = b - a;

    divisor = Dot(e, e);
    if (divisor <= epsilon)
    {
        count = 1;
        vertices[0] = va;
        vertices[0].weight = 1.0f;
        divisor = 1.0f;
        return;
    }

    Vec2 w{ Dot(q - b, a - b), Dot(q - a, b - a) };

    // Region A
    if (w.y <= 0.0f)
    {
        count = 1;
        vertices[0] = va;
        vertices[0].weight = 1.0f;
        divisor = 1.0f;
        return;
    }

    // Region B
    if (w.x <= 0.0f)
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

    vertices[0].weight = w.x;
    vertices[1].weight = w.y;
}

void Simplex::SolveTriangle(const Vec3& q)
{
    SupportPoint va = vertices[0];
    SupportPoint vb = vertices[1];
    SupportPoint vc = vertices[2];

    Vec3 a = va.point;
    Vec3 b = vb.point;
    Vec3 c = vc.point;

    Vec2 wab{ Dot(q - b, a - b), Dot(q - a, b - a) };
    Vec2 wbc{ Dot(q - c, b - c), Dot(q - b, c - b) };
    Vec2 wca{ Dot(q - a, c - a), Dot(q - c, a - c) };

    // Region A
    if (wca.x <= 0.0f && wab.y <= 0.0f)
    {
        count = 1;
        vertices[0] = va;
        vertices[0].weight = 1.0f;
        divisor = 1.0f;
        return;
    }

    // Region B
    if (wab.x <= 0.0f && wbc.y <= 0.0f)
    {
        count = 1;
        vertices[0] = vb;
        vertices[0].weight = 1.0f;
        divisor = 1.0f;
        return;
    }

    // Region C
    if (wbc.x <= 0.0f && wca.y <= 0.0f)
    {
        count = 1;
        vertices[0] = vc;
        vertices[0].weight = 1.0f;
        divisor = 1.0f;
        return;
    }

    Vec3 ab = b - a;
    Vec3 ac = c - a;

    Vec3 n = Cross(ab, ac);
    float n2 = Dot(n, n);

    float u = Dot(Cross(b - q, c - q), n);
    float v = Dot(Cross(c - q, a - q), n);
    float w = Dot(Cross(a - q, b - q), n);

    // Region AB
    if (wab.x > 0.0f && wab.y > 0.0f && (w <= 0.0f || n2 <= epsilon))
    {
        count = 2;
        vertices[0] = va;
        vertices[1] = vb;

        vertices[0].weight = wab.x;
        vertices[1].weight = wab.y;

        Vec3 e = b - a;
        divisor = Dot(e, e);
        return;
    }

    // Region BC
    if (wbc.x > 0.0f && wbc.y > 0.0f && (u <= 0.0f || n2 <= epsilon))
    {
        count = 2;
        vertices[0] = vb;
        vertices[1] = vc;

        vertices[0].weight = wbc.x;
        vertices[1].weight = wbc.y;

        Vec3 e = c - b;
        divisor = Dot(e, e);
        return;
    }

    // Region CA
    if (wca.x > 0.0f && wca.y > 0.0f && (v <= 0.0f || n2 <= epsilon))
    {
        count = 2;
        vertices[0] = vc;
        vertices[1] = va;

        vertices[0].weight = wca.x;
        vertices[1].weight = wca.y;

        Vec3 e = a - c;
        divisor = Dot(e, e);
        return;
    }

    // Region ABC
    count = 3;
    vertices[0] = va;
    vertices[1] = vb;
    vertices[2] = vc;

    vertices[0].weight = u;
    vertices[1].weight = v;
    vertices[2].weight = w;

    divisor = n2;
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
    float wa = 0.0f;
    float wb = 0.0f;
    float wc = 0.0f;
    float wd = 0.0f;

    // Use signed sub-volumes as unnormalized barycentric weights.
    if (std::fabs(volume) > epsilon)
    {
        wb = Dot(q - a, Cross(c - a, d - a));
        wc = Dot(b - a, Cross(q - a, d - a));
        wd = Dot(b - a, Cross(c - a, q - a));
        wa = volume - wb - wc - wd;

        if ((volume > 0.0f && wa >= 0.0f && wb >= 0.0f && wc >= 0.0f && wd >= 0.0f) ||
            (volume < 0.0f && wa <= 0.0f && wb <= 0.0f && wc <= 0.0f && wd <= 0.0f))
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

            divisor = volume;
            return;
        }
    }

    Simplex best;
    float bestDist2 = max_float;

    // A degenerate tetrahedron has no reliable inside/outside face test,
    // so all faces are tested and the closest one is kept.
    bool degenerate = std::fabs(volume) <= epsilon;

    // Outside faces are the only candidates for the closest feature.
    // Each weight belongs to the opposite face:
    // wd -> ABC
    // wc -> ABD
    // wb -> ACD
    // wa -> BCD

    // Region ABC
    if ((volume > 0.0f && wd < 0.0f) || (volume < 0.0f && wd > 0.0f) || degenerate)
    {
        Simplex s;
        s.count = 3;
        s.vertices[0] = va;
        s.vertices[1] = vb;
        s.vertices[2] = vc;
        s.SolveTriangle(q);

        Vec3 p = s.GetClosestPoint();
        Vec3 r = p - q;
        float dist2 = Dot(r, r);

        if (dist2 < bestDist2)
        {
            bestDist2 = dist2;
            best = s;
        }
    }

    // Region ABD
    if ((volume > 0.0f && wc < 0.0f) || (volume < 0.0f && wc > 0.0f) || degenerate)
    {
        Simplex s;
        s.count = 3;
        s.vertices[0] = va;
        s.vertices[1] = vb;
        s.vertices[2] = vd;
        s.SolveTriangle(q);

        Vec3 p = s.GetClosestPoint();
        Vec3 r = p - q;
        float dist2 = Dot(r, r);
        if (dist2 < bestDist2)
        {
            bestDist2 = dist2;
            best = s;
        }
    }

    // Region ACD
    if ((volume > 0.0f && wb < 0.0f) || (volume < 0.0f && wb > 0.0f) || degenerate)
    {
        Simplex s;
        s.count = 3;
        s.vertices[0] = va;
        s.vertices[1] = vc;
        s.vertices[2] = vd;
        s.SolveTriangle(q);

        Vec3 p = s.GetClosestPoint();
        Vec3 r = p - q;
        float dist2 = Dot(r, r);
        if (dist2 < bestDist2)
        {
            bestDist2 = dist2;
            best = s;
        }
    }

    // Region BCD
    if ((volume > 0.0f && wa < 0.0f) || (volume < 0.0f && wa > 0.0f) || degenerate)
    {
        Simplex s;
        s.count = 3;
        s.vertices[0] = vb;
        s.vertices[1] = vc;
        s.vertices[2] = vd;
        s.SolveTriangle(q);

        Vec3 p = s.GetClosestPoint();
        Vec3 r = p - q;
        float dist2 = Dot(r, r);
        if (dist2 < bestDist2)
        {
            bestDist2 = dist2;
            best = s;
        }
    }

    *this = best;
}

} // namespace muli3
