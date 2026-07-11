#include "muli3/distance.h"
#include "muli3/shape.h"

namespace muli3
{

float GetClosestFeatures(const Shape* a, const Transform& tfA, const Shape* b, const Transform& tfB, ClosestFeatures* features)
{
    GJKResult gjkResult;

    bool collide = GJK(a, tfA, b, tfB, &gjkResult);
    if (collide)
    {
        return 0.0f;
    }

    Simplex& simplex = gjkResult.simplex;
    MuliAssert(simplex.count < max_simplex_vertex_count);

    features->count = simplex.count;
    for (int32 i = 0; i < features->count; ++i)
    {
        features->featuresA[i] = simplex.vertices[i].pointA;
        features->featuresB[i] = simplex.vertices[i].pointB;
    }

    return gjkResult.distance;
}

float ComputeDistance(const Shape* a, const Transform& tfA, const Shape* b, const Transform& tfB, Vec3* pointA, Vec3* pointB)
{
    GJKResult gjkResult;

    bool collide = GJK(a, tfA, b, tfB, &gjkResult);
    if (collide)
    {
        return 0.0f;
    }

    float ra = a->GetRadius();
    float rb = b->GetRadius();
    float radii = ra + rb;

    if (gjkResult.distance < radii)
    {
        return 0.0f;
    }

    Simplex& simplex = gjkResult.simplex;
    MuliAssert(simplex.count < max_simplex_vertex_count);

    simplex.GetWitnessPoint(pointA, pointB);

    *pointA += gjkResult.direction * ra;
    *pointB -= gjkResult.direction * rb;

    return gjkResult.distance - radii;
}

Vec3 ClosestPointVsSegment(const Vec3& p, const Vec3& a, const Vec3& b)
{
    Vec3 ab = b - a;
    float ab2 = Dot(ab, ab);
    if (ab2 <= epsilon)
    {
        return a;
    }

    float t = Clamp(Dot(p - a, ab) / ab2, 0.0f, 1.0f);
    return a + ab * t;
}

Vec3 ClosestPointVsTriangle(const Vec3& p, const Vec3& a, const Vec3& b, const Vec3& c)
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

Vec3 ClosestPointVsTetrahedron(const Vec3& p, const Vec3& a, const Vec3& b, const Vec3& c, const Vec3& d)
{
    float volume = Dot(b - a, Cross(c - a, d - a));
    float wa = 0.0f;
    float wb = 0.0f;
    float wc = 0.0f;
    float wd = 0.0f;

    if (Abs(volume) > epsilon)
    {
        // Signed sub-volumes are unnormalized barycentric weights.
        wb = Dot(p - a, Cross(c - a, d - a));
        wc = Dot(b - a, Cross(p - a, d - a));
        wd = Dot(b - a, Cross(c - a, p - a));
        wa = volume - wb - wc - wd;

        if ((volume > 0.0f && wa >= 0.0f && wb >= 0.0f && wc >= 0.0f && wd >= 0.0f) ||
            (volume < 0.0f && wa <= 0.0f && wb <= 0.0f && wc <= 0.0f && wd <= 0.0f))
        {
            return p;
        }
    }

    bool degenerate = Abs(volume) <= epsilon;
    float bestDist2 = max_float;
    Vec3 best = Vec3::zero;

    // Only faces opposite negative weights can contain the closest point.
    // Degenerate tetrahedra have no reliable inside test, so every face is checked.
    if ((volume > 0.0f && wd < 0.0f) || (volume < 0.0f && wd > 0.0f) || degenerate)
    {
        Vec3 q = ClosestPointVsTriangle(p, a, b, c);
        Vec3 r = q - p;
        float dist2 = Dot(r, r);
        if (dist2 < bestDist2)
        {
            bestDist2 = dist2;
            best = q;
        }
    }

    if ((volume > 0.0f && wc < 0.0f) || (volume < 0.0f && wc > 0.0f) || degenerate)
    {
        Vec3 q = ClosestPointVsTriangle(p, a, b, d);
        Vec3 r = q - p;
        float dist2 = Dot(r, r);
        if (dist2 < bestDist2)
        {
            bestDist2 = dist2;
            best = q;
        }
    }

    if ((volume > 0.0f && wb < 0.0f) || (volume < 0.0f && wb > 0.0f) || degenerate)
    {
        Vec3 q = ClosestPointVsTriangle(p, a, c, d);
        Vec3 r = q - p;
        float dist2 = Dot(r, r);
        if (dist2 < bestDist2)
        {
            bestDist2 = dist2;
            best = q;
        }
    }

    if ((volume > 0.0f && wa < 0.0f) || (volume < 0.0f && wa > 0.0f) || degenerate)
    {
        Vec3 q = ClosestPointVsTriangle(p, b, c, d);
        Vec3 r = q - p;
        float dist2 = Dot(r, r);
        if (dist2 < bestDist2)
        {
            bestDist2 = dist2;
            best = q;
        }
    }

    MuliAssert(bestDist2 < max_float);
    return best;
}

Vec3 ClosestPointVsPolygon(const Vec3& q, std::span<const Vec3> vertices)
{
    const int32 count = int32(vertices.size());
    MuliAssert(count >= 3);

    const Vec3& origin = vertices[0];

    // The normal does not need to be normalized.
    // Using both vertices adjacent to origin also allows intermediate
    // collinear vertices elsewhere in the polygon.
    Vec3 normal = Cross(vertices[1] - origin, vertices[count - 1] - origin);
    float length2 = Dot(normal, normal);

    MuliAssert(length2 > epsilon);

    Vec3 closestPoint = origin;
    float minDistance2 = std::numeric_limits<float>::max();
    bool outside = false;

    const Vec3* a = &vertices[count - 1];

    for (int32 i = 0; i < count; ++i)
    {
        const Vec3* b = &vertices[i];

        Vec3 edge = *b - *a;
        Vec3 aq = q - *a;

        // The normal is constructed from the polygon winding, so a negative
        // value means that q lies outside this edge's supporting half-space.
        if (Dot(Cross(edge, aq), normal) < 0.0f)
        {
            outside = true;

            float edgeLength2 = Dot(edge, edge);
            float fraction = Clamp(Dot(aq, edge) / edgeLength2, 0.0f, 1.0f);

            Vec3 point = *a + fraction * edge;
            float distance2 = Length2(q - point);

            if (distance2 < minDistance2)
            {
                minDistance2 = distance2;
                closestPoint = point;
            }
        }

        a = b;
    }

    if (outside)
    {
        return closestPoint;
    }

    // q is inside the polygon's edge half-spaces, so its orthogonal
    // projection onto the polygon plane is the closest point.
    float planeOffset = Dot(q - origin, normal) / length2;
    return q - planeOffset * normal;
}

Vec2 ClosestSegmentVsSegment(const Vec3& a0, const Vec3& a1, const Vec3& b0, const Vec3& b1)
{
    // Compute the closest points on two segments.
    // s and t are the interpolation parameters on segment A and B.
    Vec3 da = a1 - a0;
    Vec3 db = b1 - b0;
    Vec3 r = a0 - b0;

    float a = Dot(da, da);
    float e = Dot(db, db);
    float f = Dot(db, r);

    float s = 0.0f;
    float t = 0.0f;

    if (a <= epsilon && e <= epsilon)
    {
        return Vec2::zero;
    }

    if (a <= epsilon)
    {
        t = Clamp(f / e, 0.0f, 1.0f);
    }
    else
    {
        float c = Dot(da, r);
        if (e <= epsilon)
        {
            s = Clamp(-c / a, 0.0f, 1.0f);
        }
        else
        {
            float b = Dot(da, db);
            float denom = a * e - b * b;

            // If denom is zero, the segments are parallel.
            // Keep s at zero first, then clamp t and recompute s if needed.
            if (denom > epsilon)
            {
                s = Clamp((b * f - c * e) / denom, 0.0f, 1.0f);
            }

            t = (b * s + f) / e;

            if (t < 0.0f)
            {
                t = 0.0f;
                s = Clamp(-c / a, 0.0f, 1.0f);
            }
            else if (t > 1.0f)
            {
                t = 1.0f;
                s = Clamp((b - c) / a, 0.0f, 1.0f);
            }
        }
    }

    return { s, t };
}

} // namespace muli3
