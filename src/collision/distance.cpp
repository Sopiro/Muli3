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
