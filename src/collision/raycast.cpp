#include "muli3/raycast.h"
#include "muli3/collision.h"
#include "muli3/settings.h"
#include "muli3/shape.h"

namespace muli3
{

static Vec3 ClosestPointOnSegment(const Vec3& a, const Vec3& b, const Vec3& p)
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

bool RayCastSphere(const Vec3& p, float r, const RayCastInput& input, RayCastOutput* output)
{
    Vec3 d = input.to - input.from;
    Vec3 f = input.from - p;
    float radii = r + input.radius;

    float a = Dot(d, d);
    if (a <= epsilon)
    {
        return false;
    }

    float b = 2.0f * Dot(f, d);
    float c = Dot(f, f) - radii * radii;

    // Quadratic equation discriminant
    float discriminant = b * b - 4.0f * a * c;

    if (discriminant < 0.0f)
    {
        return false;
    }

    discriminant = SafeSqrt(discriminant);

    float t = (-b - discriminant) / (2.0f * a);
    if (0.0f <= t && t <= input.maxFraction)
    {
        output->fraction = t;
        output->normal = Normalize(f + d * t);
        return true;
    }

    return false;
}

bool RayCastCapsule(const Vec3& va, const Vec3& vb, float radius, const RayCastInput& input, RayCastOutput* output)
{
    Vec3 p1 = input.from;
    Vec3 p2 = input.to;

    Vec3 v1 = va;
    Vec3 v2 = vb;

    Vec3 d = p2 - p1;
    Vec3 e = v2 - v1;
    Vec3 pv = p1 - v1;

    float radii = radius + input.radius;

    if (Dist2(p1, ClosestPointOnSegment(v1, v2, p1)) <= radii * radii)
    {
        return false;
    }

    float dd = Dot(d, d);
    if (dd <= epsilon)
    {
        return false;
    }

    float ee = Dot(e, e);
    if (ee <= epsilon)
    {
        return RayCastSphere(v1, radius, input, output);
    }

    float ed = Dot(e, d);
    float ep = Dot(e, pv);
    float dp = Dot(d, pv);
    float pp = Dot(pv, pv);

    float a = dd * ee - ed * ed;
    float b = dp * ee - ep * ed;
    float c = pp * ee - ep * ep - radii * radii * ee;

    if (Abs(a) > epsilon)
    {
        float h = b * b - a * c;
        if (h >= 0.0f)
        {
            float t = (-b - SafeSqrt(h)) / a;
            if (0.0f <= t && t <= input.maxFraction)
            {
                float u = ep + t * ed;
                if (0.0f < u && u < ee)
                {
                    Vec3 q = p1 + d * t;
                    Vec3 closest = v1 + e * (u / ee);
                    output->fraction = t;
                    output->normal = Normalize(q - closest);
                    return true;
                }
            }
        }
    }

    RayCastOutput sphereOutput;
    bool hitA = RayCastSphere(v1, radius, input, &sphereOutput);
    bool hitB = RayCastSphere(v2, radius, input, output);

    if (hitA == false)
    {
        return hitB;
    }

    if (hitB == false || sphereOutput.fraction <= output->fraction)
    {
        *output = sphereOutput;
    }

    return true;
}

bool ShapeCast(
    const Shape* a,
    const Transform& tfA,
    const Shape* b,
    const Transform& tfB,
    const Vec3& translationA,
    const Vec3& translationB,
    ShapeCastOutput* output
)
{
    output->point.SetZero();
    output->normal.SetZero();
    output->t = 1.0f;

    float t = 0.0f;
    Vec3 n = Vec3::zero;

    const float radii = a->GetRadius() + b->GetRadius();
    const Vec3 r = translationB - translationA; // Ray vector

    Simplex simplex;

    // Get CSO support point in inverse ray direction
    int32 idA = a->GetSupport(tfA.q.RotateInv(-r));
    Vec3 pointA = Mul(tfA, a->GetVertex(idA));
    int32 idB = b->GetSupport(tfB.q.RotateInv(r));
    Vec3 pointB = Mul(tfB, b->GetVertex(idB));
    Vec3 v = pointA - pointB;

    const float target = Max(default_radius, radii - (minimum_radius - linear_slop * 0.1f));
    const float tolerance = linear_slop * 0.1f;

    const int32 maxIterations = gjk_max_iteration;
    int32 iteration = 0;

    while (iteration < maxIterations && Length(v) - target > tolerance)
    {
        MuliAssert(simplex.count < max_simplex_vertex_count);

        // Get CSO support point in search direction(-v)
        idA = a->GetSupport(tfA.q.RotateInv(-v));
        pointA = Mul(tfA, a->GetVertex(idA));
        idB = b->GetSupport(tfB.q.RotateInv(v));
        pointB = Mul(tfB, b->GetVertex(idB));
        Vec3 p = pointA - pointB; // Outer vertex of CSO

        // -v is the plane normal at p
        v.Normalize();

        // Find intersection with support plane
        float vp = Dot(v, p);
        float vr = Dot(v, r);

        // March ray by (vp - target) / vr if the new t is greater
        if (vp - target > t * vr)
        {
            if (vr <= 0.0f)
            {
                return false;
            }

            t = (vp - target) / vr;
            if (t > 1.0f)
            {
                return false;
            }

            n = -v;
            simplex.count = 0;
        }

        SupportPoint* vertex = simplex.vertices + simplex.count;
        vertex->pointA.id = idA;
        vertex->pointA.p = pointA;
        vertex->pointB.id = idB;
        vertex->pointB.p = pointB + t * r; // This effectively shifts the ray origin to the new clip plane
        vertex->point = vertex->pointA.p - vertex->pointB.p;
        vertex->weight = 1.0f;
        simplex.count += 1;

        simplex.Advance(Vec3::zero);

        if (simplex.count == max_simplex_vertex_count)
        {
            // Initial overlap
            return false;
        }

        // Update search direction
        v = simplex.GetClosestPoint();

        ++iteration;
    }

    if (iteration == 0 || t == 0.0f)
    {
        // Initial overlap
        return false;
    }

    simplex.GetWitnessPoint(&pointA, &pointB);

    if (Length2(v) > 0.0f)
    {
        n = -v;
        n.Normalize();
    }

    output->point = pointA + a->GetRadius() * n + translationA * t;
    output->normal = n;
    output->t = t;
    return true;
}

} // namespace muli3
