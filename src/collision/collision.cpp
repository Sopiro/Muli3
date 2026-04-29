#include "muli3/collision.h"
#include "muli3/box.h"
#include "muli3/settings.h"

namespace muli3
{

bool detection_function_initialized = false;
CollideFunction* collide_function_map[Shape::shape_count][Shape::shape_count];

void InitializeDetectionFunctionMap();

inline SupportPoint CSOSupport(const Shape* a, const Transform& tfA, const Shape* b, const Transform& tfB, const Vec3& dir)
{
    SupportPoint supportPoint;
    supportPoint.pointA.id = a->GetSupport(tfA.q.RotateInv(dir));
    supportPoint.pointB.id = b->GetSupport(tfB.q.RotateInv(-dir));
    supportPoint.pointA.p = Mul(tfA, a->GetVertex(supportPoint.pointA.id));
    supportPoint.pointB.p = Mul(tfB, b->GetVertex(supportPoint.pointB.id));
    supportPoint.point = supportPoint.pointA.p - supportPoint.pointB.p;

    return supportPoint;
}

bool GJK(const Shape* a, const Transform& tfA, const Shape* b, const Transform& tfB, GJKResult* result)
{
    Simplex simplex;

    Vec3 direction = tfB.p - tfA.p;
    SupportPoint support = CSOSupport(a, tfA, b, tfB, direction);
    simplex.AddVertex(support);

    Vec3 save[max_simplex_vertex_count];
    int32 saveCount;

    for (int32 k = 0; k < gjk_max_iteration; ++k)
    {
        simplex.Save(save, &saveCount);
        simplex.Advance(Vec3::zero);

        if (simplex.count == max_simplex_vertex_count)
        {
            break;
        }

        direction = simplex.GetSearchDirection();

        // Simplex contains the origin
        if (Dot(direction, direction) == 0.0f)
        {
            break;
        }

        support = CSOSupport(a, tfA, b, tfB, direction);

        // Check duplicate vertices
        for (int32 i = 0; i < saveCount; ++i)
        {
            if (save[i] == support.point)
            {
                goto end;
            }
        }

        simplex.AddVertex(support);
    }

end:
    Vec3 closest = simplex.GetClosestPoint();
    float distance = Length(closest);

    result->simplex = simplex;
    result->direction = Normalize(direction);
    result->distance = distance;

    return distance < gjk_tolerance;
}

bool SphereVsSphere(
    const Shape* a, const Transform& transformA, const Shape* b, const Transform& transformB, ContactManifold* manifold
)
{
    Vec3 pa = Mul(transformA, a->GetCenter());
    Vec3 pb = Mul(transformB, b->GetCenter());
    Vec3 d = pb - pa;

    float ra = a->GetRadius();
    float rb = b->GetRadius();
    float radii = ra + rb;

    float distance2 = Length2(d);
    if (distance2 >= radii * radii)
    {
        return false;
    }

    if (!manifold)
    {
        return true;
    }

    float distance = SafeSqrt(distance2);
    Vec3 normal = distance > epsilon ? d / distance : Vec3{ 1.0f, 0.0f, 0.0f };
    if (distance <= epsilon)
    {
        distance = radii;
    }

    manifold->contactNormal = normal;
    manifold->contactPoints[0].id = 0;
    manifold->contactPoints[0].p = pb - normal * rb;
    manifold->referencePoint.id = 0;
    manifold->referencePoint.p = pa + normal * ra;
    manifold->contactCount = 1;
    manifold->penetrationDepth = radii - distance;
    manifold->featureFlipped = false;

    return true;
}

bool BoxVsSphere(
    const Shape* a, const Transform& transformA, const Shape* b, const Transform& transformB, ContactManifold* manifold
)
{
    const Box* box = (const Box*)a;

    Vec3 c = Mul(transformB, b->GetCenter());
    Vec3 center = Mul(transformA, box->GetCenter());

    Vec3 v0 = Mul(transformA, box->GetVertex(0));
    Vec3 axes[3] = {
        Mul(transformA, box->GetVertex(1)) - v0,
        Mul(transformA, box->GetVertex(2)) - v0,
        Mul(transformA, box->GetVertex(4)) - v0,
    };

    float extents[3];
    for (int32 i = 0; i < 3; ++i)
    {
        extents[i] = axes[i].Normalize() * 0.5f;
    }

    Vec3 d = c - center;
    Vec3 closest = center;
    float projections[3];
    bool inside = true;
    for (int32 i = 0; i < 3; ++i)
    {
        projections[i] = Dot(d, axes[i]);
        float clamped = Clamp(projections[i], -extents[i], extents[i]);
        if (clamped != projections[i])
        {
            inside = false;
        }
        closest += axes[i] * clamped;
    }

    float radii = a->GetRadius() + b->GetRadius();
    Vec3 normal = c - closest;
    float separation = normal.Normalize();
    int32 contactID = 0;

    if (inside)
    {
        float minSeparation = max_float;
        int32 axisIndex = 0;
        float sign = 1.0f;

        for (int32 i = 0; i < 3; ++i)
        {
            float positive = extents[i] - projections[i];
            if (positive < minSeparation)
            {
                minSeparation = positive;
                axisIndex = i;
                sign = 1.0f;
            }

            float negative = extents[i] + projections[i];
            if (negative < minSeparation)
            {
                minSeparation = negative;
                axisIndex = i;
                sign = -1.0f;
            }
        }

        normal = axes[axisIndex] * sign;
        closest += normal * minSeparation;
        separation = -minSeparation;
        contactID = axisIndex * 2 + (sign > 0.0f ? 1 : 0);
    }
    else if (separation > radii)
    {
        return false;
    }
    else
    {
        for (int32 i = 0; i < 3; ++i)
        {
            if (projections[i] >= extents[i] - linear_slop)
            {
                contactID |= 1 << (i * 2);
            }
            else if (projections[i] <= -extents[i] + linear_slop)
            {
                contactID |= 1 << (i * 2 + 1);
            }
        }
    }

    if (!manifold)
    {
        return true;
    }

    if (separation <= epsilon && !inside)
    {
        normal = NormalizeSafe(c - center);
        if (Length2(normal) <= epsilon)
        {
            normal = Vec3{ 1.0f, 0.0f, 0.0f };
        }
    }

    manifold->contactNormal = normal;
    manifold->contactPoints[0].id = contactID;
    manifold->contactPoints[0].p = c - normal * b->GetRadius();
    manifold->referencePoint.id = contactID;
    manifold->referencePoint.p = closest + normal * a->GetRadius();
    manifold->contactCount = 1;
    manifold->penetrationDepth = radii - separation;
    manifold->featureFlipped = false;

    return true;
}

bool ConvexVsConvex(const Shape* a, const Transform& tfA, const Shape* b, const Transform& tfB, ContactManifold* manifold)
{
    return false;
}

void InitializeDetectionFunctionMap()
{
    if (detection_function_initialized)
    {
        return;
    }

    collide_function_map[Shape::sphere][Shape::sphere] = SphereVsSphere;
    collide_function_map[Shape::box][Shape::sphere] = BoxVsSphere;
    collide_function_map[Shape::box][Shape::box] = ConvexVsConvex;

    detection_function_initialized = true;
}

bool Collide(const Shape* a, const Transform& tfA, const Shape* b, const Transform& tfB, ContactManifold* manifold)
{
    MuliAssert(a != nullptr);
    MuliAssert(b != nullptr);

    if (!detection_function_initialized)
    {
        InitializeDetectionFunctionMap();
    }

    if (manifold)
    {
        *manifold = ContactManifold{};
    }

    Shape::Type shapeA = a->GetType();
    Shape::Type shapeB = b->GetType();

    if (shapeB > shapeA)
    {
        MuliAssert(collide_function_map[shapeB][shapeA] != nullptr);

        bool collide = collide_function_map[shapeB][shapeA](b, tfB, a, tfA, manifold);
        manifold->featureFlipped = !manifold->featureFlipped;

        return collide;
    }
    else
    {
        MuliAssert(collide_function_map[shapeA][shapeB] != nullptr);

        return collide_function_map[shapeA][shapeB](a, tfA, b, tfB, manifold);
    }
}

} // namespace muli3
