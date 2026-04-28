#include "muli3/collision.h"
#include "muli3/box.h"
#include "muli3/settings.h"

namespace muli3
{

bool detectionFunctionInitialized = false;
CollideFunction* collideFunctionMap[ShapeType::shape_count][ShapeType::shape_count];

void InitializeDetectionFunctionMap();

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
    for (int32 i = 0; i < ShapeType::shape_count; ++i)
    {
        for (int32 j = 0; j < ShapeType::shape_count; ++j)
        {
            collideFunctionMap[i][j] = nullptr;
        }
    }

    collideFunctionMap[ShapeType::sphere][ShapeType::sphere] = SphereVsSphere;
    collideFunctionMap[ShapeType::box][ShapeType::sphere] = BoxVsSphere;
    collideFunctionMap[ShapeType::box][ShapeType::box] = ConvexVsConvex;
    detectionFunctionInitialized = true;
}

bool Collide(const Shape* a, const Transform& transformA, const Shape* b, const Transform& transformB, ContactManifold* manifold)
{
    MuliAssert(a != nullptr);
    MuliAssert(b != nullptr);

    if (!detectionFunctionInitialized)
    {
        InitializeDetectionFunctionMap();
    }

    if (manifold)
    {
        *manifold = ContactManifold{};
    }

    ShapeType shapeA = a->GetType();
    ShapeType shapeB = b->GetType();

    if (shapeB > shapeA)
    {
        CollideFunction* collideFunction = collideFunctionMap[shapeB][shapeA];
        MuliAssert(collideFunction != nullptr);
        if (!collideFunction)
        {
            return false;
        }

        bool collide = collideFunction(b, transformB, a, transformA, manifold);
        if (manifold && collide)
        {
            manifold->featureFlipped = !manifold->featureFlipped;
        }
        return collide;
    }

    CollideFunction* collideFunction = collideFunctionMap[shapeA][shapeB];
    MuliAssert(collideFunction != nullptr);
    if (!collideFunction)
    {
        return false;
    }

    return collideFunction(a, transformA, b, transformB, manifold);
}

} // namespace muli3
