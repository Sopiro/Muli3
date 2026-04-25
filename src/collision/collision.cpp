#include "muli3/collision.h"

namespace muli3
{

namespace
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
    manifold->referencePoint.p = pa + normal * ra;
    manifold->contactCount = 1;
    manifold->penetrationDepth = radii - distance;
    manifold->featureFlipped = false;

    return true;
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
    detectionFunctionInitialized = true;
}

} // namespace

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

    CollideFunction* collideFunction = collideFunctionMap[a->GetType()][b->GetType()];
    if (!collideFunction)
    {
        return false;
    }

    return collideFunction(a, transformA, b, transformB, manifold);
}

} // namespace muli3
